#!/usr/bin/env python3
import can, struct, time, statistics

CHANNEL="can0"
INTERFACE="socketcan"

# If None: auto-detect by SDO ping scan 1..10
NODE = 2

CMD_PER_WHEEL_RPM = 3814.0
TARGET_WHEEL_RPM = -80.0
HOLD_SEC = 5.0

RAMP_STEP_CMD = 15000
RAMP_DT = 0.12

PRINT_606C = True
TELEM_DT = 0.5

def nmt(bus, cmd, node=0):
    # NMT: arbitration 0x000, data [cmd, node]
    bus.send(can.Message(arbitration_id=0x000, data=[cmd & 0xFF, node & 0xFF], is_extended_id=False))

def sdo_ping(bus, node, timeout=0.2):
    req = 0x600 + node
    resp = 0x580 + node
    bus.send(can.Message(arbitration_id=req, data=[0x40,0x61,0x60,0x00,0,0,0,0], is_extended_id=False))
    t0=time.time()
    while time.time()-t0 < timeout:
        m = bus.recv(timeout=timeout)
        if m and m.arbitration_id == resp and len(m.data) >= 8:
            return True
    return False

def autodetect_node(bus, lo=1, hi=10):
    for n in range(lo, hi+1):
        if sdo_ping(bus, n, 0.2):
            return n
    return None

class Ctrl:
    def __init__(self, bus, node):
        self.bus=bus
        self.node=node

    def _wait(self, idx, sub, timeout=0.5):
        cob=0x580+self.node
        t0=time.time()
        while time.time()-t0 < timeout:
            m=self.bus.recv(timeout=timeout)
            if not m or m.arbitration_id!=cob or len(m.data)<8: 
                continue
            if m.data[1]==(idx&0xFF) and m.data[2]==((idx>>8)&0xFF) and m.data[3]==sub:
                return m
        return None

    def r_i32(self, idx, sub):
        self.bus.send(can.Message(arbitration_id=0x600+self.node,
                                  data=[0x40, idx&0xFF,(idx>>8)&0xFF,sub,0,0,0,0],
                                  is_extended_id=False))
        r=self._wait(idx,sub,0.5)
        if not r or r.data[0]==0x80: return None
        return int.from_bytes(bytes(r.data[4:8]), "little", signed=True)

    def w_u16(self, idx, sub, val):
        self.bus.send(can.Message(arbitration_id=0x600+self.node,
                                  data=[0x2B, idx&0xFF,(idx>>8)&0xFF,sub] + list(struct.pack("<H",val)) + [0,0],
                                  is_extended_id=False))
        r=self._wait(idx,sub,0.5)
        if r and r.data[0]==0x80:
            abort=int.from_bytes(bytes(r.data[4:8]),"little")
            raise RuntimeError(f"SDO abort write {idx:04X}:{sub:02X} 0x{abort:08X}")

    def w_i32(self, idx, sub, val):
        self.bus.send(can.Message(arbitration_id=0x600+self.node,
                                  data=[0x23, idx&0xFF,(idx>>8)&0xFF,sub] + list(struct.pack("<i",val)),
                                  is_extended_id=False))
        r=self._wait(idx,sub,0.5)
        if r and r.data[0]==0x80:
            abort=int.from_bytes(bytes(r.data[4:8]),"little")
            raise RuntimeError(f"SDO abort write {idx:04X}:{sub:02X} 0x{abort:08X}")

    def controlword(self, v): self.w_u16(0x6040,0,v)

    def set_mode_velocity(self):
        self.bus.send(can.Message(arbitration_id=0x600+self.node,
                                  data=[0x2F,0x60,0x60,0x00,0x03,0,0,0],
                                  is_extended_id=False))
        r=self._wait(0x6060,0,0.5)
        if r and r.data[0]==0x80:
            abort=int.from_bytes(bytes(r.data[4:8]),"little")
            raise RuntimeError(f"SDO abort write 6060:00 0x{abort:08X}")

    def cmd(self, v): self.w_i32(0x60FF,0,v)
    def sw(self): return self.r_i32(0x6041,0)
    def md(self): return self.r_i32(0x6061,0)
    def av(self): return self.r_i32(0x606C,0)

    def enable(self):
        self.controlword(0x0006); time.sleep(0.1)
        self.set_mode_velocity(); time.sleep(0.1)
        self.controlword(0x0007); time.sleep(0.1)
        self.controlword(0x000F); time.sleep(0.2)

    def disable(self):
        self.controlword(0x0007); time.sleep(0.05)
        self.controlword(0x0006); time.sleep(0.05)

def wheel_rpm_to_cmd(rpm):
    return int(round(rpm * CMD_PER_WHEEL_RPM))

def ramp(ctrl, start, target, step, dt):
    v=start
    if target >= start:
        while v < target:
            v=min(v+step, target); ctrl.cmd(v); time.sleep(dt)
    else:
        while v > target:
            v=max(v-step, target); ctrl.cmd(v); time.sleep(dt)

def main():
    bus = can.Bus(channel=CHANNEL, interface=INTERFACE)

    try:
        # Bring network up after power-cycle
        nmt(bus, 0x80, 0)  # pre-op all
        time.sleep(0.1)
        nmt(bus, 0x01, 0)  # start all
        time.sleep(0.1)

        node = NODE
        if node is None:
            # node = autodetect_node(bus, 1, 10)
            node = NODE
            if node is None:
                raise RuntimeError("No SDO responses from nodes 1..10. Node ID changed or drive not on CANopen/NMT.")
        print("Using NODE =", node)

        ctrl = Ctrl(bus, node)

        target_cmd = wheel_rpm_to_cmd(TARGET_WHEEL_RPM)
        print(f"TARGET_WHEEL_RPM={TARGET_WHEEL_RPM:.2f} -> target_cmd={target_cmd} (CMD_PER_WHEEL_RPM={CMD_PER_WHEEL_RPM:.1f})")

        ctrl.enable()

        md = ctrl.md()
        sw = ctrl.sw()
        print("Mode display (6061):", md)
        print("Statusword (6041):  ", sw)

        if md is None or sw is None:
            raise RuntimeError("Drive still not answering SDO reads after enable. Check node ID/NMT/bitrate/drive state.")

        # run
        ctrl.cmd(0); time.sleep(0.1)
        print("Ramping up...")
        ramp(ctrl, 0, target_cmd, RAMP_STEP_CMD, RAMP_DT)

        print(f"Holding for {HOLD_SEC:.1f}s...")
        samples=[]
        t0=time.time()
        while time.time()-t0 < HOLD_SEC:
            if PRINT_606C:
                av = ctrl.av()
                print("  606C actual vel:", av)
                if av is not None: samples.append(av)
            time.sleep(TELEM_DT)

        if PRINT_606C and samples:
            print(f"606C avg/std: {statistics.mean(samples):.1f} / {statistics.pstdev(samples):.1f}")

        print("Ramping down...")
        ramp(ctrl, target_cmd, 0, RAMP_STEP_CMD, max(0.08, RAMP_DT*0.9))

    finally:
        # safety stop
        try:
            if 'ctrl' in locals():
                ctrl.cmd(0); time.sleep(0.1)
                ctrl.disable()
        except:
            pass
        bus.shutdown()

    print("Done.")

if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# import can, struct, time, statistics

# CHANNEL="can0"
# INTERFACE="socketcan"
# NODE=1   # keep fixed since you confirmed node 1 responds

# CMD_PER_WHEEL_RPM = 3814.0

# TARGET_WHEEL_RPM = -50.0
# HOLD_SEC = 5.0

# RAMP_STEP_CMD = 15000
# RAMP_DT = 0.12

# PRINT_606C = True
# TELEM_DT = 0.5

# # ---------- CANopen helpers ----------
# def nmt(bus, cmd, node=0):
#     bus.send(can.Message(arbitration_id=0x000, data=[cmd & 0xFF, node & 0xFF], is_extended_id=False))

# def wait_sdo(bus, node, idx, sub, timeout=0.5):
#     cob = 0x580 + node
#     t0 = time.time()
#     while time.time()-t0 < timeout:
#         m = bus.recv(timeout=timeout)
#         if not m or m.arbitration_id != cob or len(m.data) < 8:
#             continue
#         if m.data[1]==(idx&0xFF) and m.data[2]==((idx>>8)&0xFF) and m.data[3]==sub:
#             return m
#     return None

# def sdo_read(bus, node, idx, sub):
#     bus.send(can.Message(arbitration_id=0x600+node,
#                          data=[0x40, idx&0xFF,(idx>>8)&0xFF,sub,0,0,0,0],
#                          is_extended_id=False))
#     r = wait_sdo(bus, node, idx, sub, 0.5)
#     if not r: return ("TIMEOUT", None)

#     if r.data[0] == 0x80:
#         abort = int.from_bytes(bytes(r.data[4:8]), "little")
#         return ("ABORT", abort)

#     # Determine size from command specifier
#     cs = r.data[0]
#     if cs in (0x4F,):  # 1 byte
#         return ("OK", int.from_bytes(bytes([r.data[4]]), "little", signed=False))
#     if cs in (0x4B,):  # 2 bytes
#         return ("OK", int.from_bytes(bytes(r.data[4:6]), "little", signed=False))
#     if cs in (0x43, 0x47):  # 4 bytes
#         return ("OK", int.from_bytes(bytes(r.data[4:8]), "little", signed=True))

#     # fallback: treat as 4 bytes
#     return ("OK", int.from_bytes(bytes(r.data[4:8]), "little", signed=True))

# def sdo_write_u16(bus, node, idx, sub, val):
#     bus.send(can.Message(arbitration_id=0x600+node,
#                          data=[0x2B, idx&0xFF,(idx>>8)&0xFF,sub] + list(struct.pack("<H",val)) + [0,0],
#                          is_extended_id=False))
#     r = wait_sdo(bus, node, idx, sub, 0.5)
#     if r and r.data[0]==0x80:
#         abort = int.from_bytes(bytes(r.data[4:8]), "little")
#         raise RuntimeError(f"SDO abort write {idx:04X}:{sub:02X} 0x{abort:08X}")

# def sdo_write_i32(bus, node, idx, sub, val):
#     bus.send(can.Message(arbitration_id=0x600+node,
#                          data=[0x23, idx&0xFF,(idx>>8)&0xFF,sub] + list(struct.pack("<i",val)),
#                          is_extended_id=False))
#     r = wait_sdo(bus, node, idx, sub, 0.5)
#     if r and r.data[0]==0x80:
#         abort = int.from_bytes(bytes(r.data[4:8]), "little")
#         raise RuntimeError(f"SDO abort write {idx:04X}:{sub:02X} 0x{abort:08X}")

# # ---------- DS402 helpers ----------
# def controlword(bus, v): sdo_write_u16(bus, NODE, 0x6040, 0x00, v)

# def set_mode_velocity(bus):
#     # 6060 int8 = 3 (Profile Velocity)
#     bus.send(can.Message(arbitration_id=0x600+NODE,
#                          data=[0x2F,0x60,0x60,0x00,0x03,0,0,0],
#                          is_extended_id=False))
#     r = wait_sdo(bus, NODE, 0x6060, 0x00, 0.5)
#     if r and r.data[0]==0x80:
#         abort = int.from_bytes(bytes(r.data[4:8]), "little")
#         raise RuntimeError(f"SDO abort write 6060:00 0x{abort:08X}")

# def read_statusword(bus):
#     st, val = sdo_read(bus, NODE, 0x6041, 0x00)
#     return None if st != "OK" else val

# def read_mode_display(bus):
#     st, val = sdo_read(bus, NODE, 0x6061, 0x00)
#     return None if st != "OK" else val

# def read_actual_velocity(bus):
#     st, val = sdo_read(bus, NODE, 0x606C, 0x00)
#     return None if st != "OK" else val

# def read_position(bus):
#     st, val = sdo_read(bus, NODE, 0x6064, 0x00)
#     return None if st != "OK" else val

# def cmd_velocity(bus, cmd):
#     sdo_write_i32(bus, NODE, 0x60FF, 0x00, int(cmd))

# def sw_fault(sw):        return bool(sw & (1<<3))   # bit3
# def sw_op_enabled(sw):   return bool(sw & (1<<2))   # bit2
# def sw_quickstop(sw):    return bool(sw & (1<<5))   # bit5

# def decode_sw(sw):
#     if sw is None: return "None"
#     bits=[]
#     if sw & (1<<0): bits.append("RTSO")
#     if sw & (1<<1): bits.append("SO")
#     if sw & (1<<2): bits.append("OE")
#     if sw & (1<<3): bits.append("FAULT")
#     if sw & (1<<5): bits.append("QuickStop")
#     if sw & (1<<6): bits.append("SwitchOnDisabled")
#     if sw & (1<<10): bits.append("TargetReached")
#     return f"0x{sw:04X} " + "|".join(bits)

# def clear_fault_if_needed(bus):
#     sw = read_statusword(bus)
#     print("Initial Statusword:", decode_sw(sw))

#     if sw is None:
#         raise RuntimeError("No statusword response")

#     if sw_fault(sw):
#         print("Fault detected -> sending Fault Reset (6040=0x0080)")
#         controlword(bus, 0x0080)
#         time.sleep(0.2)

#         # Wait for fault bit to clear
#         for _ in range(20):
#             sw = read_statusword(bus)
#             print("  After reset SW:", decode_sw(sw))
#             if sw is not None and not sw_fault(sw):
#                 break
#             time.sleep(0.1)

#         if sw is None or sw_fault(sw):
#             raise RuntimeError("Fault did not clear. Likely hardware/estop/STO/brake/interlock preventing reset.")

# def enable_drive(bus):
#     controlword(bus, 0x0006); time.sleep(0.1)
#     set_mode_velocity(bus);    time.sleep(0.1)
#     controlword(bus, 0x0007); time.sleep(0.1)
#     controlword(bus, 0x000F); time.sleep(0.2)

#     sw = read_statusword(bus)
#     print("Enabled Statusword:", decode_sw(sw))
#     if sw is None or not sw_op_enabled(sw) or sw_fault(sw):
#         raise RuntimeError("Not OperationEnabled after enable sequence. Interlock or fault still active.")

# def wheel_rpm_to_cmd(wheel_rpm):
#     return int(round(wheel_rpm * CMD_PER_WHEEL_RPM))

# def ramp(bus, start, target, step, dt):
#     v = start
#     if target >= start:
#         while v < target:
#             v = min(v + step, target)
#             cmd_velocity(bus, v)
#             time.sleep(dt)
#     else:
#         while v > target:
#             v = max(v - step, target)
#             cmd_velocity(bus, v)
#             time.sleep(dt)

# def main():
#     bus = can.Bus(channel=CHANNEL, interface=INTERFACE)
#     target_cmd = wheel_rpm_to_cmd(TARGET_WHEEL_RPM)

#     print(f"TARGET_WHEEL_RPM={TARGET_WHEEL_RPM:.2f} -> target_cmd={target_cmd} (CMD_PER_WHEEL_RPM={CMD_PER_WHEEL_RPM:.1f})")

#     try:
#         # NMT bring-up (helps after power cycle)
#         nmt(bus, 0x80, 0); time.sleep(0.1)  # pre-op all
#         nmt(bus, 0x01, 0); time.sleep(0.1)  # start all

#         md = read_mode_display(bus)
#         print("Mode display (6061):", md)

#         # If drive is faulted, clear fault first
#         clear_fault_if_needed(bus)

#         # Now enable properly
#         enable_drive(bus)

#         # Start at 0
#         cmd_velocity(bus, 0); time.sleep(0.1)

#         print("Ramping up...")
#         ramp(bus, 0, target_cmd, RAMP_STEP_CMD, RAMP_DT)

#         # Verify motion using position as ground truth
#         p0 = read_position(bus)
#         print("Position start (6064):", p0)

#         print(f"Holding for {HOLD_SEC:.1f}s...")
#         samples=[]
#         t0=time.time()
#         while time.time()-t0 < HOLD_SEC:
#             av = read_actual_velocity(bus) if PRINT_606C else None
#             p  = read_position(bus)
#             print("  606C:", av, "   6064:", p)
#             if av is not None and av != -1:
#                 samples.append(av)
#             time.sleep(TELEM_DT)

#         if samples:
#             print(f"606C avg/std: {statistics.mean(samples):.1f} / {statistics.pstdev(samples):.1f}")
#         else:
#             print("606C not usable (stays -1). Using 6064 delta to confirm motion.")

#         print("Ramping down...")
#         ramp(bus, target_cmd, 0, RAMP_STEP_CMD, max(0.08, RAMP_DT*0.9))

#     finally:
#         # safety stop
#         try:
#             cmd_velocity(bus, 0); time.sleep(0.1)
#             controlword(bus, 0x0007); time.sleep(0.05)
#             controlword(bus, 0x0006); time.sleep(0.05)
#         except:
#             pass
#         bus.shutdown()

#     print("Done.")

# if __name__ == "__main__":
#     main()
