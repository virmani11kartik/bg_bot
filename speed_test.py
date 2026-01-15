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
