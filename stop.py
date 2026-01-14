import can, struct, time

bus = can.Bus(channel="can0", interface="socketcan")

def sdo_write(node, idx, sub, data4):
    cob = 0x600 + node
    msg = can.Message(arbitration_id=cob,
                      data=[0x23, idx & 0xFF, (idx>>8)&0xFF, sub] + list(data4),
                      is_extended_id=False)
    bus.send(msg)
    # optionally read ack:
    bus.recv(timeout=0.2)

def set_target_velocity(node, rpm):
    sdo_write(node, 0x60FF, 0x00, struct.pack("<i", rpm))

def set_controlword(node, cw):
    sdo_write(node, 0x6040, 0x00, struct.pack("<H", cw) + b"\x00\x00")

NODE = 1

# 1) command 0 velocity
set_target_velocity(NODE, 0)
time.sleep(0.1)

# 2) disable operation (Operation enabled -> Switched on)
set_controlword(NODE, 0x0007)
time.sleep(0.1)

# 3) shutdown (Switched on -> Ready to switch on)
set_controlword(NODE, 0x0006)
time.sleep(0.1)

# 4) optional: disable voltage (if your drive accepts it)
set_controlword(NODE, 0x0000)

bus.shutdown()
print("Stop sequence sent.")
