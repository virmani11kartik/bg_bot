import can
import struct
import time

# Setup
bus = can.Bus(channel='can0', interface='socketcan')
MOTOR_1_ID = 0x601

def send_nmt(node_id, command):
    """Send NMT command"""
    msg = can.Message(arbitration_id=0x000, data=[command, node_id], is_extended_id=False)
    bus.send(msg)
    print(f"Sent NMT: command=0x{command:02X}, node={node_id}")

def send_sdo(node_id, index, subindex, data):
    """Send SDO write"""
    cob_id = 0x600 + node_id
    payload = [0x23, index & 0xFF, (index >> 8) & 0xFF, subindex] + list(data)
    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    bus.send(msg)
    print(f"SDO Write: 0x{index:04X}.{subindex} = {data.hex()}")

def read_statusword():
    """Read current statusword"""
    bus.send(can.Message(
        arbitration_id=MOTOR_1_ID,
        data=[0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0],
        is_extended_id=False
    ))
    time.sleep(0.05)
    msg = bus.recv(timeout=0.5)
    if msg and len(msg.data) >= 8:
        status = struct.unpack('<H', bytes(msg.data[4:6]))[0]
        return status
    return None

def send_controlword(value):
    """Send controlword via SDO"""
    data = struct.pack('<I', value)
    send_sdo(1, 0x6040, 0x00, data)

print("=" * 70)
print("FAULT RESET SEQUENCE")
print("=" * 70)

# 1. Read initial status
print("\n1. Reading initial statusword...")
status = read_statusword()
if status:
    print(f"   Current status: 0x{status:04X}")

# 2. NMT Reset
print("\n2. Sending NMT Reset Communication...")
send_nmt(1, 0x82)
time.sleep(1)

# 3. NMT Start
print("\n3. Sending NMT Start...")
send_nmt(1, 0x01)
time.sleep(0.5)

# 4. Fault Reset (controlword 0x0080)
print("\n4. Sending FAULT RESET (0x0080)...")
send_controlword(0x0080)
time.sleep(0.5)

# Check status
status = read_statusword()
if status:
    print(f"   Status after fault reset: 0x{status:04X}")

# 5. Disable voltage (controlword 0x0000)
print("\n5. Sending DISABLE (0x0000)...")
send_controlword(0x0000)
time.sleep(0.3)

# 6. Shutdown (controlword 0x0006)
print("\n6. Sending SHUTDOWN (0x0006)...")
send_controlword(0x0006)
time.sleep(0.3)

status = read_statusword()
if status:
    print(f"   Status after shutdown: 0x{status:04X}")

# 7. Switch On (controlword 0x0007)
print("\n7. Sending SWITCH ON (0x0007)...")
send_controlword(0x0007)
time.sleep(0.3)

# 8. Enable Operation (controlword 0x000F)
print("\n8. Sending ENABLE OPERATION (0x000F)...")
send_controlword(0x000F)
time.sleep(0.3)

# Final check
status = read_statusword()
if status:
    print(f"\n{'=' * 70}")
    print(f"FINAL STATUS: 0x{status:04X}")
    
    if status & 0x0008:
        print(" FAULT still present!")
    if status & 0x0004:
        print("✓ Operation enabled!")
    else:
        print("✗ Operation NOT enabled")
    
    print(f"{'=' * 70}")

bus.shutdown()
