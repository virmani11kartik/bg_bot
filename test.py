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
        print("⚠️  FAULT still present!")
    if status & 0x0004:
        print("✓ Operation enabled!")
    else:
        print("✗ Operation NOT enabled")
    
    print(f"{'=' * 70}")

bus.shutdown()

# import can
# import struct
# import time

# bus = can.Bus(channel='can0', interface='socketcan')
# MOTOR_1_ID = 0x601

# def send_sdo_write(node_id, index, subindex, data):
#     """Send SDO write"""
#     cob_id = 0x600 + node_id
#     payload = [0x23, index & 0xFF, (index >> 8) & 0xFF, subindex] + list(data)
#     msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
#     bus.send(msg)
#     time.sleep(0.1)

# def send_sdo_read(node_id, index, subindex):
#     """Send SDO read request"""
#     cob_id = 0x600 + node_id
#     payload = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
#     msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
#     bus.send(msg)
#     time.sleep(0.1)
    
#     # Read response
#     msg = bus.recv(timeout=0.5)
#     if msg and len(msg.data) >= 8:
#         if msg.data[0] == 0x43:  # SDO read response (1 byte)
#             return msg.data[4]
#         elif msg.data[0] == 0x4B:  # SDO read response (2 bytes)
#             return struct.unpack('<H', bytes(msg.data[4:6]))[0]
#         elif msg.data[0] == 0x43 or msg.data[0] == 0x47:  # 4 bytes
#             return struct.unpack('<i', bytes(msg.data[4:8]))[0]
#     return None

# def read_statusword():
#     """Read statusword"""
#     return send_sdo_read(1, 0x6041, 0x00)

# def send_controlword(value):
#     """Send controlword"""
#     data = struct.pack('<I', value)
#     send_sdo_write(1, 0x6040, 0x00, data)

# def set_mode_of_operation(mode):
#     """Set operating mode"""
#     data = struct.pack('<B', mode) + b'\x00\x00\x00'
#     send_sdo_write(1, 0x6060, 0x00, data)

# def set_velocity(rpm):
#     """Set target velocity"""
#     data = struct.pack('<i', rpm)
#     send_sdo_write(1, 0x6042, 0x00, data)

# print("=" * 70)
# print("MOTOR MODE AND VELOCITY CONTROL")
# print("=" * 70)

# # 1. Check current mode
# print("\n1. Reading current mode of operation...")
# current_mode = send_sdo_read(1, 0x6061, 0x00)  # Mode display
# print(f"   Current mode: {current_mode}")

# # 2. First, disable operation
# print("\n2. Disabling operation...")
# send_controlword(0x0006)  # Shutdown
# time.sleep(0.3)

# # 3. Set Profile Velocity Mode (mode 3)
# print("\n3. Setting Profile Velocity Mode (0x03)...")
# set_mode_of_operation(0x03)
# time.sleep(0.5)

# # Verify mode was set
# new_mode = send_sdo_read(1, 0x6061, 0x00)
# print(f"   Mode after setting: {new_mode}")

# # 4. Re-enable operation
# print("\n4. Re-enabling operation...")
# send_controlword(0x0007)  # Switch on
# time.sleep(0.3)
# send_controlword(0x000F)  # Enable operation
# time.sleep(0.5)

# status = read_statusword()
# # print(f"   Status: 0x{status:04X if status else 0:04X}")

# # 5. Test velocities
# print("\n5. Testing velocity control...")
# test_speeds = [
#     (100, 3),
#     (200, 3),
#     (-100, 3),
#     (0, 2)
# ]

# for rpm, duration in test_speeds:
#     print(f"\n   → Setting {rpm} RPM")
#     set_velocity(rpm)
    
#     # Read back to confirm
#     actual_vel = send_sdo_read(1, 0x6042, 0x00)
#     print(f"     Target velocity readback: {actual_vel}")
    
#     time.sleep(duration)
    
#     status = read_statusword()
#     # print(f"     Status: 0x{status:04X if status else 0:04X}")

# print("\n" + "=" * 70)
# print("STOPPING")
# print("=" * 70)
# set_velocity(0)
# time.sleep(1)

# bus.shutdown()
# print("✓ Complete!")


# import can
# import struct
# import time

# bus = can.Bus(channel='can0', interface='socketcan')
# MOTOR_1_ID = 0x601

# def send_sdo_write(node_id, index, subindex, data):
#     """Send SDO write"""
#     cob_id = 0x600 + node_id
#     payload = [0x23, index & 0xFF, (index >> 8) & 0xFF, subindex] + list(data)
#     msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
#     bus.send(msg)
#     time.sleep(0.05)

# def send_sdo_read(node_id, index, subindex):
#     """Send SDO read request and parse response"""
#     cob_id = 0x600 + node_id
#     payload = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
#     msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
#     bus.send(msg)
#     time.sleep(0.05)
    
#     msg = bus.recv(timeout=0.5)
#     if msg and len(msg.data) >= 8:
#         cmd = msg.data[0]
        
#         # SDO response codes:
#         # 0x42 = 2 bytes (statusword 0x6041, mode 0x6061)
#         # 0x43 = 4 bytes
#         # 0x47 = 4 bytes (velocity 0x6042)
#         # 0x4B = 2 bytes
#         # 0x4F = 1 byte
        
#         if cmd == 0x42 or cmd == 0x4B:  # 2-byte response
#             return struct.unpack('<H', bytes(msg.data[4:6]))[0]
#         elif cmd == 0x43 or cmd == 0x47:  # 4-byte response
#             return struct.unpack('<i', bytes(msg.data[4:8]))[0]
#         elif cmd == 0x4F:  # 1-byte response
#             return msg.data[4]
#     return None

# def send_controlword(value):
#     """Send controlword"""
#     data = struct.pack('<I', value)
#     send_sdo_write(1, 0x6040, 0x00, data)

# def set_velocity(rpm):
#     """Set target velocity"""
#     data = struct.pack('<i', rpm)
#     send_sdo_write(1, 0x6042, 0x00, data)

# print("=" * 70)
# print("MOTOR VELOCITY CONTROL - FIXED VERSION")
# print("=" * 70)

# # Check current status
# print("\n[Checking motor state...]")
# status = send_sdo_read(1, 0x6041, 0x00)
# mode = send_sdo_read(1, 0x6061, 0x00)

# print(f"  Status: 0x{status:04X}" if status else "  Status: No response")
# print(f"  Mode: {mode}" if mode else "  Mode: No response")

# if status and (status & 0x0004):
#     print("  ✓ Motor operational!")
# else:
#     print("  ⚠️  Motor not operational, re-enabling...")
#     send_controlword(0x0006)  # Shutdown
#     time.sleep(0.2)
#     send_controlword(0x0007)  # Switch on
#     time.sleep(0.2)
#     send_controlword(0x000F)  # Enable operation
#     time.sleep(0.3)
    
#     status = send_sdo_read(1, 0x6041, 0x00)
#     print(f"  New status: 0x{status:04X}" if status else "  Still no response")

# print("\n" + "=" * 70)
# print("VELOCITY TEST SEQUENCE")
# print("=" * 70)

# # Test different speeds
# test_sequence = [
#     (50, 2, "Slow (50 RPM)"),
#     (100, 2, "Medium (100 RPM)"),
#     (200, 2, "Fast (200 RPM)"),
#     (0, 1, "Stop"),
#     (-50, 2, "Reverse slow (-50 RPM)"),
#     (-150, 2, "Reverse fast (-150 RPM)"),
#     (0, 1, "Final stop")
# ]

# for rpm, duration, description in test_sequence:
#     print(f"\n{description}")
#     set_velocity(rpm)
    
#     # Confirm command
#     readback = send_sdo_read(1, 0x6042, 0x00)
#     if readback is not None:
#         print(f"  ✓ Target set: {readback} RPM")
#     else:
#         print(f"  ⚠️  Could not confirm")
    
#     time.sleep(duration)

# print("\n" + "=" * 70)
# print("✓ TEST COMPLETE")
# print("=" * 70)

# # Stop motor
# set_velocity(0)
# time.sleep(0.5)

# bus.shutdown()