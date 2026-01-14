#!/usr/bin/env python3
"""
Dual Motor Control - Differential Drive Robot
"""
import can
import struct
import time

bus = can.Bus(channel='can0', interface='socketcan')

CMD_PER_WHEEL_RPM = 3814.0

def send_cmd_motor(node, ctrl, vel=0):
    """Send RPDO command + SYNC to specific motor"""
    rpdo_id = 0x500 + node  # 0x501 for node 1, 0x502 for node 2
    msg = can.Message(
        arbitration_id=rpdo_id,
        data=struct.pack('<HiH', ctrl, vel, 0),
        is_extended_id=False
    )
    bus.send(msg)
    time.sleep(0.001)

def send_sync():
    """Send SYNC to all motors"""
    sync = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
    bus.send(sync)
    time.sleep(0.008)

def set_velocities(left_rpm, right_rpm):
    """Set velocities for both motors simultaneously"""
    left_cmd = int(round(left_rpm * CMD_PER_WHEEL_RPM))
    right_cmd = int(round(right_rpm * CMD_PER_WHEEL_RPM))
    
    # Send commands to both motors
    send_cmd_motor(1, 0x000F, left_cmd)   # Left motor
    send_cmd_motor(2, 0x000F, right_cmd)  # Right motor
    
    # Single SYNC triggers both
    send_sync()

print("="*70)
print("DIFFERENTIAL DRIVE ROBOT CONTROL")
print("="*70)

# Test movements
movements = [
    (50, -50, 2, "Forward"),
    (0, 0, 1, "Stop"),
    (-50, 50, 2, "Backward"),
    (0, 0, 1, "Stop"),
    (50, 50, 2, "Rotate Right"),
    (0, 0, 1, "Stop"),
    (-50, -50, 2, "Rotate Left"),
    (0, 0, 1, "Stop"),
]

print("\nExecuting movement sequence...\n")

for left_rpm, right_rpm, duration, description in movements:
    print(f"→ {description}: L={left_rpm} R={right_rpm} RPM")
    
    start = time.time()
    while time.time() - start < duration:
        set_velocities(left_rpm, right_rpm)
    
    print(f"  Complete\n")

# Stop both motors
print("Stopping both motors...")
for _ in range(30):
    set_velocities(0, 0)

# Disable both motors
for _ in range(10):
    send_cmd_motor(1, 0x0006, 0)
    send_cmd_motor(2, 0x0006, 0)
    send_sync()

bus.shutdown()
print("\n✓ Robot control test complete!")