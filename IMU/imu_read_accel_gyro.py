import serial
import struct
import time

PORT = "/dev/ttymxc4"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

def send_packet(mid, data=b''):
    pkt = bytearray([0xFA, 0xFF, mid, len(data)])
    pkt.extend(data)
    checksum = (256 - (sum(pkt[1:]) & 0xFF)) & 0xFF
    pkt.append(checksum)
    ser.write(pkt)
    time.sleep(0.1)

# 1) Go to config mode
send_packet(0x30)

# 2) Set output configuration: 
#    - Acceleration @ 100 Hz (0x4020, 0x0064) 
#    - Rate of Turn @ 100 Hz (0x8020, 0x0064)
cfg = bytes.fromhex("40 20 00 64 80 20 00 64")
send_packet(0xC0, cfg)

# 3) Go to measurement mode
send_packet(0x10)

print("Streaming Accelerometer (m/s²) and Gyroscope (rad/s)...")

while True:
    if ser.read(1) != b'\xFA':
        continue
    if ser.read(1) != b'\xFF':
        continue

    mid = ser.read(1)[0]
    length = ser.read(1)[0]
    payload = ser.read(length)
    ser.read(1)  # checksum

    # MTData2
    if mid != 0x36:
        continue

    # Initialize variables
    acc_x = acc_y = acc_z = None
    gyr_x = gyr_y = gyr_z = None

    i = 0
    while i < len(payload):
        did = payload[i] << 8 | payload[i+1]
        size = payload[i+2]
        data = payload[i+3:i+3+size]

        # Acceleration (m/s²)
        if did == 0x4020:  # Acceleration Float32
            acc_x, acc_y, acc_z = struct.unpack(">fff", data)
        
        # Rate of Turn / Gyroscope (rad/s)
        elif did == 0x8020:  # RateOfTurn Float32
            gyr_x, gyr_y, gyr_z = struct.unpack(">fff", data)

        i += 3 + size

    # Print all available data
    output_parts = []
    store_imu = dict() # store imu data in a dictionary
    
    if acc_x is not None:
        output_parts.append(f"ACC: X:{acc_x:6.3f} Y:{acc_y:6.3f} Z:{acc_z:6.3f} m/s²")
        store_imu['accel_x'] = acc_x
        store_imu['accel_y'] = acc_y
        store_imu['accel_z'] = acc_z
    
    if gyr_x is not None:
        output_parts.append(f"GYR: X:{gyr_x:6.3f} Y:{gyr_y:6.3f} Z:{gyr_z:6.3f} rad/s")
        store_imu['gyro_x'] = gyr_x
        store_imu['gyro_y'] = gyr_y
        store_imu['gyro_z'] = gyr_z
    
    if output_parts:
        print(" | ".join(output_parts))
