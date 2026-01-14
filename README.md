# BG Flexbot – Networking, CAN, and STO Debugging Notes

## Overview

This document summarizes the investigation into why a **Berkshire Grey Flexbot** robot accepts motion commands via its maintenance web interface and sends CAN traffic, but **does not physically move**.

The root cause was identified as a **safety system (STO) gating issue**, ultimately traced to a **down industrial Ethernet interface (`eth1`)** that prevents the Siemens safety I/O board from arming.

---

## System Architecture (as observed)

### 1. Control Stack Layers

The robot has a layered control architecture:

1. **Application Layer**

   * `fb-maintenance` web interface
   * Accessible at:

     ```
     http://<robot_ip>:61003/
     ```
   * Allows issuing motion commands

2. **Middleware / Controller Layer**

   * Robot control process translates UI commands into CAN messages

3. **Fieldbus Layer**

   * CAN bus (`can1`) connects to AMC motor driver boards
   * Ethernet (`eth1`) connects to Siemens safety I/O board

4. **Safety Layer**

   * Siemens safety / STO I/O board
   * Controls 24 V STO signals to AMC drives
   * Gated by safety logic and industrial Ethernet state

---

## Network Status

### Active Interfaces

```text
wlp1s0   → Wi-Fi (connected, SSH + web UI working)
eth0    → Internal robot network (bridged via br0)
eth1    → Industrial Ethernet to Siemens safety board (NO-CARRIER)
```

Key observation:

```text
eth1: <NO-CARRIER, UP>  → Link detected: no
```

This means:

* The Ethernet PHY is not active
* No industrial fieldbus communication is occurring
* Siemens safety system remains in SAFE state

---

## Wi-Fi Configuration (Sensitive)


The robot uses `wpa_supplicant` with interface-specific configuration:

```
/etc/wpa_supplicant/wpa_supplicant-wlp1s0.conf
```

Multiple Wi-Fi networks are defined with priority-based fallback.

### Configured Networks

#### 1. Enterprise Wi-Fi (Highest Priority)

```ini
network={
    ssid="bg-flexbot-net"
    key_mgmt=WPA-EAP
    eap=TLS
    identity="setupident@host.com"
    ca_cert="/etc/certs/ca.pem"
    client_cert="/etc/certs/setup.pem"
    private_key="/etc/certs/setup.key"
    private_key_passwd="ucoc0siu2eiPhi8ahwo9"
    priority=3
}
```

* Certificate-based authentication (no PSK)
* Intended for factory / corporate environment
* Requires valid certs and backend infrastructure

---

#### 2. WPA-PSK Hotspot / Lab Network

```ini
network={
    ssid="bg-flexbot-wifi6"
    key_mgmt=WPA-PSK
    psk="rac@bg1922"
    priority=2
}
```

* Used successfully with a user-created hotspot
* Robot connected automatically
* Assigned IP example:

  ```
  192.168.50.160/24
  ```

---

#### 3. WPA-PSK Fallback Network

```ini
network={
    ssid="bg-flexbot-psk"
    key_mgmt=WPA-PSK
    psk="N0w1f14youTod@y"
    priority=1
}
```

* Lowest priority
* Likely backup or emergency access network

---

### Wi-Fi Notes

* Interface name is **`wlp1s0`**, not `wlan0`
* Connection verified via:

  ```bash
  iw dev wlp1s0 link
  ip a show wlp1s0
  ```
* SSH and maintenance web UI confirmed working over Wi-Fi

---

## CAN Bus Observations

### CAN Traffic Present

* `candump can1` shows:

  * Periodic command frames
  * Heartbeat-like frames (e.g. `0x701`)
  * Feedback-like frames (small signed values)

This confirms:

* Motion commands **are being generated**
* CAN communication **is functional**
* Lack of motion is **not** a CAN issue

---

## STO (Safe Torque Off) Observations

### AMC Drive Behavior

* STO indicator LEDs on AMC motor drives are **OFF**
* This indicates **no 24 V present on STO inputs**
* Drives therefore **refuse to generate torque**

### Wiring Insight

* STO inputs on all AMC drives are wired to a **central Siemens safety I/O board**
* The Siemens board:

  * Is powered by the robot battery
  * Is connected to the robot computer via **Ethernet**
  * Controls STO centrally for all drives

---

## Siemens Safety I/O Board Role

From behavior and wiring, the Siemens board is acting as:

* A **safety-rated STO controller**
* Likely a PROFINET / PROFIsafe or similar module
* Outputs dual-channel 24 V STO **only when safety logic is healthy**

Key principle:

> **If the safety Ethernet link is down, STO outputs are intentionally disabled.**

---

## Root Cause Identified

### Primary Blocker

**`eth1` has NO-CARRIER (no physical / logical link)**

This causes the following chain:

1. `eth1` is down
2. Siemens safety board does not see its fieldbus master
3. Safety logic remains in SAFE / STOP state
4. STO outputs remain at 0 V
5. AMC drives do not enable torque
6. Robot does not move

Everything else (CAN, UI, software) is downstream of this.

---

## Blue Buttons (Safety Acknowledge)

The robot has **two blue buttons**, one on each side.

Based on standard Siemens / warehouse robot design, these are likely:

* Dual-hand **safety acknowledge / reset** buttons
* Must be pressed **simultaneously**
* Often required to:

  * Exit safe stop
  * Arm STO
  * Enable fieldbus communication

This is a **designed human acknowledgment step**, not a software command.

---

## Safety Note (Critical)

* STO and Siemens safety circuits are **certified safety systems**
* Bypassing safety buttons or shorting wires can:

  * Latch permanent safety faults
  * Cause unexpected motion
  * Damage hardware or injure people

## STO Bypass Configuration (All Four Driver Boards)

> **WARNING**  
> The STO (Safe Torque Off) inputs on the AMC motor driver boards are part of a
> **certified functional safety system**.  
> The procedure described below **intentionally disables all safety protections**
> and must only be used for controlled commissioning and debugging.

---

### Overview

To enable motor operation without the Siemens safety circuit, the STO system was
**intentionally bypassed on all four AMC driver boards**.

---

### Procedure (Repeated Identically on All Four Drives)

For **each AMC driver board (Driver 1–4)**:

1. **Locate the STO terminals** on the AMC drive:
   - `STO1+`
   - `STO2+`
   - `STO COM` / `STO GND`

2. **Apply a direct +24 V DC supply** to:
   - `STO1+`
   - `STO2+`

3. **Verify the following conditions**:
   - Both STO channels (`STO1` and `STO2`) receive **continuous 24 V**
   - The **same 24 V supply** is used for all four driver boards
   - The 24 V supply ground is correctly referenced to the drive logic ground,
     per AMC documentation

4. This configuration was applied **identically to all four AMC driver boards**,
   resulting in:
   - STO asserted (logic-high) on every drive
   - All drives permitted to generate torque when enabled via CAN

---

### Resulting System State

With this configuration:

- **STO functionality is fully disabled**
- Siemens safety relays, E-stop chains, and interlocks are **inactive**
- Motor torque can be enabled **purely via CAN commands**
- Any software, wiring, or control fault can cause **immediate motor motion**

---

### Safety Implications

Bypassing STO can:

- Defeat Safe Torque Off protection
- Cause unexpected or uncontrolled motion
- Latch irreversible drive faults
- Damage hardware
- Can also cause Injury


---

## Current Status Summary

| Subsystem          | Status       |
| ------------------ | ------------ |
| Wi-Fi / SSH        | ✅ Working    |
| Maintenance Web UI | ✅ Working    |
| CAN Bus            | ✅ Active     |
| Motion Commands    | ✅ Sent       |
| STO Outputs        | ✅ Disabled   |
| eth1 Link          | ❌ NO-CARRIER |
| Motor Torque       | ✅ Inhibited  |

---

## Key Takeaway

> **The robot is healthy, but intentionally safe.**
> Motion is blocked because the Siemens safety system has not armed, and that arming is gated by the `eth1` industrial Ethernet link.

Once `eth1` comes up, existing motion commands should immediately produce movement.

---


# AMC CANopen Control (CiA-402) — Practical Guide

A comprehensive guide for controlling **Advanced Motion Controls (AMC)** drives via **CANopen** using raw CAN frames with Linux `can-utils` (`cansend`, `candump`).

This guide demonstrates how to inspect RPDOs, determine their status, understand their mappings, and control motor drives using the CiA-402 protocol.

**Tested Configuration:** Drive with NodeID = 2

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [CANopen Quick Model](#canopen-quick-model)
- [Key CAN IDs](#key-can-ids)
- [CiA-402 Objects](#cia-402-objects)
- [NMT Commands](#nmt-commands)
- [SDO Operations](#sdo-operations)
- [CiA-402 Enable Sequence](#cia-402-enable-sequence)
- [RPDO Inspection](#rpdo-inspection)
- [RPDO Activation](#rpdo-activation)
- [Discovered PDO Layout](#discovered-pdo-layout)
- [Velocity Control](#velocity-control)
- [Position Control](#position-control)
- [Diagnostics](#diagnostics)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

### 1. Setup CAN Interface

```bash
# Bring up CAN interface at 1 Mbps
sudo ip link set can0 up type can bitrate 1000000
```

### 2. Install Tools

```bash
sudo apt-get install can-utils
```

### 3. Monitor Traffic

```bash
candump can0
```

---

## CANopen Quick Model

- **NMT**: Controls node state (Pre-Operational / Operational)
- **SDO** (0x600/0x580): Configuration & diagnostics (read/write objects)
- **PDO**: High-rate control and telemetry
  - **RPDO**: Master → Drive (commands)
  - **TPDO**: Drive → Master (telemetry)
- **SYNC** (0x080): Required for synchronous PDO operation

---

## Key CAN IDs

For NodeID = N:

| Function | CAN ID |
|----------|--------|
| NMT | 0x000 |
| SYNC | 0x080 |
| SDO Request (to node) | 0x600 + N |
| SDO Response (from node) | 0x580 + N |
| Heartbeat | 0x700 + N |

**Example for Node 2:**
- SDO Request: `0x602`
- SDO Response: `0x582`
- Heartbeat: `0x702`

---

## CiA-402 Objects

| Object | Description |
|--------|-------------|
| 0x6040 | Controlword |
| 0x6041 | Statusword |
| 0x6060 | Mode of Operation |
| 0x6061 | Mode Display |
| 0x60FF | Target Velocity |
| 0x607A | Target Position |
| 0x606C | Actual Velocity |
| 0x6064 | Actual Position |
| 0x6081 | Profile Velocity |
| 0x6083 | Profile Acceleration |
| 0x6084 | Profile Deceleration |

---

## NMT Commands

```bash
# Enter Pre-Operational for node 2
cansend can0 000#8002

# Start node 2 (Operational)
cansend can0 000#0102
```

---

## SDO Operations

### Read Object

Example: Read Statusword (0x6041:00)

```bash
cansend can0 602#4041600000000000
```

**Typical Response:**
```
582  [8]  42 41 60 00 37 06 00 00
```

### Write Object

Example: Set Mode of Operation (0x6060 = 3)

```bash
cansend can0 602#2F60600003000000
```

---

## CiA-402 Enable Sequence

```bash
# 1. Shutdown
cansend can0 602#2B40600006000000

# 2. Switch On
cansend can0 602#2B40600007000000

# 3. Enable Operation
cansend can0 602#2B4060000F000000
```

**Verify enabled status:**

```bash
cansend can0 602#4041600000000000
```

Expected statusword: `0x0637` (bytes `37 06`)

---

## RPDO Inspection

### Object Dictionary Locations

For RPDO k (1..4):

| What | Object | Subindex |
|------|--------|----------|
| RPDOk COB-ID | 0x1400 + (k-1) | :01 |
| RPDOk Transmission Type | 0x1400 + (k-1) | :02 |
| RPDOk Mapping Count | 0x1600 + (k-1) | :00 |
| RPDOk Mapping Entry i | 0x1600 + (k-1) | :i |

### 1. Check if RPDO is Active

Read RPDO4 COB-ID (0x1403:01):

```bash
cansend can0 602#4003140100000000
```

**Interpret 4 bytes (little-endian):**
- `01 02 00 80` → `0x80000201` → Disabled
- `01 02 00 00` → `0x00000201` → Enabled

**Rule:** If bit 31 is set (`0x80000000`), PDO is disabled.

### 2. Find RPDO Mapping

Read mapping count for RPDO4 (0x1603:00):

```bash
cansend can0 602#4003160000000000
```

Read mapping entries:

```bash
cansend can0 602#4003160100000000
cansend can0 602#4003160200000000
```

**Mapping entry format (32-bit):**
```
(Index 16-bit) | (Subindex 8-bit) | (Size-in-bits 8-bit)
```

**Examples:**
- `0x60400010` → 0x6040:00, 16 bits → Controlword
- `0x60FF0020` → 0x60FF:00, 32 bits → Target Velocity
- `0x607A0020` → 0x607A:00, 32 bits → Target Position

### 3. Check Transmission Type

Read RPDO4 transmission type (0x1403:02):

```bash
cansend can0 602#4003140200000000
```

**Interpretation:**
- `0x01` (or `0x00..0xF0`) → Synchronous (requires SYNC)
- `0xFF` → Asynchronous (immediate)

---

## RPDO Activation

If COB-ID shows `0x8XXXXXXX` (disabled), enable it:

```bash
# 1. Enter Pre-Operational
cansend can0 000#8002

# 2. Write COB-ID with bit 31 cleared (example: 0x80000301 → 0x00000301)
cansend can0 602#2301140101030000

# 3. Verify
cansend can0 602#4001140100000000

# 4. Start node
cansend can0 000#0102
```

---

## Discovered PDO Layout

### RPDO2 (COB-ID 0x301)

**Mapping:**
- 0x6040 (Controlword, 16 bits)
- 0x6060 (Mode of Operation, 8 bits)

**Payload:** `[CW_L][CW_H][MODE]`

**Examples:**

```bash
# Set Profile Velocity (mode=3) and enable
cansend can0 301#0F0003

# Set Profile Position (mode=1)
cansend can0 301#0F0001
```

### RPDO4 (COB-ID 0x201)

**Mapping:**
- 0x6040 (Controlword, 16 bits)
- 0x60FF (Target Velocity, 32 bits)

**Transmission Type:** 0x01 (SYNC-driven)

**Payload:** `[CW_L][CW_H][VEL0][VEL1][VEL2][VEL3][00][00]`

---

## Velocity Control

### SYNC Generation

**Terminal A (50 Hz SYNC loop):**

```bash
I recommend logging the candump by running the run_motor.sh in PCAN-View and find the appropriate cycle time
```


```bash
while true; do
  cansend can0 080#
  sleep 0.02
done
```

**Terminal B (Commands):**

```bash
# Ensure node is operational and enabled first
cansend can0 301#0F0003                 # Set Profile Velocity mode
cansend can0 201#0F00881300000000       # CW=0x000F, vel=+5000
```

**Stop:**

```bash
cansend can0 201#0F00000000000000
```

**Read actual velocity:**

```bash
cansend can0 602#406C600000000000
```

### Optional: Make RPDO4 Asynchronous

```bash
cansend can0 000#8002
cansend can0 602#2F031402FF000000   # Set 0x1403:02 = 0xFF
cansend can0 000#0102
```

---

## Position Control

### RPDO3 Configuration

- **COB-ID:** Typically `0x401 or 0x501` (after enabling) 
- **Mapping:**
  - 0x6040 (Controlword, 16 bits)
  - 0x607A (Target Position, 32 bits)

### Set Profile Parameters

```bash
# Profile Velocity = 100000
cansend can0 602#23816000A0860100

# Profile Acceleration = 200000
cansend can0 602#23836000400D0300

# Profile Deceleration = 200000
cansend can0 602#23846000400D0300
```

### Relative Move Example

**Move +10000 counts:**

```bash
# Trigger (0x007F: enabled + immediate + relative + new-setpoint)
cansend can0 401#7F00102700000000

# Clear toggle (0x006F)
cansend can0 401#6F00102700000000
```

**Read actual position:**

```bash
cansend can0 602#4064600000000000
```

---

## Diagnostics

```bash
# Statusword
cansend can0 602#4041600000000000

# Mode Display
cansend can0 602#4061600000000000

# Actual Velocity
cansend can0 602#406C600000000000

# Actual Position
cansend can0 602#4064600000000000
```

---

## Troubleshooting

### Checklist

- [ ] **Node alive?** Look for heartbeat at `0x700+N` (e.g., `0x702`)
- [ ] **NMT state correct?** Run `cansend can0 000#0102`
- [ ] **CiA-402 enabled?** Statusword should show `0x0637`
- [ ] **RPDOs enabled?** Check bit 31 of COB-ID (`0x140x:01`)
- [ ] **Correct COB-ID?** Verify drive configuration
- [ ] **RPDO SYNC-driven?** If transmission type is `0x01`, send SYNC or change to `0xFF`
- [ ] **PDO mapping matches payload?** Verify mapping entries match your data structure

### Quick RPDO4 Inspection (Node 2)

```bash
# COB-ID
cansend can0 602#4003140100000000

# Transmission Type
cansend can0 602#4003140200000000

# Mapping Count
cansend can0 602#4003160000000000

# Mapping Entries
cansend can0 602#4003160100000000
cansend can0 602#4003160200000000
```

---


# CAN Motor Control for Differential Drive Robot

Complete guide to control CANopen motors on a differential drive robot using Python and SocketCAN.

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Initial Setup](#initial-setup)
- [Understanding Your Motors](#understanding-your-motors)
- [Getting Started](#getting-started)
- [Scripts Overview](#scripts-overview)
- [Troubleshooting](#troubleshooting)
- [Advanced Usage](#advanced-usage)

---

## Hardware Requirements

- Differential drive robot with CANopen motor controllers
- CAN-to-USB adapter (or built-in CAN interface)
- Linux computer (tested on Ubuntu)
- 2x Motors with CANopen support (tested with nodes 1 and 2)

## Software Requirements

- Python 3.7+
- python-can library
- SocketCAN (included in Linux kernel)

### Installation
```bash
# Install python-can
pip install python-can

# Verify SocketCAN is available
sudo modprobe can
sudo modprobe can_raw
```

---

## Initial Setup

### 1. Configure CAN Interface

First, set up your CAN interface. Replace `can0` with your interface name if different:
```bash
# Bring up CAN interface at 1Mbit/s (adjust bitrate if needed)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify interface is up
ip link show can0
```

**Make it persistent** (optional):

Create `/etc/systemd/network/80-can.network`:
```ini
[Match]
Name=can0

[CAN]
BitRate=1000000
```

### 2. Verify CAN Communication

Before running any scripts, verify you can see CAN traffic:
```bash
# Install can-utils if not present
sudo apt-get install can-utils

# Monitor CAN bus (Ctrl+C to stop)
candump can0
```

You should see CAN messages if your motors are powered on.

---

## Understanding Your Motors

### Identifying Motor Node IDs

Motors communicate using CANopen node IDs. Common patterns:
- **Node 1** (Left motor): Uses IDs `0x601` (SDO request), `0x581` (SDO response), `0x501` (RPDO)
- **Node 2** (Right motor): Uses IDs `0x602`, `0x582`, `0x502`

### Discovery Script

Use this script to find which nodes are responding:
```python
#!/usr/bin/env python3
"""ping.py - Discover active CANopen nodes"""
import can
import time

bus = can.Bus(channel='can0', interface='socketcan')

print("Scanning for CANopen nodes...")

for node_id in range(1, 5):
    print(f"\nTrying Node {node_id}...")
    
    # Send SDO read request for statusword (0x6041)
    req_id = 0x600 + node_id
    resp_id = 0x580 + node_id
    
    msg = can.Message(
        arbitration_id=req_id,
        data=[0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0],
        is_extended_id=False
    )
    bus.send(msg)
    time.sleep(0.1)
    
    # Check for response
    response = bus.recv(timeout=0.5)
    if response and response.arbitration_id == resp_id:
        print(f"  ✓ Node {node_id} responded!")
        print(f"    Response: {response}")
    else:
        print(f"  ✗ No response from Node {node_id}")

bus.shutdown()
```

Run it:
```bash
python3 ping.py
```

---

## Getting Started

### Step 1: Capture Motor Configuration

If your robot came with proprietary control software, capture the CAN traffic to understand the initialization sequence:
```bash
# Start logging (in one terminal)
candump can0 -l

# Run the proprietary software to start motors (in another terminal/computer)

# Stop logging with Ctrl+C
# This creates candump-YYYY-MM-DD_HHMMSS.log
```

### Step 2: Initialize Motors

Create separate initialization scripts for each motor. Here's the template:

**`init_left_motor.py`** (Node 1):
```python
#!/usr/bin/env python3
"""
Left Motor (Node 1) - Complete Initialization
"""
import can
import struct
import time

bus = can.Bus(channel='can0', interface='socketcan')
NODE = 1

def wait_for_sdo_response(timeout=0.5):
    """Wait for SDO response"""
    start = time.time()
    while time.time() - start < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == (0x580 + NODE):
            return True
    return False

def send_sdo(data):
    """Send SDO and wait for response"""
    bus.send(can.Message(
        arbitration_id=0x600 + NODE, 
        data=data, 
        is_extended_id=False
    ))
    return wait_for_sdo_response()

def send_cmd(ctrl, vel=0):
    """Send RPDO command + SYNC"""
    msg = can.Message(
        arbitration_id=0x500 + NODE,
        data=struct.pack('<HiH', ctrl, vel, 0),
        is_extended_id=False
    )
    bus.send(msg)
    time.sleep(0.002)
    
    # Send SYNC
    sync = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
    bus.send(sync)
    time.sleep(0.008)

def send_nmt(data):
    """Send NMT command"""
    bus.send(can.Message(
        arbitration_id=0x000, 
        data=data, 
        is_extended_id=False
    ))
    time.sleep(0.1)

print("="*70)
print(f"MOTOR NODE {NODE} - INITIALIZATION")
print("="*70)

# Reset node
print(f"\n[1/11] Resetting Node {NODE}...")
send_nmt(bytes([0x80, NODE]))
time.sleep(0.5)

# Configure PDOs, motor parameters, etc.
# (Add all SDO configuration commands from your captured log)
print("[2/11] Configuring manufacturer parameters...")
send_sdo(bytes([0x42, 0x02, 0x20, 0x05, 0x00, 0x00, 0x00, 0x00]))
# ... (add all your SDO commands here)

# CRITICAL: State machine activation via RPDO
print("[11/11] State machine via RPDO (MOTOR ENGAGE)...")
send_cmd(0x0080, 0)  # Fault reset
for _ in range(5):
    send_cmd(0x0006, 0)  # Shutdown
for _ in range(5):
    send_cmd(0x0007, 0)  # Switch on
for _ in range(10):
    send_cmd(0x000F, 0)  # Enable operation

print(f"\n✓ MOTOR NODE {NODE} INITIALIZED AND ENGAGED!")
print("  You should have heard a click sound")

bus.shutdown()
```

**For Node 2 (Right motor)**: Copy the script and change `NODE = 1` to `NODE = 2`.

Run initialization:
```bash
python3 init_left_motor.py
python3 init_right_motor.py
```

You should hear a click/engagement sound from each motor.

### Step 3: Test Motor Control

**`speed_test.py`** - Test single motor:
```python
#!/usr/bin/env python3
"""
Single Motor Speed Test
"""
import can
import struct
import time

bus = can.Bus(channel='can0', interface='socketcan')

# CONFIGURATION
NODE = 1  # Change to 2 for right motor
CMD_PER_WHEEL_RPM = 3814.0  # Calibration constant
TARGET_RPM = 50  # Test speed

def send_cmd(ctrl, vel=0):
    """Send RPDO command + SYNC"""
    msg = can.Message(
        arbitration_id=0x500 + NODE,
        data=struct.pack('<HiH', ctrl, vel, 0),
        is_extended_id=False
    )
    bus.send(msg)
    time.sleep(0.002)
    
    sync = can.Message(arbitration_id=0x080, data=[], is_extended_id=False)
    bus.send(sync)
    time.sleep(0.008)

def send_sdo_read(node_id, index, subindex):
    """Read via SDO"""
    cob_id = 0x600 + node_id
    payload = [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
    msg = can.Message(arbitration_id=cob_id, data=payload, is_extended_id=False)
    bus.send(msg)
    time.sleep(0.05)
    
    msg = bus.recv(timeout=0.5)
    if msg and len(msg.data) >= 8:
        if msg.data[0] == 0x42:  # 2 bytes
            return struct.unpack('<H', bytes(msg.data[4:6]))[0]
        elif msg.data[0] == 0x43 or msg.data[0] == 0x47:  # 4 bytes
            return struct.unpack('<i', bytes(msg.data[4:8]))[0]
    return None

print(f"Testing Motor Node {NODE}")
print(f"Target: {TARGET_RPM} RPM\n")

# Check status
status = send_sdo_read(NODE, 0x6041, 0x00)
if not status or not (status & 0x0004):
    print("⚠️  Motor not enabled! Run initialization script first.")
    bus.shutdown()
    exit()

print("✓ Motor operational!\n")

target_cmd = int(round(TARGET_RPM * CMD_PER_WHEEL_RPM))

# Ramp up
print("Ramping up...")
steps = 20
for i in range(steps + 1):
    vel = int(target_cmd * i / steps)
    send_cmd(0x000F, vel)
    time.sleep(0.1)

# Hold
print(f"Holding at {TARGET_RPM} RPM for 5 seconds...")
start = time.time()
while time.time() - start < 5.0:
    send_cmd(0x000F, target_cmd)

# Ramp down
print("Ramping down...")
for i in range(steps, -1, -1):
    vel = int(target_cmd * i / steps)
    send_cmd(0x000F, vel)
    time.sleep(0.08)

# Stop
print("Stopping...")
for _ in range(30):
    send_cmd(0x000F, 0)

# Disable
for _ in range(10):
    send_cmd(0x0006, 0)

bus.shutdown()
print("\n✓ Test complete!")
```

Run test:
```bash
python3 speed_test.py
```

---

## Scripts Overview

### Dual Motor Control

**`robot_control.py`** - Control both motors for differential drive:
```python
#!/usr/bin/env python3
"""
Differential Drive Robot Control
"""
import can
import struct
import time

bus = can.Bus(channel='can0', interface='socketcan')
CMD_PER_WHEEL_RPM = 3814.0

def send_cmd_motor(node, ctrl, vel=0):
    """Send RPDO command to specific motor"""
    rpdo_id = 0x500 + node
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
    
    send_cmd_motor(1, 0x000F, left_cmd)
    send_cmd_motor(2, 0x000F, right_cmd)
    send_sync()

# Movement primitives
def forward(speed_rpm, duration):
    """Drive forward"""
    start = time.time()
    while time.time() - start < duration:
        set_velocities(speed_rpm, speed_rpm)

def backward(speed_rpm, duration):
    """Drive backward"""
    forward(-speed_rpm, duration)

def rotate_left(speed_rpm, duration):
    """Rotate counter-clockwise"""
    start = time.time()
    while time.time() - start < duration:
        set_velocities(-speed_rpm, speed_rpm)

def rotate_right(speed_rpm, duration):
    """Rotate clockwise"""
    start = time.time()
    while time.time() - start < duration:
        set_velocities(speed_rpm, -speed_rpm)

def stop():
    """Stop both motors"""
    for _ in range(30):
        set_velocities(0, 0)

# Example usage
if __name__ == "__main__":
    print("Robot Control Test\n")
    
    print("→ Forward")
    forward(50, 2)
    stop()
    time.sleep(0.5)
    
    print("→ Backward")
    backward(50, 2)
    stop()
    time.sleep(0.5)
    
    print("→ Rotate Right")
    rotate_right(50, 2)
    stop()
    time.sleep(0.5)
    
    print("→ Rotate Left")
    rotate_left(50, 2)
    stop()
    
    # Disable motors
    for _ in range(10):
        send_cmd_motor(1, 0x0006, 0)
        send_cmd_motor(2, 0x0006, 0)
        send_sync()
    
    bus.shutdown()
    print("\n✓ Complete!")
```

---

## Troubleshooting

### Motor doesn't engage (no click sound)

**Symptom**: Initialization runs without errors but no click sound.

**Solution**: Make sure you're sending the RPDO state machine commands at the end of initialization:
```python
send_cmd(0x0080, 0)  # Fault reset
for _ in range(5):
    send_cmd(0x0006, 0)  # Shutdown
for _ in range(5):
    send_cmd(0x0007, 0)  # Switch on
for _ in range(10):
    send_cmd(0x000F, 0)  # Enable operation
```

### Motor reports "Operation Enabled" but doesn't move

**Symptom**: Status shows `0x0637` but velocity stays at 0.

**Cause**: Sending velocity via SDO instead of RPDO.

**Solution**: Use RPDO commands with SYNC:
```python
send_cmd(0x000F, velocity_command)  # Not send_sdo!
```

### "No SDO response" warnings

**Symptom**: Warnings during initialization but motor still works.

**Cause**: Timing issues or non-critical SDO reads.

**Solution**: Usually safe to ignore if motor engages. If problematic, increase timeout:
```python
def wait_for_sdo_response(timeout=1.0):  # Increase from 0.5 to 1.0
```

### CAN interface not found
```bash
# Check interface name
ip link show

# Bring up interface
sudo ip link set can0 up

# Check for errors
dmesg | grep can
```

### Wrong bitrate

**Symptom**: No CAN traffic or garbled messages.

**Solution**: Common CAN bitrates are 125k, 250k, 500k, 1M. Try:
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

### Statusword stuck at 0x0228 (Quick Stop)

**Symptom**: Motor in Quick Stop state, not ready.

**Solution**: Send fault reset before enabling:
```python
send_sdo(bytes([0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00]))  # Fault reset
time.sleep(0.5)
# Then proceed with normal state machine
```

---

## Advanced Usage

### Velocity Feedback

Read actual motor velocity (0x606C):
```python
def read_actual_velocity(node):
    """Read actual velocity from motor"""
    # Send SDO read request for 0x606C
    req = can.Message(
        arbitration_id=0x600 + node,
        data=[0x40, 0x6C, 0x60, 0x00, 0, 0, 0, 0],
        is_extended_id=False
    )
    bus.send(req)
    time.sleep(0.05)
    
    # Parse response
    resp = bus.recv(timeout=0.5)
    if resp and len(resp.data) >= 8:
        actual_cmd = struct.unpack('<i', bytes(resp.data[4:8]))[0]
        actual_rpm = actual_cmd / CMD_PER_WHEEL_RPM
        return actual_rpm
    return None
```

### Closed-Loop Control
```python
class MotorController:
    def __init__(self, node, kp=0.1):
        self.node = node
        self.kp = kp  # Proportional gain
    
    def set_velocity_closed_loop(self, target_rpm, duration):
        """Maintain target velocity with feedback"""
        start = time.time()
        
        while time.time() - start < duration:
            actual_rpm = read_actual_velocity(self.node)
            if actual_rpm is not None:
                error = target_rpm - actual_rpm
                correction = self.kp * error
                adjusted_rpm = target_rpm + correction
                
                cmd = int(round(adjusted_rpm * CMD_PER_WHEEL_RPM))
                send_cmd_motor(self.node, 0x000F, cmd)
            
            time.sleep(0.05)
```

### Emergency Stop
```python
def emergency_stop():
    """Immediate stop for both motors"""
    for _ in range(50):
        send_cmd_motor(1, 0x000F, 0)
        send_cmd_motor(2, 0x000F, 0)
        send_sync()
        time.sleep(0.001)
```

### Calibration

Find your `CMD_PER_WHEEL_RPM` constant:
```python
# 1. Command a known value (e.g., 100000)
# 2. Measure actual wheel RPM with tachometer or encoder
# 3. Calculate: CMD_PER_WHEEL_RPM = command_value / measured_rpm
```

---

## Understanding CANopen Communication

### SDO (Service Data Object)
- Used for configuration and setup
- Request/response protocol
- Node 1: `0x601` (request) → `0x581` (response)

### RPDO (Receive Process Data Object)
- Used for real-time control commands
- No response required
- Node 1: `0x501` (commands)

### SYNC
- Triggers PDO processing
- Broadcast message: `0x080`
- Sent after RPDO commands

### NMT (Network Management)
- Controls node states (reset, start, stop)
- Broadcast or addressed
- `0x000`

---

## Project Structure
```
robot-control/
├── README.md
├── ping.py                 # Discover active nodes
├── init_left_motor.py      # Initialize left motor (Node 1)
├── init_right_motor.py     # Initialize right motor (Node 2)
├── speed_test.py           # Single motor test
├── robot_control.py        # Dual motor control
└── can_traffic_monitor.py  # Log CAN traffic
```

---

## Contributing

If you're adapting this for different motors:

1. Capture your motor's CAN traffic with `candump`
2. Identify the SDO configuration sequence
3. Adapt the initialization scripts
4. Test with a single motor first
5. Submit a PR with your motor configuration!

---

## Acknowledgments

- Built using [python-can](https://python-can.readthedocs.io/)
- CANopen protocol specification: CiA 301
- Tested on differential drive robots with CANopen motor controllers

---

## Support

Having issues? Check:
1. CAN interface is up: `ip link show can0`
2. Correct bitrate: Try 125k, 250k, 500k, 1M
3. Motor power is on
4. Correct node IDs (use `ping.py`)
5. Initialization script completes successfully
6. You hear the engagement click

Still stuck? Open an issue with:
- Your `candump` log
- Output from initialization script
- Motor controller model

---

## Important Notes

- Many drives accept SDO writes for setpoints but only act on PDO commands
- In synchronous configurations, no SYNC = no motion
- Always verify PDO mapping—it's the source of truth for what the drive expects

---

## License


## Contributing

Contributions, corrections, and improvements are welcome! Please submit issues or pull requests.