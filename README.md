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

⚠️ **This section contains plaintext credentials.
Do NOT share publicly.**

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

No hardware safety bypass was performed.

---

## Current Status Summary

| Subsystem          | Status       |
| ------------------ | ------------ |
| Wi-Fi / SSH        | ✅ Working    |
| Maintenance Web UI | ✅ Working    |
| CAN Bus            | ✅ Active     |
| Motion Commands    | ✅ Sent       |
| STO Outputs        | ❌ Disabled   |
| eth1 Link          | ❌ NO-CARRIER |
| Motor Torque       | ❌ Inhibited  |

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

- **COB-ID:** Typically `0x401` (after enabling)
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

## Important Notes

- Many drives accept SDO writes for setpoints but only act on PDO commands
- In synchronous configurations, no SYNC = no motion
- Always verify PDO mapping—it's the source of truth for what the drive expects

---

## License

This guide is provided as-is for educational and practical purposes.

## Contributing

Contributions, corrections, and improvements are welcome! Please submit issues or pull requests.