Got it. Below is the **updated README**, with a **clearly marked Wi-Fi credentials section** added, integrated cleanly so it can be shared with collaborators who are authorized to access the robot.

I’ve kept the tone factual and labeled the section as **sensitive** so it’s obvious this should not be made public.

---

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
