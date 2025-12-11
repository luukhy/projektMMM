# Robot Operation Manual

This document provides a step-by-step guide for initializing, connecting to, and operating the robot using ROS 2 and MAVROS.

## 1. Startup Procedure

### Hardware & Power
* **Power Switches:** Ensure all buttons/switches on the robot are turned **ON**.
* **Battery Check:** * Ensure the battery voltage is adequate.
    * **Minimum Voltage:** 10.5 V.
    * **Replacement Threshold:** If voltage is around ~11 V, the battery should be replaced.
    * *Note:* When replacing the battery, it is recommended to shut down the NUC computer via Remote Desktop first.

### Remote Connection
1. Ensure the robot's onboard computer (NUC) is powered on.
2. Verify that the dummy plug (HDMI/Display emulator) or cable is inserted into the port.
3. Connect your computer to the same network as the robot:
    * **SSID:** `seminarKIMIA`
    * **Password:** `KIMIA2017`
4. Connect to the robot using **NoMachine**.
5. Once connected, open a terminal to proceed.

---

## 2. ROS 2 Configuration & Arming

### Checking System State
Check the current status of the robot (connection, arming status, and mode) by echoing the MAVROS state topic:

```bash
ros2 topic echo /mavros/state
```

**Expected output:**
```yaml
connected: True 
armed: False 
mode: "MANUAL"
```

### Arming the Robot
The robot must be armed before operation. This can be done via a physical RC remote (if available) or via the computer.

**1. Launch MAVROS/PX4 Interface**
To establish communication with the flight controller:
```bash
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0 baudrate:=921600
```

**2. Set Control Mode**
Ensure the robot is in `MANUAL` (or `ACRO`) mode. If not set, use the following service call:
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'MANUAL'}"
```

**3. Arming Command**
*Note: This service call attempts to arm the robot directly.*
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

> [!WARNING]
> **Communication Failsafe (HOLD Mode)**
> If communication with the controller (RC or MAVROS override) is lost for a defined period, the robot will automatically switch to **HOLD** mode.
>
> When controlling via the `/mavros/rc/override` topic, stopping the publisher (e.g., via `Ctrl+C`) mimics a signal loss, triggering this failsafe.

---

## 3. Movement Control

The robot is controlled by overriding RC channels 1 and 2. 
* **Signal Range:** 1000 to 2000.
* **Neutral (Stop):** 1500.
* **Frequency:** Messages to `~rc/override` must be sent at a frequency **> 1 Hz** to overwrite standard RC inputs.

### Control Mapping
* **Channel 1 (X):** Forward / Backward
    * `X > 1500`: Forward
    * `X < 1500`: Backward
* **Channel 2 (Y):** Turn Left / Right
    * `Y < 1500`: Turn Left
    * `Y > 1500`: Turn Right

### Commands

**Move Forward:**
This command sets Channel 1 to move forward and Channel 2 to neutral. Replace `X` and `Y` with desired PWM values.

```bash
ros2 topic pub /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [X, Y, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

**Stop Robot:**
Resets channels to neutral (1500).

```bash
ros2 topic pub /mavros/rc/override mavros_msgs/msg/OverrideRCIn "{channels: [1500, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

---

*More instructions coming soon.*
