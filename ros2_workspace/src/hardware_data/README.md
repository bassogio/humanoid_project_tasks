# hardware_data

## Overview

This is a simple ROS 2 package that demonstrates a Talker/Listener setup.

The **publisher node** reads your laptop's CPU load (using `psutil`) at a sampling rate (R1) and publishes it to the `/myHWtopic` at a publish rate (R2).  
The **subscriber node** listens on `/myHWtopic` and prints every message it receives to the console.

---

## Package Contents

- **hardware_data_pub**: Publisher node
- **hardware_data_sub**: Subscriber node

---

## Parameters

### Publisher node
| Name               | Default    | Description                                  |
| ------------------ | ---------- | -------------------------------------------- |
| `publisher_topic`  | myHWtopic  | Topic to publish CPU load to                 |
| `sampling_rate`    | 1.0        | Rate in Hz to read CPU load (R1)             |
| `publish_rate`     | 0.5        | Rate in Hz to publish data on ROS topic (R2) |

### Subscriber node
| Name                | Default    | Description                               |
| ------------------- | ---------- | ----------------------------------------- |
| `subscriber_topic`  | myHWtopic  | Topic to subscribe and print data         |

---

## How to Build

Navigate to your workspace root:

```bash
colcon build
source install/setup.bash
