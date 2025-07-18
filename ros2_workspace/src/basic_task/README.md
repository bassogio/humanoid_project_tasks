# basic_task

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
```

---

## How to Run

Open **two separate terminals**:

###  Terminal 1 - Run the Publisher Node

This node samples your CPU load at `sampling_rate` (R1) and publishes to `/myHWtopic` at `publish_rate` (R2):

```bash
ros2 run basic_task hardware_data_pub
```

You can also override parameters using:

```bash
ros2 run basic_task hardware_data_pub --ros-args -p sampling_rate:=2.0 -p publish_rate:=1.0
```

Example output:

```bash
[INFO] [hardware_data_pub_node]: Published: 'CPU Load: 18.7%'
```

###  Terminal 2 - Run the Subscriber Node
**NOTE: In the second terminal you should also run**

```bash
source install/setup.bash
```

This node listens on /myHWtopic and prints received messages to stdout:

```bash
ros2 run basic_task hardware_data_sub
```

Example output:

```bash
[INFO] [hardware_data_sub_node]: Received: 'CPU Load: 18.7%'
```



https://github.com/user-attachments/assets/1fecd4c7-c3e5-4ed3-9d97-0d31abdede66

