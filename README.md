# obstacle_avoider — Autonomous Obstacle Avoidance (ROS 2 Humble)

A ROS 2 package that drives a simulated TurtleBot3 through an environment while
detecting and avoiding obstacles in real time, using LiDAR data.

Built as part of the Egrobots ROS 2 Week Task (Autonomous Obstacle Avoidance).

---

## 1. Overview

The robot continuously reads its LiDAR scan (`/scan`) and checks a forward-facing
cone for obstacles. If something is closer than a configurable safe distance, the
robot stops moving forward and turns in place until the path is clear again.
Otherwise, it drives forward. The behavior can be started and stopped on demand
via ROS 2 services, and all key thresholds are exposed as runtime-tunable
parameters rather than hardcoded values.

---

## 2. Package Contents

| File | Purpose |
|---|---|
| `obstacle_avoider/obstacle_avoider_node.py` | Main node: subscribes to `/scan`, publishes to `/cmd_vel`, implements avoidance logic, parameters, and start/stop services |
| `obstacle_avoider/manual_controller.py` | Optional keyboard teleop node for manually driving the robot (testing/demo aid) |
| `obstacle_avoider/scan_subscriber.py` | Early learning-step node: LiDAR subscriber only (not part of the final behavior) |
| `obstacle_avoider/velocity_publisher.py` | Early learning-step node: constant-forward publisher only (not part of the final behavior) |
| `launch/obstacle_avoidance.launch.py` | Launch file that starts Gazebo + the obstacle avoider node together |

`scan_subscriber` and `velocity_publisher` were built first, separately, to
understand ROS 2 pub/sub before combining both into `obstacle_avoider_node`. They
are kept in the package as a record of that process but aren't required to run
the final application.

---

## 3. Build & Run — From a Clean Workspace

### Prerequisites
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3-teleop   # optional, only if using built-in teleop instead of manual_controller
```

Set the TurtleBot3 model (once, persists via `.bashrc`):
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place this package here as obstacle_avoider/

cd ~/ros2_ws
colcon build --packages-select obstacle_avoider
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Run everything with one command
```bash
ros2 launch obstacle_avoider obstacle_avoidance.launch.py
```

This launches Gazebo (empty world, TurtleBot3 spawned) and starts
`obstacle_avoider_node`. The node starts **disabled** by design (see Section 5),
so avoidance must be explicitly started:

```bash
ros2 service call /start_avoidance std_srvs/srv/Trigger
```

To stop the behavior at any time (robot halts immediately):
```bash
ros2 service call /stop_avoidance std_srvs/srv/Trigger
```

### Overriding parameters at launch
```bash
ros2 launch obstacle_avoider obstacle_avoidance.launch.py safe_distance:=1.0 linear_speed:=0.15
```

### Manual driving (optional, for testing)
```bash
ros2 run obstacle_avoider manual_controller
```
Controls: `w`/`s` forward/backward, `a`/`d` turn left/right, `x` stop, `Ctrl+C` quit.
Do not run this at the same time as `obstacle_avoider_node` while it's enabled —
both publish to `/cmd_vel` and will conflict.

---

## 4. ROS 2 Components

### Node: `obstacle_avoider_node`

**Subscribes to:**
| Topic | Type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR distance readings, used to detect obstacles ahead |

**Publishes to:**
| Topic | Type | Purpose |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands (linear/angular) sent to the robot |

**Provides services:**
| Service | Type | Purpose |
|---|---|---|
| `/start_avoidance` | `std_srvs/srv/Trigger` | Enables the avoidance behavior |
| `/stop_avoidance` | `std_srvs/srv/Trigger` | Disables the behavior and immediately publishes a zero `Twist` so the robot doesn't coast |

**Parameters:**
| Parameter | Default | Meaning |
|---|---|---|
| `safe_distance` | `0.5` (m) | Distance below which an obstacle is considered too close |
| `linear_speed` | `0.2` (m/s) | Forward speed used while the path is clear |
| `angular_speed` | `0.5` (rad/s) | Turning speed used while avoiding an obstacle |
| `cone_angle_deg` | `30.0` (deg) | Half-angle of the forward-facing detection cone |

Parameters can be read/changed live while the node is running:
```bash
ros2 param list /obstacle_avoider_node
ros2 param get /obstacle_avoider_node linear_speed
ros2 param set /obstacle_avoider_node safe_distance 1.0
```

### Node: `manual_controller`
Publishes `geometry_msgs/msg/Twist` to `/cmd_vel` based on keyboard input, for
manual testing/demo purposes.

---

## 5. Design Decisions

**Reactive avoidance over a planned path.** The robot doesn't build a map or plan
a route — it reacts to the closest obstacle in its immediate forward view each
scan cycle. This was chosen because it satisfies the task's core requirement
(real-time detection + avoidance) with a design that's easy to reason about,
test, and explain, and it maps directly onto sensor data without extra
dependencies like SLAM/Nav2.

**Forward-facing cone, not the full 360° scan.** An early version used
`min()` over the entire LiDAR scan, which caused the robot to react to objects
beside or behind it — objects nowhere near its actual path. Restricting the
check to a ±30° cone in front (`cone_angle_deg` parameter) fixed this and made
the behavior match visible driving intent.

**Turn-in-place recovery.** When blocked, the robot sets `linear.x = 0` and
turns via `angular.z` until the cone clears. This is simple, predictable, and
sufficient for a single-obstacle empty-world environment; it does not attempt
to pick the "best" direction to turn (always turns the same way), which is a
known simplification worth extending later (e.g., turning toward the side with
more clearance).

**Start/stop as a service, not a topic.** `/start_avoidance` and
`/stop_avoidance` are one-shot, on/off commands with no ongoing data to stream —
a service (request → response) fits that better than a topic, which is meant for
continuous data. `std_srvs/srv/Trigger` was used specifically because it needs
no input arguments and returns a simple success/message pair, matching the
"just do this" nature of the command.

**Parameters over hardcoded constants.** `safe_distance`, `linear_speed`,
`angular_speed`, and `cone_angle_deg` are all ROS 2 parameters so they can be
tuned without recompiling — either at launch time (`ros2 launch ... param:=value`)
or live at runtime (`ros2 param set`), which was verified during testing.

**Launch file reuses TurtleBot3's own Gazebo launch file** via
`IncludeLaunchDescription` rather than duplicating its setup, keeping the launch
file focused on what this package actually adds (the avoider node and its
parameters).

---

## 6. Known Limitations / Possible Extensions

- Always turns the same direction when avoiding — doesn't check which side has
  more clearance.
- No recovery for being fully boxed in (e.g., a dead end); would spin in place
  indefinitely.
- Tested in an empty Gazebo world with single obstacles; not evaluated in
  cluttered or dynamic (moving-obstacle) environments.
- Manual keyboard control processes one key at a time, so simultaneous combined
  movement (e.g., forward + turn) isn't possible without adding explicit combo
  keys — a limitation of single-key polling, not of the robot's `Twist` command
  itself (which does support simultaneous linear + angular motion).
