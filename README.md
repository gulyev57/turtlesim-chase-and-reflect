# 🐢 ROS 2 Turtle Motion Behaviors – `hw_9`

A ROS 2 package that simulates two distinct motion behaviors using Turtlesim:

- **Follower Turtle**: Implements multiple path-following strategies including PID-style pursuit, Pure Pursuit, and Stanley controller.
- **Leader Turtle**: Moves independently to be followed or tracked.

This project explores pursuit, control theory, and reactive behaviors in a simple ROS 2 simulation using Python.

---

## 📦 Package Structure

```
.
├── hw_9/                        # Main package code
│   ├── robot2_leader.py                # Leader turtle logic
│   ├── robot1_follower.py              # Basic PID-like follower
│   ├── robot1_follower_pure_pursuit.py# Pure Pursuit controller
│   ├── robot1_follower_stanley.py     # Stanley controller
│   ├── config.json                     # Configuration file
│   └── __init__.py
├── launch/
│   └── follower_launch.py             # Launch file to run the simulation
├── test/                               # Linting and formatting tests
├── LICENSE
├── package.xml
├── setup.py
├── setup.cfg
```

---

## 🚀 How to Run

### 1. Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```

### 2. Build the package
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. Launch the simulation
```bash
ros2 launch launch/follower_launch.py
```

> 🔧 You can edit `follower_launch.py` or `config.json` to switch between Pure Pursuit, Stanley, and other control modes.

---

## 📁 Dependencies

Make sure you have the following installed:

- ROS 2 Humble (or later)
- `turtlesim`
- Python 3.8+
- `geometry_msgs`, `turtlesim`, `rclpy`, `launch_ros`, `math`

Install Turtlesim if not already installed:
```bash
sudo apt install ros-humble-turtlesim
```

---

## 🤖 Algorithms Implemented

### 🔵 `robot1_follower.py` – Basic PID-Like Follower
- Tracks the leader turtle using proportional control:
  ```python
  error = distance_to_goal
  angular_z = angular_gain * angle_difference
  linear_x = linear_gain * error
  ```

### 🟢 `robot1_follower_pure_pursuit.py`
- Uses the Pure Pursuit algorithm to adjust steering based on lookahead point and curvature.

### 🟡 `robot1_follower_stanley.py`
- Implements Stanley Controller used in self-driving cars.
- Adjusts heading based on cross-track error and speed.

### 🔴 `robot2_leader.py`
- Moves randomly or along a predefined path.
- Acts as the target for follower turtles.

---

## ⚙️ Configuration

You can tweak parameters like:
- Control mode
- Gain values
- Lookahead distance
- Leader speed

in either the Python files or `config.json`.

---

## 🧪 Tests

Includes basic Python formatting and style tests:
```bash
colcon test
```

---

## 📌 TODO

- Add GUI or CLI toggle to switch control modes live
- Implement live parameter tuning
- Add visualization for path and control points in Rviz

---

## 🙌 Credits

Created as part of a robotics project exploring motion control and behavior simulation in ROS 2 using Turtlesim.

---

## 📜 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
