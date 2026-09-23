# ROS 2 UR3/UR3e Cartesian Letter Drawing

ROS 2 package for simulating a **Universal Robots UR3/UR3e** robot and performing Cartesian trajectory-based letter drawing using **MoveIt 2** and **Gazebo**.

## 1. Requirements

The project requires:

* Ubuntu Linux
* ROS 2
* MoveIt 2
* Gazebo
* `colcon`
* `rosdep`

The exact ROS 2 distribution should match the versions of the Universal Robots and MoveIt 2 packages used by the project.

---

## 2. Clone the repository

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository:

```bash
git clone https://github.com/linhnghu/ur_gz.git
```

Return to the workspace:

```bash
cd ~/ros2_ws
```

---

## 3. Install dependencies

Source ROS 2 first:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

Install package dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` is not installed:

```bash
sudo apt update
sudo apt install python3-rosdep
```

Then initialize it if necessary:

```bash
sudo rosdep init
rosdep update
```

---

## 4. Build the workspace

From the workspace root:

```bash
cd ~/workspaces/ur_gz
colcon build --symlink-install
```

After building successfully:

```bash
source install/setup.bash
```

For convenience, the workspace can also be sourced automatically in every new terminal:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Run the simulation

First, source ROS 2 and the workspace:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
```

Check the available launch files:

```bash
ros2 launch
```

Then run the project's launch file from the `launch/` directory:

```bash
ros2 launch ur3_draw_letter draw_letter.launch.py
```

## 7. Typical workflow

The complete execution sequence is:

```text
ROS 2
  │
  ├── Gazebo
  │     └── UR3/UR3e simulation
  │
  ├── MoveIt 2
  │     └── Motion planning
  │
  └── Drawing node
        └── Cartesian trajectory
              └── Letter drawing
```

In general:

1. Start Gazebo and the UR3/UR3e simulation.
2. Start MoveIt 2 and the robot controllers.
3. Start the letter-drawing node.
4. The node generates Cartesian waypoints.
5. MoveIt 2 plans/execut
   es the robot trajectory.
6. The robot follows the trajectory in simulation.

---

## 8. Useful commands

### Check ROS 2 nodes

```bash
ros2 node list
```

### Check available topics

```bash
ros2 topic list
```

### Check running controllers

```bash
ros2 control list_controllers
```

## 9. Troubleshooting

### `Package '<package_name>' not found`

Make sure the workspace has been built and sourced:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Check whether the package exists:

```bash
ros2 pkg list | grep <package_name>
```

---

### `ros2 launch` cannot find the launch file

Check the installed package:

```bash
ros2 pkg prefix ur3_draw_letter
```

Then verify that the launch file exists in the package.

Rebuild if necessary:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

### MoveIt 2 cannot plan the trajectory

Check that:

* Gazebo is running.
* The robot description has loaded correctly.
* The required controllers are active.
* MoveIt 2 is running.
* The robot's planning group matches the configuration.
* The requested Cartesian waypoints are inside the robot workspace.

Check controllers:

```bash
ros2 control list_controllers
```

Check nodes:

```bash
ros2 node list
```

---

### Gazebo starts but the robot does not move

Check whether the required controllers are active:

```bash
ros2 control list_controllers
```

Also check the terminal where Gazebo and the controllers were launched for error messages.

---

## 10. Project structure

```text
ur_gz/
├── launch/
│   └── ...
├── src/
│   └── ...
└── README.md
```

### `launch/`

Contains ROS 2 launch files used to start the simulation and required components.

### `src/`

Contains the source code responsible for robot control and Cartesian letter drawing.

---

## 11. Repository

Source code:

https://github.com/linhnghu/ur_gz

## 12. Notes

This project is intended for simulation and development with ROS 2, Gazebo and MoveIt 2.

Make sure that the installed ROS 2 distribution, Gazebo version, MoveIt 2 version and Universal Robots packages are compatible with each other.
