# Tutorial: compose an existing system, the turtlebot in simulation

In this tutorial you compose systems from models of the catalogue: the turtlebot3 in the Gazebo simulation, a keyboard node to drive it, and the cartographer node to build a map. You do not write any node models, only systems. It takes about 30 minutes. The models and the generation are checked; the execution needs a ROS 2 installation with Gazebo and the turtlebot3 packages, which is not part of this tutorial's checks.

**What you need:** the [VS Code extension](../vscode/overview.md), which downloads the [catalogue](../vscode/catalogue.md) of models, and the tutorials about [system models](LearnRosSystemModels.md) and [subsystems](LearnRosSystemModels2.md).

## 1. The turtlebot model

The catalogue [RosModelsCatalog](https://github.com/ipa-nhg/RosModelsCatalog) has the system `turtlebot_gazebo` in `robots/turtlebot3/simulation/turtlebot_gazebo.rossystem`. It describes the nodes that the [launch file of the package turtlebot3_gazebo](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/main/turtlebot3_gazebo/launch) starts, and starts the system through that launch file:

```text
turtlebot_gazebo:
  fromFile: "turtlebot3_gazebo/launch/turtlebot3_world.launch.py"
  nodes:
    gazebo:
      from: "gazebo.gazebo"
    turtlebot3_diff_drive:
      from: "turtlebot3_diff_drive.turtlebot3_diff_drive"
      interfaces:
        - cmd_vel_diff: sub->"turtlebot3_diff_drive::cmd_vel"
        - tf: pub->"turtlebot3_diff_drive::tf"
        - odom_diff: pub->"turtlebot3_diff_drive::odom"
    turtlebot3_laserscan:
      from: "turtlebot3_laserscan.turtlebot3_laserscan"
      interfaces:
        - scan_turtlebot: pub-> "turtlebot3_laserscan::scan"
    # ... robot_state_publisher, turtlebot3_joint_state, turtlebot3_imu and camera_driver
```

(The listing is shortened.) `fromFile` is the launch file that starts the whole system. The nodes and interfaces are the ones that the author chose to expose for higher-level systems. How much to describe is the choice of the developer: a file with only the first two lines is valid, but the more is described, the better for documentation and composition. See [Existing systems with a launch file](LearnRosSystemModels2.md#existing-systems-with-a-launch-file).

## 2. Compose the simulation with a keyboard node

The catalogue also has a keyboard node, in `common/teleop_twist_keyboard.ros2`. It takes the keys and publishes velocity commands, `geometry_msgs/msg/Twist`. Create a new system file, `turtlebot_with_teleop.rossystem`, with the turtlebot as a subsystem and the keyboard node:

<!-- check: group=mb1 file=turtlebot_with_teleop.rossystem -->
```text
turtlebot_with_teleop:
  subSystems:
    "turtlebot_gazebo"
  nodes:
    "teleop_key":
      from: "teleop_twist_keyboard.teleop_twist_keyboard"
```

Generate the package with **Generate ROS 2 Package**. It is written to `src-gen/turtlebot_with_teleop/`. Its README has the instructions to install the dependencies and start the system, and its launch file includes the launch file of the simulation next to the keyboard node. The simulation needs the model of the robot to be set:

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

A nice property of models during the design is how easy it is to try other components. The catalogue has another keyboard node, the one of the turtlebot, in `robots/turtlebot3/turtlebot3_teleop.ros2`. Change one line:

<!-- check: group=mb2 file=turtlebot_with_teleop.rossystem -->
```text
turtlebot_with_teleop:
  subSystems:
    "turtlebot_gazebo"
  nodes:
    "teleop":
      from: "turtlebot3_teleop.teleop_keyboard"
```

After generating again, the package depends on the other implementation and starts it with the simulation.

## 3. Add the cartographer node

The system can grow in the same way. The catalogue has a system for cartographer with the configuration for the turtlebot, in `robots/turtlebot3/cartographer.rossystem`. Use it as another subsystem:

<!-- check: group=mb3 file=turtlebot_with_teleop_and_cartographer.rossystem -->
```text
turtlebot_with_teleop_and_cartographer:
  subSystems:
    "turtlebot_gazebo"
    "cartographer"
  nodes:
    "teleop":
      from: "turtlebot3_teleop.teleop_keyboard"
```

Generate the package, follow its README, and start the system. The robot moves with the keys, and in RViz you can watch the map. To save it, use the standard command:

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 4. Make the connections explicit

In the systems above nothing says which ports are connected. This works because ROS connects a publisher and a subscriber that have the same topic name and type at run time. But the designer does not see it, and the model does not check it. If you open the system in RosTooling Studio you see only the subsystems and the node, without wires:

```{image} images/tut_mobilebase_implicit.png
:alt: The system without explicit connections
:width: 100%
```

This is incomplete as documentation, and for people who do not know ROS. Be as explicit as is reasonable. The subsystems have labels for their interfaces, `cmd_vel_diff`, `scan_turtlebot` for the laser scan and `odom_diff` for the odometry on the side of the robot, and `scan_cart` and `odom_cart` on the side of cartographer. Expose the interface of the keyboard node and connect them:

<!-- check: group=mb4 file=turtlebot_with_teleop_and_cartographer.rossystem -->
```text
turtlebot_with_teleop_and_cartographer:
  subSystems:
    "turtlebot_gazebo"
    "cartographer"
  nodes:
    "teleop":
      from: "turtlebot3_teleop.teleop_keyboard"
      interfaces:
        - "cmd_vel": pub-> "teleop_keyboard::cmd_vel"
  connections:
    - ["cmd_vel", "cmd_vel_diff"]
    - ["scan_turtlebot", "scan_cart"]
    - ["odom_diff", "odom_cart"]
```

Now the validator checks that the ports exist and have the same type, the diagram shows the wires, and the model documents the system for others:

```{image} images/tut_mobilebase_explicit.png
:alt: The system with explicit connections
:width: 100%
```
