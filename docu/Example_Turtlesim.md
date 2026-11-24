# Tutorial: turtlesim

One of the first ROS tutorials is turtlesim, a small window with a turtle that moves with velocity commands ([official tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)). Here it is the example for modelling nodes that already exist. A system of two nodes is built, the turtle and the node that sends it commands from the keyboard, and the launch file is generated. It takes about 20 minutes and needs a ROS 2 installation with turtlesim (the commands use Jazzy).

**What you need:** the [VS Code extension](../vscode/overview.md), and the syntax from the tutorials about [package models](LearnRosModels.md) and [system models](LearnRosSystemModels.md).

## 1. Model the nodes

Both nodes are in the package `turtlesim` of the repository [ros_tutorials](https://github.com/ros/ros_tutorials). A model of an existing node describes what the node offers. You can write it by hand from the documentation of the node, as here, or extract it from the code, see [Static code analysis](StaticCodeAnalyis.md).

Create a folder, open it in VS Code and create `turtlesim.ros2`:

<!-- check: group=turtle file=turtlesim.ros2 -->
```text
turtlesim:
  fromGitRepo: "https://github.com/ros/ros_tutorials.git:jazzy"
  artifacts:
    turtle_teleop_key:
      node: turtle_teleop_key
      publishers:
        "turtle1/cmd_vel":
          type: "geometry_msgs/msg/Twist"
      parameters:
        scale_angular:
          type: Double
          default: 2.0
        scale_linear:
          type: Double
          default: 2.0
    turtlesim_node:
      node: turtlesim_node
      publishers:
        "turtle1/color_sensor":
          type: "turtlesim/msg/Color"
        "turtle1/pose":
          type: "turtlesim/msg/Pose"
      subscribers:
        "turtle1/cmd_vel":
          type: "geometry_msgs/msg/Twist"
      serviceservers:
        clear:
          type: "std_srvs/srv/Empty"
        kill:
          type: "turtlesim/srv/Kill"
        reset:
          type: "std_srvs/srv/Empty"
        spawn:
          type: "turtlesim/srv/Spawn"
        "turtle1/set_pen":
          type: "turtlesim/srv/SetPen"
        "turtle1/teleport_absolute":
          type: "turtlesim/srv/TeleportAbsolute"
        "turtle1/teleport_relative":
          type: "turtlesim/srv/TeleportRelative"
```

The names of the interfaces are the names that the code uses, `turtle1/cmd_vel` and so on. A name with a slash is written in double quotes. The artifacts are named like the executables of the package, `turtlesim_node` and `turtle_teleop_key`, because the launch file starts the executables by these names. `fromGitRepo` is the repository of the source code, a colon and the branch.

Right-click the file and choose **Open in RosTooling Visual Studio** to see the two nodes as forms:

```{image} images/tut_turtlesim_component.png
:alt: The turtlesim package in the Component view
:width: 100%
```

## 2. Compose the system

A system of the two nodes, with the interfaces that are to be connected, the subscription to the velocity commands of the turtle and the publication of the keyboard node. Create `turtlesim_system.rossystem`:

<!-- check: group=turtle file=turtlesim_system.rossystem -->
```text
turtlesim_system:
  nodes:
    "turtlesim":
      from: "turtlesim.turtlesim_node"
      interfaces:
        - "cmd_subscriber": sub-> "turtlesim_node::turtle1/cmd_vel"
    "key_teleop":
      from: "turtlesim.turtle_teleop_key"
      interfaces:
        - "cmd_publisher": pub-> "turtle_teleop_key::turtle1/cmd_vel"
  connections:
    - ["cmd_publisher", "cmd_subscriber"]
```

The `from:` entries refer to the nodes of `turtlesim.ros2`, in the form `<package>.<node>`. The interfaces refer to the ones of the model, `<artifact>::<interface>`. A system can only use interfaces that the original code has.

```{image} images/tut_turtlesim_system.png
:alt: The turtlesim system in RosTooling Studio
:width: 100%
```

## 3. Generate and launch

Right-click `turtlesim_system.rossystem` and choose **Generate ROS2 Package**. The package `turtlesim_system` is written to `src-gen/`. Its launch file starts both nodes and connects them with a remapping of the topic of both to the label of the connection:

```python
  turtlesim = Node(
    package="turtlesim",
    executable="turtlesim_node",
    output='screen',
    name="turtlesim",
    remappings=[
      ("turtle1/cmd_vel", "cmd_publisher")]
  )
  key_teleop = Node(
    package="turtlesim",
    executable="turtle_teleop_key",
    output='screen',
    name="key_teleop",
    remappings=[
      ("turtle1/cmd_vel", "cmd_publisher")]
  )
```

The nodes are in the installed package `turtlesim`, so the generated package only has to be built. Copy or link `src-gen/turtlesim_system` into a workspace and run:

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
colcon build --symlink-install
source install/setup.bash
ros2 launch turtlesim_system turtlesim_system.launch.py
```

The turtle window opens. The keyboard node reads the arrow keys from the terminal, which a launch file does not provide, so it stops with the message "Failed to get old console mode" when you start the system this way. Start it in a terminal of its own, with the same topic name:

```bash
ros2 run turtlesim turtle_teleop_key --ros-args -r turtle1/cmd_vel:=cmd_publisher
```

With the cursor keys in that terminal the turtle moves. You can also send a command by hand and watch the turtle change its position:

```bash
ros2 topic pub --once /cmd_publisher geometry_msgs/msg/Twist "{linear: {x: 2.0}}"
ros2 topic echo /turtle1/pose --once
```

The README of the generated package has the commands to create the workspace and call the launch file.
