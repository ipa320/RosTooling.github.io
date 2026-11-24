# System generation

From a [system model](RosSystemModelDescription.md) (`.rossystem`) RosTooling generates a ROS 2 package that starts the system: the launch file, the parameter files, a README and a diagram. The nodes themselves come from the [package models](Ros2CodeGeneration.md) that the system refers to.

## Generating

**In VS Code**, right-click a `.rossystem` file in the editor and choose **Generate ROS2 Package**.

**With an AI agent**, the RosTooling MCP server offers the tool `generate_rossystem_code` with the path of the `.rossystem` file.

The package is written to `src-gen/<system>/`. Before anything is written, the model is validated, and errors stop the generation with the messages of the [validator](Validation.md), so no launch file for a model that is wrong is created.

## What is generated

For a system `demo_system` with two nodes, one connection and a parameter that the system sets:

```text
demo_system/
├── package.xml
├── CMakeLists.txt
├── README.md                     the nodes, the connections, how to build and start
├── launch/demo_system.launch.py  starts the system
├── config/listener.yaml          parameter file of one node
└── resource/demo_system.puml     PlantUML diagram of the system
```

| File | Content |
| --- | --- |
| `launch/<system>.launch.py` | The launch file. This is the part you use. |
| `config/<node>.yaml` | The parameter values that the system model sets for a node, in the ROS 2 parameter file format. |
| `README.md` | The list of nodes and their connections, the commands to install the dependencies, build the workspace and start the launch file. |
| `resource/<system>.puml` | A [PlantUML](https://plantuml.com/) description of the system, see [System visualization](SystemModelsVisualization.md). |
| `package.xml`, `CMakeLists.txt` | The package: it depends on the packages of the nodes and on the launch packages, and installs the launch and config folders. |

The package is regenerated every time, so do not edit its files by hand.

## The launch file

The launch file starts each node of the system with the `Node` action of `launch_ros`, or with `LifecycleNode` when the node is a lifecycle node. It uses the name of the artifact as the executable and the label of the node in the system as the node name. This is the launch file generated for the system above:

```python
def generate_launch_description():
  ld = LaunchDescription()

  # *** PARAMETERS ***
  rate_arg = DeclareLaunchArgument(
    "rate", default_value=TextSubstitution(text="5.0")
  )
  ld.add_action(rate_arg)

  # *** ROS 2 standalone nodes ***
  talker = Node(
    package="talker_pkg",
    executable="talker",
    output='screen',
    name="talker"
  )
  listener = Node(
    package="listener_pkg",
    executable="listener",
    output='screen',
    name="listener",
    remappings=[
      ("chatter_in", "chatter")],
    parameters=[{
    "rate": LaunchConfiguration("rate"),}]
  )
  # (the rest of the file adds the actions to the launch description)
```

### Connections

ROS connects publishers and subscribers by topic name. A connection of the model is therefore written as a remapping: the receiving side is remapped to the name of the sending side. In the example, `chatter_in` of the listener is remapped to `chatter`, the interface label of the talker. A connection does not rename anything on the sending side, so choose the labels of the senders to be the topic names you want.

### Parameters

Every parameter value that the system sets for a node becomes a launch argument with that value as its default, so it can be changed when the launch file is started:

```bash
ros2 launch demo_system demo_system.launch.py rate:=2.0
```

A parameter that the system declares itself under `parameters:` is also a launch argument, and `$(var name)` in a [launch substitution](RosSystemModelDescription.md#launch-substitutions) refers to it. Give it its starting value with `value:`. (With a RosTooling version newer than the current release, a `default:` on its type is used when there is no `value:`.) A string value with a launch substitution is passed to the launch system as it is, and the generated code makes sure that it stays a string.

### Namespaces

A `namespace:` of a node in the model becomes the namespace of its action in the launch file.

### Processes

By default every node is an executable of its own. The nodes of a [`processes:`](RosSystemModelDescription.md#processes) entry are started as components of one container, which hosts them in one process:

```python
  container_golden_container = ComposableNodeContainer(
    name="golden_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
      ComposableNode(
        package="golden_pkg",
        plugin="golden_pkg::ServiceNodeNode",
        name="service_node"
      ),
      # (more ComposableNode entries)
    ],
    output='screen'
  )
```

The container runs a single-threaded executor (`component_container`). With `threads` greater than 1 it is `component_container_mt`, which runs a multi-threaded executor. The nodes must be C++ nodes generated by [ROS 2 code generation](Ros2CodeGeneration.md), because they are loaded as components. Nodes that are not part of a process still start as executables.

### Subsystems

The launch file of a subsystem is included in the launch file of the system that uses it, so starting the outer system starts everything. A subsystem that has `fromFile:` is included through the launch file it names.

## Using the package

Copy or link the package into the `src/` folder of a ROS 2 workspace and build it together with the packages of the nodes:

```bash
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
ros2 launch demo_system demo_system.launch.py
```

The same commands are in the README of the generated package.

If you generate a system into a folder that already holds a package, for example the package of your nodes, the generator adds what is missing, the launch and config install rules and the launch dependencies, to its `CMakeLists.txt` and `package.xml` and leaves the rest alone.
