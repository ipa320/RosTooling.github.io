# Subsystems and existing launch files

The [previous tutorial](LearnRosSystemModels.md) composes a system from nodes. A system can also contain other systems, and it can describe a system that a launch file already starts. This tutorial shows both. The models use the same two package models as the previous tutorial, `awesome_pkg.ros2` and `awesome2_pkg.ros2`; the reference is [System language](RosSystemModelDescription.md#subsystems).

## Subsystems from a model

A subsystem is a system of its own, in its own `.rossystem` file. Here is one with a single node that publishes:

<!-- check: group=sub1 file=my_subsystem.rossystem with=tools/fixtures/awesome -->
```text
my_subsystem:
  nodes:
    "sensor":
      from: "awesome_pkg.awesome_node"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
```

Another system uses it under `subSystems:`, by the name of the system. A connection can refer to an interface of a node inside the subsystem by the path `<subsystem>::<node>::<interface>`:

<!-- check: group=sub1 file=top_system.rossystem with=tools/fixtures/awesome -->
```text
top_system:
  subSystems:
    "my_subsystem"
  nodes:
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
  connections:
    - ["my_subsystem::sensor::my_pub", "my_sub"]
```

In the Studio the subsystem is a dashed frame with its nodes, and the wire goes from the node inside to the node outside. You can also create such a connection by dragging, and the Studio writes the path for you, see [RosTooling Studio](../vscode/studio.md#the-system-view).

```{image} images/tut_subsystem.png
:alt: A subsystem frame with a connection to a node of the outer system
:width: 100%
```

Generate the package of `top_system`. Its launch file starts the node of the system and includes the launch file of the subsystem, and its `package.xml` depends on the package of the subsystem:

```python
  # *** ROS 2 subsystems (include launch files)***
  include_my_subsystem= IncludeLaunchDescription(
    PythonLaunchDescriptionSource([ get_package_share_directory('my_subsystem') + '/launch/my_subsystem.launch.py'])
  )
```

So the package of `my_subsystem` has to be generated, built and installed too.

## Existing systems with a launch file

Many ROS systems are started by a launch file that already exists. RosTooling does not need to generate another one. The attribute `fromFile` names the launch file: the package, a slash, and the path in the package, with the extension:

<!-- check: group=sub2 file=launch_system.rossystem with=tools/fixtures/awesome -->
```text
launch_system:
  fromFile: "bringup_pkg/launch/bringup.launch.py"
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
```

For such a system the generator does not write a launch file. It writes the README, with the instructions to install and start the system, and the diagram description, `resource/<system>.puml`. The nodes and interfaces are optional, but they document the system and are shown in the diagrams, so describe at least the important ones. A file with only the first line, `fromFile`, is valid.

A `fromFile` system is most useful as a subsystem of another system. The launch file of the other system includes it:

<!-- check: group=sub2 file=outer_system.rossystem with=tools/fixtures/awesome -->
```text
outer_system:
  subSystems:
    "launch_system"
  nodes:
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
  connections:
    - ["launch_system::node1::my_pub", "my_sub"]
```

```python
  include_launch_system= IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('bringup_pkg') + '/launch/bringup.launch.py'])
  )
```

The `package.xml` of `outer_system` depends on `bringup_pkg`. The [turtlebot tutorial](MobileBase_beginner.md) uses this to compose a simulation that already exists with new nodes.
