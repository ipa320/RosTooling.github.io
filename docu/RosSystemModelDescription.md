# System language (.rossystem)

A system model describes a set of ROS nodes that run together and how they are connected. It uses the extension `.rossystem` and refers to nodes and interfaces that are described in [package models](RosModelDescription.md). From a system model the tooling generates the launch file and the package around it ([System generation](CodeGeneration.md)).

Like the package language, the system language is indented text in a YAML-like style that is checked by the validator, not by a YAML parser. Names that refer to other models are written in double quotes.

## A first system

<!-- check: group=basic file=talker_pkg.ros2 -->
```text
talker_pkg:
  artifacts:
    talker:
      node: talker
      publishers:
        chatter:
          type: "std_msgs/msg/String"
```

<!-- check: group=basic file=listener_pkg.ros2 -->
```text
listener_pkg:
  artifacts:
    listener:
      node: listener
      subscribers:
        chatter_in:
          type: "std_msgs/msg/String"
```

<!-- check: group=basic file=demo.rossystem -->
```text
demo_system:
  nodes:
    "talker":
      from: "talker_pkg.talker"
      interfaces:
        - "chatter": pub-> "talker::chatter"
    "listener":
      from: "listener_pkg.listener"
      interfaces:
        - "chatter_in": sub-> "listener::chatter_in"
  connections:
    - ["chatter", "chatter_in"]
```

The system `demo_system` has two nodes, each taken from a package model, and one connection from the publisher of the first to the subscriber of the second. The sections below explain each part.

## Structure

```text
system
├── fromFile        (optional) an existing launch file that starts this system
├── subSystems      (optional) other systems that are part of this one
├── processes       (optional) which nodes share a process
├── nodes           the nodes of the system
│   └── node        from, namespace, slots, interfaces, parameters
├── parameters      (optional) parameters of the whole system
└── connections     which interfaces are connected
```

The sections can be written in any order.

## Nodes

Every entry under `nodes:` is a node of the system. Its name is a label that must be unique in the system, written in double quotes. The members follow in this order: `from:`, `namespace:`, `slots:`, `interfaces:`, `parameters:`.

`from:` names the node description as `"<package>.<node>"`: the name of the package model, a dot and the name of the **node** in it. It is the node name that counts, not the artifact name. For the package below, the reference is `"moveit2_scripts.simple_pick_and_place"`, even though the artifact is called `pick_and_place`.

```text
moveit2_scripts:
  artifacts:
    pick_and_place:
      node: simple_pick_and_place
```

`namespace:` puts the node in a ROS namespace when the launch file starts it.

### Interfaces

A node has all the interfaces of its package model. List an interface in `interfaces:` when a connection needs to refer to it, or to give it another name in this system. An entry has a label, an arrow that says what kind of interface it is, and the interface it refers to:

| Arrow | Interface |
| --- | --- |
| `pub->` | publisher |
| `sub->` | subscriber |
| `ss->` | service server |
| `sc->` | service client |
| `as->` | action server |
| `ac->` | action client |

The reference after the arrow is `"<artifact>::<interface>"`: the name of the **artifact** in the package model, two colons and the interface name. Note the difference to `from:`: the node reference uses the node name, the interface reference uses the artifact name. When an artifact and its node have the same name, as in the examples here, both work, which is why the difference is easy to miss.

Within one node, each interface label must be unique, because connections refer to it by name. A repeated label is an error.

## Connections

A connection joins an output to an input. It is written as a pair of interface labels, from the sender to the receiver:

| From | To |
| --- | --- |
| publisher (`pub->`) | subscriber (`sub->`) |
| service server (`ss->`) | service client (`sc->`) |
| action server (`as->`) | action client (`ac->`) |

Both ends must have the same message, service or action type. The validator reports a connection in the wrong direction, with different types, or to an interface that does not belong to the system.

When two nodes have interfaces with the same label, a short label is ambiguous. Qualify it with the node label and two colons: `"<node>::<interface>"`.

<!-- check: group=qualified file=pkg_a.ros2 -->
```text
pkg_a:
  artifacts:
    node_a:
      node: node_a
      publishers:
        scan:
          type: "sensor_msgs/msg/LaserScan"
```

<!-- check: group=qualified file=pkg_b.ros2 -->
```text
pkg_b:
  artifacts:
    node_b:
      node: node_b
      subscribers:
        scan:
          type: "sensor_msgs/msg/LaserScan"
```

<!-- check: group=qualified file=qualified.rossystem -->
```text
qualified_system:
  nodes:
    "a":
      from: "pkg_a.node_a"
      interfaces:
        - "scan": pub-> "node_a::scan"
    "b":
      from: "pkg_b.node_b"
      interfaces:
        - "scan": sub-> "node_b::scan"
  connections:
    - ["a::scan", "b::scan"]
```

## Parameters

### Setting a parameter of a node

A node can give a new value to a parameter that its package model declares. The entry has a label, a reference `"<artifact>::<parameter>"` and the value on the next line. Write the value as the declared type requires: `true` and `false` for booleans, `1.0` and not `1` for a double, strings in double quotes. [Parameters](ParametersAPI.md) lists all types.

<!-- check: group=params file=params_pkg.ros2 -->
```text
params_pkg:
  artifacts:
    filter:
      node: filter
      parameters:
        rate:
          type: Double
          default: 10.0
        topic_name:
          type: String
          default: "scan"
        enabled:
          type: Boolean
          default: true
```

<!-- check: group=params file=params.rossystem -->
```text
params_system:
  parameters:
    "filter_rate":
      type: Double
      value: 20.0
    "filter_topic":
      type: String
      value: "scan"
  nodes:
    "filter":
      from: "params_pkg.filter"
      parameters:
        - "rate": "filter::rate"
          value: 20.0
        - "topic_name": "filter::topic_name"
          value: "$(var filter_topic)"
        - "enabled": "filter::enabled"
          value: false
```

### Parameters of the whole system

`parameters:` directly under the system declares parameters of the system itself. Each one has a type and a `value:`, which is its starting value. A `default:` on the type is used when there is no `value:`, but this needs a RosTooling version that is newer than the current release; until then write `value:`. In the generated launch file they become launch arguments, so the system can be configured when it is started: `ros2 launch <package> <system>.launch.py filter_rate:=30.0`.

### Launch substitutions

A string value that contains `$(...)` is passed to the ROS 2 launch system and computed when the system starts. The validator checks that the parentheses are balanced and that the substitution is one of the known ones, such as `var`, `env`, `find-pkg-share` and `command`. A `$(var name)` that does not refer to a declared system parameter is a warning, and a substitution assigned to a parameter that is not a string is a warning too, because the launch file then passes the value as a string.

<!-- check: group=substitution file=subst_pkg.ros2 -->
```text
subst_pkg:
  artifacts:
    loader:
      node: loader
      parameters:
        config_file:
          type: String
        robot_name:
          type: String
```

<!-- check: group=substitution file=subst.rossystem -->
```text
subst_system:
  parameters:
    "robot":
      type: String
      value: "robot1"
  nodes:
    "loader":
      from: "subst_pkg.loader"
      parameters:
        - "config_file": "loader::config_file"
          value: "$(find-pkg-share subst_pkg)/config/robot.yaml"
        - "robot_name": "loader::robot_name"
          value: "$(var robot)"
```

## Subsystems

A system can contain other systems. List them under `subSystems:` by name. The subsystem is defined in its own `.rossystem` file, and a connection can refer to an interface of one of its nodes with the path `"<subsystem>::<node>::<interface>"`.

<!-- check: group=subsystem file=sensors.rossystem -->
```text
sensors_system:
  nodes:
    "talker":
      from: "talker_pkg.talker"
      interfaces:
        - "chatter": pub-> "talker::chatter"
```

<!-- check: group=subsystem file=talker_pkg.ros2 -->
```text
talker_pkg:
  artifacts:
    talker:
      node: talker
      publishers:
        chatter:
          type: "std_msgs/msg/String"
```

<!-- check: group=subsystem file=listener_pkg.ros2 -->
```text
listener_pkg:
  artifacts:
    listener:
      node: listener
      subscribers:
        chatter_in:
          type: "std_msgs/msg/String"
```

<!-- check: group=subsystem file=robot.rossystem -->
```text
robot_system:
  subSystems:
    "sensors_system"
  nodes:
    "listener":
      from: "listener_pkg.listener"
      interfaces:
        - "chatter_in": sub-> "listener::chatter_in"
  connections:
    - ["sensors_system::talker::chatter", "chatter_in"]
```

## Processes

By default every node runs in its own process. A `processes:` section groups nodes into one process, as composable nodes in one container. Each process lists its nodes and may set the number of threads.

<!-- check: group=processes file=talker_pkg.ros2 -->
```text
talker_pkg:
  artifacts:
    talker:
      node: talker
      publishers:
        chatter:
          type: "std_msgs/msg/String"
```

<!-- check: group=processes file=listener_pkg.ros2 -->
```text
listener_pkg:
  artifacts:
    listener:
      node: listener
      subscribers:
        chatter_in:
          type: "std_msgs/msg/String"
```

<!-- check: group=processes file=processes.rossystem -->
```text
process_system:
  processes:
    "pair":
      nodes: ["talker", "listener"]
      threads: 2
  nodes:
    "talker":
      from: "talker_pkg.talker"
    "listener":
      from: "listener_pkg.listener"
```

A node can be in only one process. The nodes named in a process must be declared in the same system, and the number of threads cannot be negative.

## Attach plugins to a slot

A node whose package model declares a [plugin slot](RosModelDescription.md#plugins) gets its plugins from the system. The `slots:` section of the node names the slot and lists the plugins, written as `<package>.<plugin>`, in square brackets. Without `multiple: true` on the slot, at most one plugin can be attached; `[]` states that none is attached, so a default of the slot does not apply.

<!-- check: group=slots file=nav_pkg.ros2 -->
```text
nav_pkg:
  pluginbases:
    Planner:
      class: "nav_core::GlobalPlanner"
      header: "nav_core/global_planner.hpp"
      loader: pluginlib
  artifacts:
    planner_server:
      node: planner_server
      pluginslots:
        planner:
          base: nav_pkg.Planner
          parameter: planner_plugin
    straight_line:
      plugins:
        StraightLine:
          implements: nav_pkg.Planner
```

<!-- check: group=slots file=slots.rossystem -->
```text
nav_system:
  nodes:
    "server":
      from: "nav_pkg.planner_server"
      slots:
        planner: [nav_pkg.StraightLine]
```

The validator reports a slot that is attached twice, a plugin that is listed twice, several plugins in a slot that takes one, and a plugin that does not implement the base of the slot.

## Systems that start an existing launch file

`fromFile:` points to a launch file that already starts the system, in the form `"<package>/<path>/<file>.launch.py"`. This is how an existing ROS system is described without modelling its nodes. When another system lists it under `subSystems:`, the generated launch file includes that launch file.

<!-- check: group=fromfile file=fromfile.rossystem -->
```text
existing_system:
  fromFile: "demo_bringup/launch/bringup.launch.py"
```

## What the validator checks

The validator reports nodes that are declared twice, interfaces that do not exist, connections in the wrong direction or between different types, invalid launch substitutions, wrong parameter values, and problems with processes and plugin slots. The complete list is on the [validation page](Validation.md).
