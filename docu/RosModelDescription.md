# Package language (.ros1, .ros2)

A package model describes one ROS package: the nodes it provides, the interfaces and parameters of each node and, for ROS 2, quality of service, lifecycle behaviour and plugins. Packages for ROS 2 use the extension `.ros2`, packages for ROS 1 use `.ros1`. The editor recognises the extension and switches on highlighting, validation and completion (`Ctrl` + `Space`).

The files are indented text in a YAML-like style, but they are not YAML. Words such as `artifacts:` and `publishers:` are keywords of the language and are checked by the validator, not by a YAML parser. This page uses `.ros2`; [ROS 1 packages](#ros-1-packages) are the same without the ROS 2 features.

## A first model

<!-- check: group=first file=demo_pkg.ros2 -->
```text
demo_pkg:
  artifacts:
    talker:
      node: talker
      publishers:
        chatter:
          type: "std_msgs/msg/String"
```

The package is `demo_pkg`. It has one artifact, `talker` (the executable that the build produces, named as in `CMakeLists.txt`), and the artifact holds one node, also called `talker`, that publishes `chatter` with the message type `std_msgs/msg/String`.

## Structure

```text
package
├── fromGitRepo        (optional) where the source code lives
├── pluginbases        (optional) plugin interfaces the package defines
├── artifacts
│   └── artifact       one executable or one plugin library
│       ├── node       one node, with its interfaces and parameters
│       └── plugins    or plugins, never both
└── dependencies       (optional) extra package dependencies
```

The order of the package members is fixed: `fromGitRepo`, then `pluginbases` and `artifacts` (in either order), then `dependencies`. Inside a node the sections can come in any order.

An artifact holds either one node or a set of [plugins](#plugins), never both. For several nodes, declare several artifacts.

<!-- check: group=structure file=structure_pkg.ros2 -->
```text
structure_pkg:
  fromGitRepo: "https://github.com/example/structure_pkg.git:main"
  artifacts:
    sensor_driver:
      node: sensor_driver
      publishers:
        data:
          type: "sensor_msgs/msg/Imu"
    sensor_monitor:
      node: sensor_monitor
      subscribers:
        data:
          type: "sensor_msgs/msg/Imu"
  dependencies: [ExternalDependency diagnostic_updater, ExternalDependency tf2_ros]
```

`fromGitRepo` takes the repository address, a colon and the branch. Each entry of `dependencies` is one of two forms:

- the name of another package model, for a package that is described in the workspace or the catalogue, such as `std_msgs` or `base_pkg`;
- `ExternalDependency <name>`, for a ROS package that has no model, such as `rclcpp`. A name without a model behind it is an error unless it is written this way.

`dependencies:` comes after `artifacts:`; written before it, the file does not parse.

Every entry is added as a `<depend>` entry to the generated `package.xml`.

:::{note}
Naming another package model in `dependencies:` needs a RosTooling version that is newer than the current release. The server of the current release only accepts `ExternalDependency`.
:::

<!-- check: group=dependency file=base_pkg.ros2 skip="package dependencies do not resolve yet" -->
```text
base_pkg:
  artifacts:
    base_node:
      node: base_node
```

<!-- check: group=dependency file=app_pkg.ros2 skip="package dependencies do not resolve yet" -->
```text
app_pkg:
  artifacts:
    app_node:
      node: app_node
  dependencies: [base_pkg, ExternalDependency tf2_ros]
```

### Naming rules

Names of packages, artifacts, nodes and parameters follow the ROS naming conventions ([REP 144](https://ros.org/reps/rep-0144.html)): lowercase letters, digits and underscores, starting with a letter. Write them without quotes. The validator reports a capital letter in a package name as an error, and in an artifact, node or parameter name as a warning. Message and service types, repository addresses and string values are written in double quotes.

A few words are keywords of the language and cannot be used as an unquoted name: `default`, `feedback`, `goal`, `message`, `node`, `request`, `response`, `result`, `type` and `value`. If an interface really has such a name, write the name in double quotes.

<!-- check: group=keywords file=keywords_pkg.ros2 -->
```text
keywords_pkg:
  artifacts:
    arm_client:
      node: arm_client
      subscribers:
        "goal":
          type: "std_msgs/msg/String"
```

## Interfaces

A node lists its communication interfaces in one section per kind. Each interface has a name and a `type:` that refers to a message, service or action type. The type is looked up in the [communication object models](NewCommunicationObjects.md) and in the catalogue, so a misspelled type is reported while you type.

| Section | Interface | Type refers to |
| --- | --- | --- |
| `publishers:` | topic publisher | message, `pkg/msg/Name` |
| `subscribers:` | topic subscriber | message, `pkg/msg/Name` |
| `serviceservers:` | service server | service, `pkg/srv/Name` |
| `serviceclients:` | service client | service, `pkg/srv/Name` |
| `actionservers:` | action server | action, `pkg/action/Name` |
| `actionclients:` | action client | action, `pkg/action/Name` |

<!-- check: group=interfaces file=interfaces_pkg.ros2 -->
```text
interfaces_pkg:
  artifacts:
    scan_filter:
      node: scan_filter
      publishers:
        filtered_scan:
          type: "sensor_msgs/msg/LaserScan"
      subscribers:
        scan:
          type: "sensor_msgs/msg/LaserScan"
      serviceservers:
        reset:
          type: "std_srvs/srv/Empty"
      serviceclients:
        calibrate:
          type: "std_srvs/srv/Trigger"
      actionservers:
        follow:
          type: "control_msgs/action/FollowJointTrajectory"
      actionclients:
        follow_remote:
          type: "control_msgs/action/FollowJointTrajectory"
      parameters:
        range_max:
          type: Double
          default: 30.0
```

An interface name only has to be unique within its section of the artifact. Connections in a [system model](RosSystemModelDescription.md) refer to interfaces by name.

### Namespaces

An interface can carry a namespace, `ns:`, with one of the values `GlobalNamespace`, `RelativeNamespace` or `PrivateNamespace`.

<!-- check: group=namespace file=namespace_pkg.ros2 -->
```text
namespace_pkg:
  artifacts:
    diag:
      node: diag
      publishers:
        status:
          type: "std_msgs/msg/String"
          ns: PrivateNamespace
```

## Quality of service

In ROS 2 every interface and every parameter can have a `qos:` block. All its entries are optional and can be written in any order.

| Entry | Values |
| --- | --- |
| `profile` | `default_qos`, `services_qos`, `sensor_qos`, `parameter_qos` |
| `history` | `keep_last`, `keep_all` |
| `depth` | a whole number |
| `reliability` | `best_effort`, `reliable` |
| `durability` | `transient_local`, `volatile` |
| `liveliness` | `automatic`, `manual` |
| `lease_duration`, `lifespan`, `deadline` | a number of nanoseconds in double quotes, or `infinite` |

<!-- check: group=qos file=qos_pkg.ros2 -->
```text
qos_pkg:
  artifacts:
    camera:
      node: camera
      publishers:
        image:
          type: "sensor_msgs/msg/Image"
          qos:
            profile: sensor_qos
            reliability: best_effort
            history: keep_last
            depth: 5
            deadline: "100000000"
            liveliness: automatic
            lease_duration: infinite
```

The profile sets the starting values and the other entries override them. The generated code passes the result to the publisher or subscriber when it creates it.

## Lifecycle nodes

Write `lifecycle: true` directly under `node:` to model a managed node (`rclcpp_lifecycle::LifecycleNode` in C++, `rclpy.lifecycle.Node` in Python). Interfaces of a lifecycle node can state the lifecycle states in which they are active with `active_in:`, using the states `Unconfigured`, `Inactive`, `Active` and `Finalized`.

<!-- check: group=lifecycle file=lifecycle_pkg.ros2 -->
```text
lifecycle_pkg:
  artifacts:
    planner:
      node: planner
      lifecycle: true
      publishers:
        plan:
          type: "std_msgs/msg/String"
          active_in: [Active]
      subscribers:
        goal_in:
          type: "std_msgs/msg/String"
          active_in: [Inactive, Active]
```

`active_in:` is an error on a node that is not a lifecycle node, and a repeated state is a warning.

## Parameters

A node lists its parameters under `parameters:`, each with a `type:` and an optional default or value. The types, the value rules and the way a system overrides them are described in [Parameters](ParametersAPI.md).

## Plugins

A package can define plugin interfaces, provide plugins that implement them, and give a node a slot that takes plugins. These are the three keywords involved. The pluginlib family of ROS 2 packages, such as the Nav2 planners, is the typical use.

<!-- check: group=plugins file=nav_pkg.ros2 -->
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

- `pluginbases:` names a plugin interface by its C++ class, its header and the loader that finds implementations. RosTooling does not generate the base; it only states the facts that the other parts need.
- `plugins:` in an artifact lists plugins. Each one says which base it `implements:` and may declare interfaces and parameters of its own. An artifact with plugins is built as a shared library.
- `pluginslots:` in a node declares where plugins plug in: the `base:` they must implement and the `parameter:` that carries the chosen plugins. Optional entries are `multiple: true` (several plugins may attach), `format: library` (the parameter takes library names, not class names) and `default:` (a plugin that applies when a system attaches none).

A [system model](RosSystemModelDescription.md#attach-plugins-to-a-slot) chooses the plugins for each slot. Plugin support is a preview and may still change before the release.

## ROS 1 packages

A `.ros1` file has the same structure, but without the ROS 2 features: no `qos:`, no `lifecycle:` and no `active_in:`. Types are written the same way as in ROS 2.

```text
teleop:
  artifacts:
    teleop_twist_joy_node:
      node: teleop_twist_joy_node
      publishers:
        cmd_vel:
          type: "geometry_msgs/msg/Twist"
      subscribers:
        joy:
          type: "sensor_msgs/msg/Joy"
```

## What the validator checks

The validator reports naming problems, unknown types, wrong parameter values, invalid QoS values, `active_in` on a node that is not a lifecycle node and a package with unknown plugin loaders or slots. The complete list, with the message each rule gives, is on the [validation page](Validation.md).
