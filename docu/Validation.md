# Validation and diagnostics

Every model file is checked while you edit it. The checks are the same in all tools, so a model that is clean in the editor is clean in generation, in the diagram and for an AI agent that uses the MCP tools. A diagnostic has one of three severities:

| Severity | Meaning | Effect |
| --- | --- | --- |
| **Error** | The model is wrong or cannot be resolved. | Code generation is refused until the errors are fixed. |
| **Warning** | The model is valid, but something is probably not what you meant. | Generation continues. |
| **Info** | A hint, for example the format of a value. | None. |

Generating code first validates the model and reports the errors it finds, so no half-valid launch file or node wrapper is written.

The sections below list the checks by what they are about, with the message you get. Each example is a complete model; the diagnostic is the one the server reports for it.

## Names

| Check | Severity |
| --- | --- |
| A package name contains a capital letter | Error |
| An artifact, node or parameter name contains a capital letter | Warning |
| A name that is a keyword of the language is not quoted | Error (parse error) |

<!-- check: group=name file=Bad_Name.ros2 expect="Capital letters are not allowed" -->
```text
Bad_Name:
  artifacts:
    node_artifact:
      node: node_artifact
```

The message is "The name of a package has to follow the ROS naming conventions: Capital letters are not allowed". See [Naming rules](RosModelDescription.md#naming-rules).

## Types and values

| Check | Severity |
| --- | --- |
| A message, service or action type cannot be found | Error, with a quick fix that points to the catalogue |
| A parameter value does not fit the declared type, or a list has the wrong length | Error |
| A QoS duration is not a string of nanoseconds or `infinite` | Error |
| A value uses a format that the editor can explain, such as a list, a struct or a binary value | Info |

The messages name the expected and the given type, for example "Mismatched input ParameterInteger expecting ParameterDoubleType" when `1` is given to a `Double` parameter. See [Parameters](ParametersAPI.md).

## Lifecycle

<!-- check: group=lifecycle file=lc_pkg.ros2 expect="can only be specified if the parent node is configured as a lifecycle node" -->
```text
lc_pkg:
  artifacts:
    planner:
      node: planner
      publishers:
        plan:
          type: "std_msgs/msg/String"
          active_in: [Active]
```

`active_in` on a node that does not have `lifecycle: true` is an error. A repeated state in the list is a warning.

## Connections

In a [system model](RosSystemModelDescription.md#connections) the validator checks that

- both ends of a connection are interfaces of nodes in the system (error),
- the first end is a publisher, service server or action server and the second end is the matching subscriber, service client or action client (error),
- both ends have the same type (error),
- the interface labels of one node are unique (error).

<!-- check: group=direction file=pub_pkg.ros2 -->
```text
pub_pkg:
  artifacts:
    pub_node:
      node: pub_node
      publishers:
        out:
          type: "std_msgs/msg/String"
```

<!-- check: group=direction file=sub_pkg.ros2 -->
```text
sub_pkg:
  artifacts:
    sub_node:
      node: sub_node
      subscribers:
        inp:
          type: "std_msgs/msg/String"
```

<!-- check: group=direction file=direction.rossystem expect="the output port can have only one of the following types" -->
```text
direction_system:
  nodes:
    "p":
      from: "pub_pkg.pub_node"
      interfaces:
        - "out": pub-> "pub_node::out"
    "s":
      from: "sub_pkg.sub_node"
      interfaces:
        - "inp": sub-> "sub_node::inp"
  connections:
    - ["inp", "out"]
```

Here the connection is written from the subscriber to the publisher, so the first end is not an output. Swap the two labels to fix it.

<!-- check: group=mismatch file=pub_pkg.ros2 -->
```text
pub_pkg:
  artifacts:
    pub_node:
      node: pub_node
      publishers:
        out:
          type: "std_msgs/msg/String"
```

<!-- check: group=mismatch file=sub_pkg.ros2 -->
```text
sub_pkg:
  artifacts:
    sub_node:
      node: sub_node
      subscribers:
        inp:
          type: "std_msgs/msg/Int32"
```

<!-- check: group=mismatch file=mismatch.rossystem expect="have different types" -->
```text
mismatch_system:
  nodes:
    "p":
      from: "pub_pkg.pub_node"
      interfaces:
        - "out": pub-> "pub_node::out"
    "s":
      from: "sub_pkg.sub_node"
      interfaces:
        - "inp": sub-> "sub_node::inp"
  connections:
    - ["out", "inp"]
```

The message is "A connection can only be formed by interfaces with the same type", followed by the two interface names.

## Processes

| Check | Severity |
| --- | --- |
| A node listed in a process is not part of the system | Error |
| A node is in more than one process | Error |
| The number of threads is negative | Error |

## Launch files and substitutions

| Check | Severity |
| --- | --- |
| `fromFile:` is not in the form `package/path/file.launch.py` | Error |
| A launch substitution has unbalanced parentheses | Error |
| A launch substitution name is not one that the launch system knows | Error |
| `$(var name)` names a variable that is not a parameter of the system | Warning |
| A launch substitution is assigned to a parameter that is not a string | Warning |
| A parameter uses a launch substitution, so its value is not checked | Info |

<!-- check: group=subst file=subst_pkg.ros2 -->
```text
subst_pkg:
  artifacts:
    loader:
      node: loader
      parameters:
        config_file:
          type: String
```

<!-- check: group=subst file=subst.rossystem expect="Unknown launch substitution" -->
```text
subst_system:
  nodes:
    "loader":
      from: "subst_pkg.loader"
      parameters:
        - "config_file": "loader::config_file"
          value: "$(locate something)"
```

## Plugins

| Check | Severity |
| --- | --- |
| The `class` of a plugin base is not a C++ class with a namespace, such as `nav_core::GlobalPlanner` | Error |
| The `loader` of a plugin base is not `pluginlib` and no enabled plugin provides it | Error |
| A node declares two slots with the same name, or two slots with the same parameter | Error |
| A slot parameter is also declared under `parameters:` of the node | Error |
| The `default:` plugin of a slot does not implement the base of the slot | Error |
| A system attaches a slot twice, a plugin twice, or several plugins to a slot that takes one | Error |
| A system attaches a plugin that does not implement the base of the slot | Error |
| An `@` block is not known to any enabled plugin | Warning |
| An `@` block name is claimed by several plugins | Error |

<!-- check: group=loader file=loader_pkg.ros2 expect="which no enabled plugin registers" -->
```text
loader_pkg:
  pluginbases:
    Behavior:
      class: "my_ns::Behavior"
      header: "my_ns/behavior.hpp"
      loader: custom
```

A model that uses an `@` block of a plugin that is not enabled still opens and generates: the unknown block is only a warning, so models stay usable on a machine without the plugin.

<!-- check: group=unknownblock file=block_pkg.ros2 expect="Unknown extension block" -->
```text
block_pkg:
  artifacts:
    worker:
      node: worker
      @custom:
        level: 3
```

<!-- check: group=slotmany file=nav_pkg.ros2 -->
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
    plugins_lib:
      plugins:
        StraightLine:
          implements: nav_pkg.Planner
        Curved:
          implements: nav_pkg.Planner
```

<!-- check: group=slotmany file=slots.rossystem expect="takes one plugin" -->
```text
nav_system:
  nodes:
    "server":
      from: "nav_pkg.planner_server"
      slots:
        planner: [nav_pkg.StraightLine, nav_pkg.Curved]
```

Set `multiple: true` on the slot to allow several plugins.

## Where the diagnostics appear

- In **VS Code**, errors and warnings are underlined in the editor and listed in the Problems view. Hover over the underline for the message.
- In **Eclipse**, they appear as markers in the editor and the Problems view.
- An **AI agent** that uses the RosTooling MCP server calls `validate_ros_model` and receives the same diagnostics with their line numbers.
