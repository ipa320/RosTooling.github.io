# Plugins: example, BehaviorTree

:::{note}
**Preview.** The plugin support described in this section is not released yet and may still change.
:::

The BehaviorTree plugin is a RosTooling plugin that uses everything the [other pages](PluginConcepts.md) describe: a loader, a block and generated code. It models the leaf nodes of [BehaviorTree.CPP](https://www.behaviortree.dev/) v4 behaviour trees, which call ROS 2 actions, services and topics through BehaviorTree.ROS2, and generates their code and the node description for the [Groot2](https://www.behaviortree.dev/groot/) editor. It is the repository `rostooling-bt-plugin`, plugin id `de.fraunhofer.ipa.ros.plugin.bt`.

## Enabling the plugin

Build the JAR and enable it as described in [Loading and trust](PluginLoading.md):

```bash
mvn test package
```

The result is `target/rostooling-bt-plugin-3.6.0.jar`. A rebuilt JAR has a new fingerprint, so it has to be approved again.

## The model of a leaf

A leaf is a plugin in a package. Its base is `behaviortree_cpp.TreeNode`, which a catalogue model of the plugin declares with the loader `bt`, the loader that this plugin provides:

```text
behaviortree_cpp:
  pluginbases:
    TreeNode:
      class: "BT::TreeNode"
      header: "behaviortree_cpp/tree_node.h"
      loader: bt
```

Add the `catalogue/` folder of the plugin as a catalogue source, so that the reference `behaviortree_cpp.TreeNode` resolves.

The leaf declares the ROS 2 interface that it uses, and a `@bt` block that says what kind of leaf it is, which interface it wraps and which ports it has in the tree:

<!-- check: group=btleaves file=behaviortree_cpp.ros2 skip="needs the BehaviorTree plugin to be enabled and approved" -->
```text
behaviortree_cpp:
  pluginbases:
    TreeNode:
      class: "BT::TreeNode"
      header: "behaviortree_cpp/tree_node.h"
      loader: bt
```

<!-- check: group=btleaves file=bt_leaves.ros2 skip="needs the BehaviorTree plugin to be enabled and approved" -->
```text
bt_leaves:
  artifacts:
    leaves:
      plugins:
        LookupTarget:
          implements: behaviortree_cpp.TreeNode
          actionclients:
            lookup_tf: { type: "tf2_msgs/action/LookupTransform" }
          @bt:
            kind: action
            wraps: ac-> lookup_tf
            ports:
              target_point: { direction: in, type: "geometry_msgs/msg/Point", description: "Target 3D point" }
              frame_name: { direction: in, type: String, default: "base_link" }
              found: { direction: out, type: Boolean }
        ToggleGate:
          implements: behaviortree_cpp.TreeNode
          serviceclients:
            set_gate: { type: "std_srvs/srv/SetBool" }
          @bt:
            kind: action
            wraps: sc-> set_gate
            ports:
              enable: { direction: in, type: Boolean, default: "true" }
              ok: { direction: out, type: Boolean }
        WaitReady:
          implements: behaviortree_cpp.TreeNode
          subscribers:
            ready_topic: { type: "std_msgs/msg/Bool" }
          @bt:
            kind: condition
            wraps: sub-> ready_topic
            ports:
              require_true: { direction: in, type: Boolean, default: "true" }
              active: { direction: out, type: Boolean }
```

The entries of the `@bt` block:

| Entry | Meaning |
| --- | --- |
| `kind` | `action` for a leaf that does something and can run for some time, `condition` for a leaf that only checks something. |
| `wraps` | The interface of the leaf that it uses: `ac-> name` for an action client, `sc-> name` for a service client, `sub-> name` for a subscriber. |
| `ports` | The ports of the leaf in the tree. Each has a `direction` (`in`, `out` or `inout`), a `type` (`String`, `Boolean`, `Integer`, `Double`, or a message type), and optionally a `default` for input ports and a `description` for Groot2. |

A leaf on the `bt` loader that has no `@bt` block is reported in the notices of the generation, because nothing would register it.

## What is generated

For each leaf, RosTooling builds the library of the plugins, and the BehaviorTree plugin adds the code of the leaves and their registration:

```text
bt_leaves/
├── include/bt_leaves/
│   ├── LookupTargetPorts.hpp     ports and the data of the interface, rewritten every time
│   ├── LookupTargetBtNode.hpp    the leaf class, built on BehaviorTree.ROS2, rewritten every time
│   ├── LookupTargetLogic.hpp     your code, written once and never overwritten
│   └── …                         the same three files for each leaf
├── src/leaves_bt_plugin.cpp      registers the leaves with BehaviorTree.CPP, rewritten every time
└── plugins/de_fraunhofer_ipa_ros_plugin_bt/
    ├── bt_leaves_tree_nodes.xml  description of the leaves for Groot2
    └── bt_leaves_bt_signatures.txt
```

Your logic file is the only one that you edit. For an action leaf it has two methods: `set_goal` turns the input ports into the goal of the action, and `on_result` turns the result into the output ports and the outcome of the leaf, `SUCCESS` or `FAILURE`:

```cpp
class LookupTargetLogic {
public:
  bool set_goal(const Inputs& inputs, Goal& goal) {
    (void)inputs;
    (void)goal;
    return true;
  }

  Outcome on_result(const Result& result, Outputs& outputs) {
    (void)result;
    (void)outputs;
    return Outcome::SUCCESS;
  }
};
```

The ports header gives these methods typed inputs and outputs:

```cpp
namespace bt_golden_leaves::lookup_target {

enum class Outcome {
  SUCCESS,
  FAILURE
};

using ActionType = tf2_msgs::action::LookupTransform;
using Goal = ActionType::Goal;
using Result = ActionType::Result;

struct Inputs {
  geometry_msgs::msg::Point target_point{};
  std::string frame_name{};
};

struct Outputs {
  bool found{};
};

}  // namespace bt_golden_leaves::lookup_target
```

The file for Groot2 describes every leaf with its ports, so that the tree can be edited with them:

```xml
<root BTCPP_format="4">
  <TreeNodesModel>
    <Action ID="LookupTarget">
      <input_port name="action_name" default="" type="std::string">Action server name</input_port>
      <input_port name="target_point" type="geometry_msgs::msg::Point">Target 3D point</input_port>
      <input_port name="frame_name" default="base_link" type="std::string"/>
      <output_port name="found" type="bool"/>
    </Action>
    <Condition ID="WaitReady">
      <input_port name="topic_name" default="" type="std::string">Topic name</input_port>
      <input_port name="require_true" default="true" type="bool"/>
      <output_port name="active" type="bool"/>
    </Condition>
  </TreeNodesModel>
</root>
```

The plugin adds the port with the name of the action, service or topic to each leaf, so the tree can choose the server to call.

## Running the tree

The plugin ships a ready node, `bt_orchestrator`, that executes behaviour trees with `BT::TreeExecutionServer`. Its model in the catalogue has a plugin slot for the leaves, which holds the leaf libraries. A system chooses the leaves, and RosTooling sets the parameter `leaf_libraries` in the launch file:

<!-- check: group=btsystem file=bt_system.rossystem skip="needs the BehaviorTree plugin to be enabled and approved" -->
```text
bt_app_sys:
  nodes:
    orchestrator:
      from: "bt_orchestrator.bt_orchestrator"
      slots:
        bt_leaves: [bt_leaves.LookupTarget, bt_leaves.WaitReady]
```

## Tools

While the plugin is enabled, editors and AI agents have these tools in addition to the general ones:

| Tool | Purpose |
| --- | --- |
| `bt_add_node` | Adds a leaf with its `@bt` block to a package model. |
| `bt_add_port` | Adds or updates a port in the `@bt` block of a leaf. |
| `bt_set_wraps` | Sets the wrapped interface, and optionally the kind, of a leaf. |
| `bt_validate_tree` | Checks BehaviorTree.CPP XML trees against the leaves of a model: ports, directions, missing ports and subtrees. Nodes that are not in the model are warnings. |

## Requirements and distributions

BehaviorTree.ROS2 is not available as a binary package, so it has to be built from source in the workspace, at the version that the plugin pins. The generated and shipped code is built and its smoke tests are run in the official ROS images:

| Distribution | Status |
| --- | --- |
| Humble, Jazzy, Kilted, Lyrical | Supported |
| Rolling | Does not build: BehaviorTree.ROS2 uses a header that Rolling has removed. This is a problem of BehaviorTree.ROS2, not of the plugin. |
