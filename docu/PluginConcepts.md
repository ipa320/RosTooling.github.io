# Plugins: concepts

:::{note}
**Preview.** The plugin support described in this section is not released yet and may still change.
:::

The word *plugin* has two meanings in RosTooling, and this section covers both.

| | What it is | Where it is described |
| --- | --- | --- |
| **A plugin in a model** | A class that implements an interface of another node, and is loaded by that node when it starts. This is the *pluginlib* pattern of ROS 2, as used by the Nav2 planners and controllers. You describe it in a `.ros2` file and a `.rossystem` file. | This page |
| **A RosTooling plugin** | A Java extension of RosTooling itself. It adds blocks to the model language, checks them, generates code for them and offers tools to editors and AI agents. | [Extension blocks](PluginBlocks.md), [Loading and trust](PluginLoading.md), [Writing a plugin](PluginDevelopment.md) |

## The parts of a plugin model

| Term | Meaning | Keyword |
| --- | --- | --- |
| **Plugin base** | The interface that plugins implement: a C++ class, its header and the way plugins of this base are loaded. | `pluginbases:` in a package |
| **Plugin** | A class that implements a base. | `plugins:` in an artifact, with `implements:` |
| **Slot** | A place in a node where plugins are plugged in: which base it takes, and the parameter that names the plugins. | `pluginslots:` in a node |
| **Attachment** | The choice of the plugins for one slot of one node in one system. | `slots:` in a system node |

The model separates the roles: the package of the base says what a plugin is, the package of the host node says where plugins go, the package of the plugins provides them, and the system chooses which ones to use.

## An example

The package `demo_core` defines two plugin bases, a planner and a critic.

<!-- check: group=demo file=demo_core.ros2 -->
```text
demo_core:
  pluginbases:
    Planner:
      class: "demo_core::Planner"
      header: "demo_core/planner.hpp"
      loader: pluginlib
    Critic:
      class: "demo_core::Critic"
      header: "demo_core/critic.hpp"
      loader: pluginlib
```

RosTooling does not generate the base itself. The header is written by hand, or comes from another package. The model only states the facts that the other parts need.

The node `planner_server` has two slots. The slot `planner` takes one planner, and uses `StraightLine` when no system chooses another. The slot `critics` takes any number of critics.

<!-- check: group=demo file=demo_planner_server.ros2 -->
```text
demo_planner_server:
  artifacts:
    planner_server:
      node: planner_server
      pluginslots:
        planner:
          base: demo_core.Planner
          parameter: planner_plugin
          default: demo_planners.StraightLine
        critics:
          base: demo_core.Critic
          parameter: critic_plugins
          multiple: true
```

The package `demo_planners` provides plugins. An artifact with `plugins:` holds plugins instead of a node, and is built as a shared library.

<!-- check: group=demo file=demo_planners.ros2 -->
```text
demo_planners:
  artifacts:
    planners:
      plugins:
        StraightLine:
          implements: demo_core.Planner
        Smooth:
          implements: demo_core.Planner
    critics:
      plugins:
        AlignCritic:
          implements: demo_core.Critic
        GoalCritic:
          implements: demo_core.Critic
```

A system chooses the plugins for the slots of its node:

<!-- check: group=demo file=demo_system.rossystem -->
```text
demo_system:
  nodes:
    planner_server:
      from: "demo_planner_server.planner_server"
      slots:
        planner: [demo_planners.Smooth]
        critics: [demo_planners.AlignCritic, demo_planners.GoalCritic]
```

References between packages are written `<package>.<name>`, for example `demo_core.Planner` and `demo_planners.Smooth`.

## Slots

A slot has these entries:

| Entry | Meaning |
| --- | --- |
| `base:` | The plugin base that its plugins must implement. Required. |
| `parameter:` | The name of the node parameter that holds the chosen plugins. Required. The slot declares this parameter, so do not declare it again under `parameters:`. |
| `multiple:` | `true` lets any number of plugins attach, and makes the parameter a list of strings. Without it a slot takes at most one plugin. |
| `format:` | What the parameter holds. `class` (the default) names the plugin classes, for example `demo_planners::Smooth`. `library` names the shared libraries that contain them. |
| `default:` | The plugin that applies when no system attaches one. It must implement the base of the slot. |

## What is generated

**For the plugin package** (`demo_planners`), per plugin: a class for your code that is generated once and never overwritten, a wrapper for the ROS 2 side, and the registration with `PLUGINLIB_EXPORT_CLASS`. For the artifact: a shared library and the description file `plugins.xml` that pluginlib reads:

```xml
<class_libraries>
<library path="planners">
<class name="demo_planners::StraightLine" type="demo_planners::StraightLine" base_class_type="demo_core::Planner">
  <description>ROS 2 pluginlib plugin 'StraightLine'</description>
</class>
…
```

The class of a plugin is created like this. You implement the methods of the base in it:

```cpp
class Smooth : public demo_core::Planner {
public:
    Smooth() = default;
    ~Smooth() override = default;

    // Implement pure virtual methods of demo_core::Planner here and use ros_wrapper_.

protected:
    SmoothRosWrapper ros_wrapper_;
};
```

**For the host node** (`planner_server`): the wrapper declares one parameter for each slot, `planner_plugin` and `critic_plugins`, and loads the plugins with a pluginlib class loader when the node starts. The loaded plugins are available to your logic as `get_planner_plugin()` and `get_critics_plugins()`. The parameters are read-only: the plugins are chosen at startup.

**For the system** (`demo_system`): the launch file sets the slot parameters to the attached plugins:

```python
  planner_server = Node(
    package="demo_planner_server",
    executable="planner_server",
    output='screen',
    name="planner_server",
    parameters=[{
          "planner_plugin": ParameterValue("demo_planners::Smooth", value_type=str),
          "critic_plugins": ["demo_planners::AlignCritic", "demo_planners::GoalCritic"],}]
  )
```

Started without the system, the host node loads the `default:` of its slot.

## Loaders

The `loader:` of a plugin base says how the plugins of the base are found and registered. `pluginlib` is built into RosTooling. Other loaders are provided by a RosTooling plugin, which then generates the registration code. The BehaviorTree plugin provides the loader `bt` for BehaviorTree.CPP leaf nodes, see [the BehaviorTree example](PluginBtExample.md). A base that names a loader that no enabled plugin provides is reported as an error.

## Checks

The validator checks the plugin parts: the class of a base, the loader, duplicate slots, slot parameters that are declared twice, defaults that do not implement the base, and the attachments of a system. The list is on the [validation page](Validation.md#plugins).
