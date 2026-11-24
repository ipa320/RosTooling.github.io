# Known issues

This page lists what does not work as you might expect. Entries marked *fixed* stay for a while so that people who read older material find them.

## Languages and validation

**Interfaces of other nodes can be instantiated.** When you add a node to a system, the tooling lets you reference an interface that the node does not declare in its own `.ros2` model. Such a reference is not an error, but it is wrong. Check that the interfaces of a node in a system are the ones its package model defines.

**Ports with the same name (fixed in 3.3.1).** Connections used to need unique interface names. Write the interface as `<node>::<interface>`, or `<subsystem>::<node>::<interface>`, to tell two ports of the same name apart. The short form still works where the name is unique. See [Systems](RosSystemModelDescription.md).

**Connections into subsystems (fixed in 3.5.0).** Subsystem interfaces are resolved by their qualified path, and the launch file remaps the topics of connections that cross a subsystem boundary.

## Code generation

**Generated nodes have no start-up hook.** A node that has to do something when it starts, for example create a timer, has to do it in the code of the generated wrappers that you own. There is no model element for it yet.

**Namespaced nodes and parameter files.** For a node with a namespace, the parameter file that the launch file loads can use the wrong key. A fix is in progress. Until it is released, check the generated `config` file of such a node.

**`ros2 launch` and nodes that read the keyboard.** A node such as `turtle_teleop_key` needs a terminal and does not work when started from a launch file. Start it with `ros2 run` in its own terminal.

**Features that need a newer RosTooling than the current release.** These features are described as working. The server of the current release rejects or ignores them, and the pages that describe them carry a note.

- `Base64` values in parameters, see [Parameters](ParametersAPI.md),
- naming another package model in `dependencies:`, see [Packages](RosModelDescription.md),
- arrays with a fixed size or an upper bound, see [Messages, services and actions](NewCommunicationObjects.md),
- the `default:` of a system parameter, see [Systems](RosSystemModelDescription.md),
- importing a node of the catalogue in the Studio, see [Catalogue](../vscode/catalogue.md).

If a model with one of these is rejected as invalid, your RosTooling is older than the feature. See [What's new](WhatsNew.md) for the versions.

## Plugins (Preview)

The plugin interface may still change before it is released, see [Plugin concepts](PluginConcepts.md). A JAR without plugin metadata is listed as `Cannot load: no plugin metadata` and is never loaded.

## Components

See [Messages, services and actions](NewCommunicationObjects.md) for the types that the generators support.

## VS Code

Problems with the extension, such as a missing Java, are in the [troubleshooting section](../vscode/reference.md) of the extension reference.

## Eclipse

See [Eclipse known issues](eclipse_issues.md).
