# RosTooling Studio

RosTooling Studio is the diagram editor of the extension. It opens a RosTooling file as a graph or a set of cards, and every change you make in it is written to the text file. The model server that checks and lays out the diagram is the same that the text editor uses.

## Opening

Right-click a `.rossystem`, `.ros2`, `.ros1` or `.ros` file and choose **Open in RosTooling Visual Studio**, use the graph icon in the title bar of the editor, or run **RosTooling: Open in RosTooling Visual Studio**. A file opens as text by default; the Studio is an additional way to look at it. **Open Code** in the toolbar returns to the text.

There are three views, one for each kind of file.

| File | View | Shows |
| --- | --- | --- |
| `.rossystem` | System | The nodes of a system as boxes with their interfaces as ports, the connections as wires, subsystems and processes. |
| `.ros2`, `.ros1` | Component | The nodes of a package as cards, with their interfaces, parameters, quality of service, plugin bases, slots and plugins. |
| `.ros` | Communication objects | The messages, services and actions of a package with their fields. |

## The toolbar

The toolbar changes with the view. In every view:

| Control | What it does |
| --- | --- |
| model name | Shows the name of the system or package. Edit it to rename the model. |
| Undo, Redo | Undo and redo changes made in the Studio (`Ctrl+Z`, `Ctrl+Shift+Z`). |
| Diagnostics pill | Shows **Valid**, or the number of errors and warnings of the model. Click it to see the list. |
| Catalogue | Opens the [catalogue](catalogue.md) of nodes, subsystems and types. |
| Sync Catalogue | Updates the catalogue from GitHub. |
| Open Code | Opens the text of the file. |

In the System view:

| Control | What it does |
| --- | --- |
| + Import Node | Adds a node from the catalogue or the workspace. |
| + Import Subsystem | Adds another system as a subsystem. |
| + Add Process | Adds a process, a container in which nodes run together. |
| interface filter | Shows only the ports and wires of one kind: topics, services or actions. |
| Generate ROS 2 Package | [Generates the package](generate-and-build.md) of the system. |

On the canvas of the System view there are also **Fit view**, **Connect**, **Auto-connect**, **Export SVG** and a reload button.

## The System view

**Layout.** The diagram is laid out automatically. Nodes are arranged in layers and the wires are routed in right angles around the nodes. You can drag a node to another place. A node that you moved keeps its place, and the places are stored in a file next to the model, `<model file>.layout`. Wires that cross show a small jump.

**Connecting.** Click **Connect**, then a source port and a target port, or drag from one port to another. The Studio only accepts a publisher with a subscriber, a service server with a service client, and an action server with an action client, with the same type. To change a connection, select the wire and drag one of its end handles onto another port; **Delete connection** removes it. To let the Studio propose connections, use [Auto-connect](auto-connect.md).

**Subsystems.** A subsystem is drawn as a frame. Collapse it to see only its boundary ports, expand it to see its nodes. You can wire to the interfaces of the nodes inside a subsystem. The Studio writes such a connection with the qualified name, `<subsystem>::<node>::<interface>`. When a connection needs an interface that the subsystem has not exposed yet, the Studio adds it to the subsystem's model.

**Processes.** A process is a frame that contains the nodes that run in one container, with the number of executor threads. Select it to rename it, change the threads and choose its member nodes. See [System generation](../docu/CodeGeneration.md#processes).

**The inspector.** Selecting an element shows its properties in the side panel, and the entries can be edited:

| Selected | You can |
| --- | --- |
| nothing | add, edit and remove the `@` blocks of the system that [plugins](plugins.md) provide |
| a node | change its name and namespace, see the component it comes from and open its model, rename (expose) its interfaces, set parameter values, see the plugins attached to each slot, edit its `@` blocks |
| a connection | see its kind and both ends, reconnect it, delete it |
| a process | change its name and threads, choose its member nodes |
| a subsystem | collapse or expand it, open its model, remove it |

The name of the system is changed in the toolbar. A node inside a subsystem is read-only: open the model of the subsystem to change it. Parameter values are checked against the declared type when you enter them.

**Exporting.** **Export SVG** saves the laid-out diagram as an SVG file.

## The Component view

The cards show the nodes of a package. **+ Add New Node** adds a node, and a node can be duplicated or deleted. A lifecycle node has an **LC** badge. For every interface the card shows its name and type, and the quality of service is edited with menus for reliability, durability, history, depth, liveliness, lease duration and deadline, see [Package language](../docu/RosModelDescription.md#quality-of-service). Parameters are listed with their types and defaults.

For [plugins](../docu/PluginConcepts.md) the view shows the plugin bases of the package, the slots of each node (with the base, the parameter, the format, whether several plugins can attach and the default), and library artifacts with their plugins. A node, a base or a plugin that comes with `@` blocks shows them in the inspector.

## The Communication objects view

**+ Message**, **+ Service** and **+ Action** add a type. Each type is a card with its fields; a field has a name and a type, and can be an array. The view shows which other types a field refers to, and a filter limits the list to one kind of type.

## Read-only models

A model that comes from the catalogue is shown with a banner and cannot be edited, so that you do not change the shared library by accident. Copy it into your workspace to change it.

## Keeping text and diagram in sync

- Changes in the diagram are written to the file as small edits, so the comments and the formatting of the rest of the file stay as they are.
- Edits in the diagram are saved shortly after you stop typing in a field, and before undo, redo or switching away.
- A change in the text, or by another tool, is picked up and shown in the diagram. The same is true for the models that the open file refers to: saving a `.ros2`, `.ros` or `.rossystem` file updates the diagrams that use it.
- When a subsystem's model is saved, the parent systems that refer to it are updated, so that remapped connections do not break.
