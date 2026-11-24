# Editing models

Every RosTooling file opens as text in a normal VS Code editor. The extension adds the language features below. They come from a language server that runs in the background, the same one that checks the models in the diagram, in code generation and for AI agents, so all of them show the same result.

## Languages and files

| File | Language | Describes |
| --- | --- | --- |
| `.ros2` | ROS 2 package | nodes, interfaces, parameters, plugins of a ROS 2 package. See [Package language](../docu/RosModelDescription.md). |
| `.ros1` | ROS 1 package | the same for ROS 1 packages. |
| `.ros` | communication objects | message, service and action types. See [Communication objects](../docu/NewCommunicationObjects.md). |
| `.rossystem` | system | nodes that run together and their connections. See [System language](../docu/RosSystemModelDescription.md). |

## Features

- **Highlighting.** Keywords, names, types, values and `@` blocks have their own colours, and brackets are matched.
- **Completion.** `Ctrl+Space` offers what fits at the cursor: keywords, packages, artifacts, interfaces, parameters, and the message, service and action types of the workspace and the [catalogue](catalogue.md). In an `@` block of a [plugin](plugins.md) it offers the entries of the block and the choices that the plugin suggests.
- **Validation.** Errors and warnings are underlined while you type and listed in the Problems view. Hover over an underline for the message. [Validation and diagnostics](../docu/Validation.md) lists what is checked.
- **Hover text.** Hovering over an element shows information about it. For `@` blocks it shows the help text of the plugin that provides them.
- **Navigation.** `Ctrl` + click on a reference, such as an interface in a connection of a system, goes to its definition, also across files.

A reference to an interface of a node in a subsystem is written `<subsystem>::<node>::<interface>`, and the editor resolves it in the same way.

## Types that are not found

When a `type:` refers to a message that RosTooling does not know, the message of the diagnostic suggests two fixes: to add the project of the specifications, or to generate the `.ros` model for the package. See [Catalogue](catalogue.md) for where the types come from.

## Opening a file in the diagram

A file opens as text. To see the same file as a diagram, right-click it and choose **Open in RosTooling Visual Studio**, or use the graph icon in the editor title bar. Every change in the diagram is written to the text, and a change that you make in the text appears in the diagram, so you can switch between the two at any time. See [RosTooling Studio](studio.md).

## Server and Java

The language server needs Java 21 or later. If the extension cannot find it, it shows an error with a **Download Java** action. Use the setting `rostooling-languages.java.home` to point the extension to a JDK, `rostooling-languages.server.trace` to see the messages between the editor and the server, and the command **RosTooling: Select Language Server JAR…** to use another server build. [Commands, settings and troubleshooting](reference.md) has the details.
