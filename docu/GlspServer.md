# Diagram server (GLSP)

RosTooling Studio does not parse or lay out models itself. A Java server, built on the [Eclipse GLSP](https://eclipse.dev/glsp/) framework, loads the model with the Xtext languages, builds the graph, lays it out and applies every edit to the model. The webview only draws what the server sends. This page is for people who build their own client or want to know what happens behind the Studio.

## Starting the server

The server is part of the same JAR as the language server. Run it standalone with:

```bash
java -jar <server JAR> --glsp --host 127.0.0.1 --port 5008
```

The default port is 5008. The VS Code extension starts it for you and connects over JSON-RPC.

A client sends `initialize` with the application id `rostooling-vscode-glsp`, protocol version `1.0.0` and the path of the catalogue as `args.cataloguePath`. Then it opens a diagram by sending a `process` notification with the standard GLSP actions.

## What the server builds

The diagram of a model is a graph of these element types:

| Type | Shows |
| --- | --- |
| `node:ros-node` | A node of a system, or the node of a package. |
| `node:subsystem` | A subsystem, collapsible. Collapsed, it shows its boundary ports. |
| `node:process` | A process that groups nodes. |
| `node:ros-package` | A package in the component view. |
| `port:pub`, `port:sub` | Publisher and subscriber. |
| `port:ss`, `port:sc` | Service server and client. |
| `port:as`, `port:ac` | Action server and client. |
| `edge:ros-connection` | A connection between two ports, routed orthogonally. |

Positions come from an automatic layered layout. Positions that you set by hand are pinned and stored in the `.layout` file next to the model.

## Actions and operations

Next to the standard GLSP operations (create, delete, reconnect, change bounds) the server defines these.

| Name | Kind | Purpose |
| --- | --- | --- |
| `requestRosIndex` / `setRosIndex` | action | The data the sidebars show: nodes, interfaces, parameters. |
| `requestRosTextEdits` / `setRosTextEdits` | action | The text edits that the last operation caused, so the client can apply them as one undoable edit. |
| `requestRosAutoConnect` / `setRosAutoConnect` | action | The candidate connections of the [auto-connect](../vscode/auto-connect.md) dialog. |
| `applyRosAutoConnect` | operation | Applies the selected candidates in one step. |
| `editRosProperty` | operation | Changes a property, a parameter value or a remap. |
| `toggleSubSystem` | operation | Collapses or expands a subsystem. |

## Related

The same server answers the MCP tools of the extension, so an agent edits the same model through the same code as a person does in the Studio. See [Architecture overview](Architecture.md) and [LSP commands](LspCommands.md).
