# LSP commands

Besides the standard language features, the RosTooling language server answers a set of workspace commands. The VS Code extension and the MCP server use them, and you can call them from any LSP client with `workspace/executeCommand`.

Generation validates the model first and refuses to generate when it has errors. The answer then contains `error` with the diagnostics.

## Generation

| Command | Purpose |
| --- | --- |
| `ros2.generateWrappersServer` | Generates the C++ or Python wrapper of the nodes in a `.ros2` file. |
| `ros2.generateCppWrapper`, `ros2.generatePythonWrapper` | The same, fixed to one language. |
| `ros2.listDistros` | Lists the supported ROS 2 distributions and the default one. |
| `rossystem.generateCode` | Generates the launch files and package of a `.rossystem` file. |

Arguments of `ros2.generateWrappersServer`, in this order:

| Argument | Meaning |
| --- | --- |
| file URI | The `.ros2` file. |
| target nodes | List of node names, empty for all. |
| language | `cpp`, `python` or `both`. |
| distribution | A name such as `jazzy`, or `auto`. `auto` takes `$ROS_DISTRO`, else the newest LTS. |
| existing files | Map of paths to the current content of files already generated, so that your own code is never overwritten. |
| output root | Folder the files are written to, for example `src-gen/my_pkg`. Plugins use it to read their earlier output. |

`rossystem.generateCode` takes the file URI, the existing files, the distribution and the output root.

The result is the same for all generation commands:

```json
{
  "success": true,
  "files": { "my_pkg/CMakeLists.txt": "...", "my_pkg/src/my_node.cpp": "..." },
  "prunedFiles": [],
  "warnings": [],
  "notices": []
}
```

`files` maps a path inside the output folder to its content. The command does not write anything: the client does. `prunedFiles` are plugin files that an earlier run produced and that are no longer wanted. `notices` carry informational messages of plugins.

## Plugins

| Command | Purpose |
| --- | --- |
| `rostooling.listDiscoveredPlugins` | Lists the plugin JARs found in the configured folders by reading their metadata only. Nothing is loaded. |
| `rostooling.getPluginDocumentation` | Returns the Markdown documentation of an active plugin. |
| `rostooling.listPluginTools` | Lists the typed tools that the active plugins offer, derived from their block schemas. |
| `rostooling.executePluginTool` | Runs one of these tools. |
| `rostooling.getExtensionBlock`, `rostooling.setExtensionBlock` | Read and write an `@` block of a model element. |
| `rostooling.generatePluginOutputs` | Runs only the plugins, for a package or a system. |

See [Plugin concepts](PluginConcepts.md) and [Loading and trusting plugins](PluginLoading.md) for what these do.

## Starting the server

The server is the JAR that the extension ships in its `server` folder, started with `java -jar` and talking LSP on standard input and output. Plugins are configured with `--plugins-dir <folder>` and `--enable-plugins <id,id>`. The extension sets these from its [settings](../vscode/reference.md). The GLSP server is started from the same JAR, see [Diagram server](GlspServer.md).
