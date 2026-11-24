# MCP server and agent skills

The extension includes a server for the Model Context Protocol (MCP), `rostooling-mcp`, with which AI coding agents can read, write, check and generate RosTooling models. It uses the same language server and diagram server as VS Code, so an agent gets the same validation and the same edits as you, and it works without a running VS Code.

An agent does not edit the text of a model. It calls tools such as "add a node to this system" or "connect these two interfaces", the tools change the model through the model server and report the result and the diagnostics. The positions of the diagram are not part of the calls: the diagram is laid out automatically.

## Setting up an agent

The command **RosTooling: Install AI Agent Skills & MCP** sets up one or all of these agents: Claude Code, Antigravity, Codex CLI, Cursor and Gemini CLI. You choose the agent and whether the setup is for the current workspace or for all workspaces. It installs

- the **skills** of RosTooling, instructions for writing models, building systems and implementing nodes that the agent loads when it needs them,
- the **MCP configuration**, so that the agent starts `rostooling-mcp`,
- the **hooks** that check a model after the agent has written it, for the agents that support hooks.

The extension offers this when you open a ROS workspace. The setting `rostooling-languages.agentSkills.promptOnStartup` turns that question off. When the extension is updated, the paths in the configurations that it created are refreshed, and enabling or disabling a [plugin](plugins.md) updates the plugin options in them.

### Configuring by hand

Any agent that supports MCP can use the server. Add it to the configuration of the agent:

```json
{
  "mcpServers": {
    "rostooling": {
      "command": "node",
      "args": ["/path/to/RosTooling_Extension/rostooling-languages/bin/rostooling-mcp.js"]
    }
  }
}
```

If `rostooling-mcp` is installed with npm, `"command": "rostooling-mcp"` is enough. The server finds the language server JAR, Java and the catalogue itself. These options and environment variables change that:

| Option or variable | Meaning |
| --- | --- |
| `--plugins-dir <path>`, `ROS_PLUGINS_DIR` | Plugin JARs, or folders with them. |
| `--enable-plugins <ids>`, `ROS_PLUGINS_ENABLED` | The ids of the plugins to enable, separated by commas. |
| `ROSMODEL_JAR` | The language server JAR to use. |
| `ROSMODEL_JAVA` | The Java executable to use. |
| `ROSMODEL_CATALOGUE_DIR` | The catalogue folder. |

Plugins are subject to the same [approval](../docu/PluginLoading.md#approval) as in VS Code.

## The tools

**Checking, generating and building**

| Tool | Purpose |
| --- | --- |
| `validate_ros_model` | Validates a `.ros`, `.ros2` or `.rossystem` file and returns the diagnostics with their lines. |
| `generate_ros2_wrappers` | [Generates the node code](../docu/Ros2CodeGeneration.md) of a `.ros2` model, in C++, Python or both, for a chosen distribution. |
| `generate_rossystem_code` | [Generates the launch package](../docu/CodeGeneration.md) of a `.rossystem` model. |
| `list_ros2_distros` | Lists the supported [distributions](../docu/TargetDistributions.md), the default, and what `auto` means now. |
| `build_ros2_package` | Builds generated packages with `colcon build`. |

**System models (`.rossystem`)**

| Tool | Purpose |
| --- | --- |
| `ros_add_node` | Adds a node, referring to a `.ros2` model, and returns its interfaces. Creates the file if needed. |
| `ros_add_subsystem` | Adds a subsystem. |
| `ros_set_system_property` | Sets the name or `fromFile` of the system. |
| `ros_connect_interfaces` | Connects two interfaces, named `<node>.<interface>`. The order does not matter. |
| `ros_auto_connect` | Proposes connections, and adds the chosen ones, see [Auto-connect](auto-connect.md). |
| `ros_set_parameter` | Sets a parameter of a node, typed by its declaration. |
| `ros_add_process` | Groups nodes into a process. |
| `ros_attach_plugins` | Sets the plugins of a slot of a node. |
| `ros_remove_element` | Removes a node, process, subsystem or connection. |
| `export_system_diagram` | Renders the diagram as an SVG file and returns a summary of the system with its diagnostics. |

**Package models (`.ros2`) and types (`.ros`)**

| Tool | Purpose |
| --- | --- |
| `ros2_add_artifact` | Adds an artifact with a node, or a library without a node. |
| `ros2_add_interface` | Adds a publisher, subscriber, service or action interface to a node. |
| `ros2_add_parameter` | Adds a parameter. |
| `ros2_set_qos` | Sets or clears the quality of service of an interface. |
| `ros2_set_property` | Changes a property of a package, plugin base, artifact, plugin or plugin slot. |
| `ros2_add_plugin_base`, `ros2_add_plugin`, `ros2_add_plugin_slot` | Add a [plugin base, a plugin and a slot](../docu/PluginConcepts.md). |
| `ros2_remove_element` | Removes an artifact, interface, parameter or type. |
| `ros_add_spec` | Adds a message, service or action type to a `.ros` file. |

**Plugins**

| Tool | Purpose |
| --- | --- |
| `ros_list_plugins` | Lists the active plugins with their blocks, loaders and tools. |
| `ros_get_extension_block`, `ros_set_extension_block` | Read, write or remove an [`@` block](../docu/PluginBlocks.md) with its schema and validation. |
| `generate_plugin_outputs` | Runs the generation of the core and of the active plugins. |

An enabled plugin adds tools of its own, named with the prefix of the plugin, and a typed tool for each of its blocks. The documentation of a plugin is offered as a resource of the server.
