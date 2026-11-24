# Plugins in VS Code

:::{note}
**Preview.** The plugin support is not released yet and may still change.
:::

RosTooling can be extended with plugins: Java libraries that add blocks, checks and code generators to the models. The pages in [Plugins](../docu/PluginConcepts.md) explain what plugins are and how they work, and [Loading and trust](../docu/PluginLoading.md) explains how a plugin is found, approved and loaded. This page covers how you use them in VS Code.

## The RosTooling Plugins view

In a workspace with RosTooling files, the Explorer has a view named **RosTooling Plugins**. It lists the plugin JARs that the extension found, with the id, the version and the summary of each, and its state: enabled, disabled, waiting for approval, or with a load error. The toolbar of the view has **Refresh Plugins** and **Manage Plugins…**. The context menu and the buttons of a plugin have these actions:

| Action | Command | What it does |
| --- | --- | --- |
| Enable | `rostooling.enablePlugin` | Enables the plugin. If its JAR has not been approved yet, or has changed, you are asked to approve it first. |
| Disable | `rostooling.disablePlugin` | Disables it. |
| Approve JAR | `rostooling.approvePluginJar` | Shows the file name and the SHA-256 fingerprint of the JAR and records your approval. |
| Show Load Error | `rostooling.showPluginLoadError` | Shows why a plugin could not be loaded. |
| Open Documentation | `rostooling.openPluginDocs` | Shows the documentation that the plugin contains, in the Markdown preview. |

## Manage Plugins

**RosTooling: Manage Plugins…** (`rostooling.managePlugins`) in the Command Palette does the same with a picker. You can enable or disable the compatible plugins, add a plugin JAR or a folder with plugin JARs, and open the settings. When you enable a plugin, you choose whether it is saved in the settings of the workspace, which can be shared with your team, or in your user settings. The extension then restarts the language server and the diagram server, and updates the configuration of the AI agents that you let it manage.

## Settings

| Setting | Meaning |
| --- | --- |
| `rostooling-languages.plugins.paths` | The JAR files and the folders with plugin JARs. `${workspaceFolder}` and paths relative to the workspace can be used. |
| `rostooling-languages.plugins.enabled` | The ids of the enabled plugins. A plugin that is found but not listed here stays disabled. |

The settings of a workspace that you have not trusted in VS Code are ignored.

## What a plugin changes in the editor

- In the text editor, the blocks of a plugin, `@<name>:`, are highlighted, completed and checked.
- In the diagram, an element that has a block of a plugin, or a plugin base that uses a loader of a plugin, shows a badge with the name of the plugin. The inspector lists the blocks with their fields, and offers the blocks that the active plugins provide for that kind of element.
- In the MCP server, the tools of the plugin are added, see [MCP server and agent skills](mcp-and-agents.md).
- In the generated package, the files of the plugin appear in the folders that are named after it, and the notices of the plugin are shown as messages.

See [Extension blocks](../docu/PluginBlocks.md) for the syntax of a block.
