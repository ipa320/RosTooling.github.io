# Commands, settings and troubleshooting

## Commands

All commands are in the Command Palette (`Ctrl+Shift+P`). Some are also in the context menu of a file or in a view.

| Command | Title | Where | What it does |
| --- | --- | --- | --- |
| `rostooling.openVisualStudio` | RosTooling: Open in RosTooling Visual Studio | file menus, editor title bar | Opens the file in [RosTooling Studio](studio.md). |
| `rossystem.triggerCodeGeneration` | ROSSYSTEM: Generate ROS2 Package | `.rossystem` editor, Studio | [Generates the package](generate-and-build.md#generate-the-package-of-a-system) of a system. |
| `ros2.generateWrappers` | ROS2: Generate ROS 2 Node Wrappers | `.ros2` editor and Explorer | [Generates the node code](generate-and-build.md#generate-the-nodes-of-a-package) of a package. |
| `rossdl.buildPackage` | ROSSDL: Build ROS 2 Package | Command Palette | [Builds](generate-and-build.md#build) with `colcon`. |
| `rostooling.installAgentSkills` | RosTooling: Install AI Agent Skills & MCP | Command Palette | Sets up [agents](mcp-and-agents.md#setting-up-an-agent). |
| `rostooling.selectServerJar` | RosTooling: Select Language Server JAR… | Command Palette | Chooses the language server JAR: the bundled one or your own. |
| `rostooling.managePlugins` | RosTooling: Manage Plugins… | Command Palette, Plugins view | Enables and disables [plugins](plugins.md). |
| `rostooling.refreshPluginsView` | RosTooling: Refresh Plugins | Plugins view | Lists the plugins again. |
| `rostooling.enablePlugin`, `rostooling.disablePlugin` | RosTooling: Enable Plugin, Disable Plugin | Plugins view | Enable or disable one plugin. |
| `rostooling.approvePluginJar` | RosTooling: Approve JAR | Plugins view | Approves the JAR of a plugin. |
| `rostooling.showPluginLoadError` | RosTooling: Show Load Error | Plugins view | Shows why a plugin did not load. |
| `rostooling.openPluginDocs` | RosTooling: Open Documentation | Plugins view | Shows the documentation of a plugin. |

## Settings

| Setting | Default | Meaning |
| --- | --- | --- |
| `rostooling-languages.java.home` | empty | The folder of the JDK to use. Empty uses the Java of the system. |
| `rostooling-languages.server.jarPath` | empty | A language server JAR to use instead of the bundled one, for example a build of your own. |
| `rostooling-languages.server.vmargs` | `-XX:TieredStopAtLevel=1`, `-Xms256m`, `-Xmx1024m` | Java options of the language server, for example for more memory. |
| `rostooling-languages.server.trace` | `off` | How much of the communication with the language server is logged: `off`, `messages` or `verbose`. |
| `rostooling-languages.agentSkills.promptOnStartup` | `true` | Offers to set up AI agents when a ROS workspace is opened. |
| `rostooling-languages.plugins.paths` | empty | Plugin JARs and folders with them. See [Plugins](plugins.md). |
| `rostooling-languages.plugins.enabled` | empty | The ids of the enabled plugins. |

The earlier setting `rostooling-languages.plugins.trustedChecksums` is deprecated: its entries are moved to the approval file `~/.rostooling/trusted-plugins.json`.

## Where to look

The extension writes what it does to the output channel **ROS LSP** (View → Output, then choose the channel). It shows the Java that is used, the catalogue folder, the start of the language server, the progress of generation and the output of the build. With `rostooling-languages.server.trace` set to `messages` or `verbose` it also shows the messages that the editor and the server exchange.

## Troubleshooting

**The extension says Java is missing or too old.** The language server needs Java 21 or later. The check of the extension only tests for a lower version, but the server itself does not start with an older one. Install a JDK 21 and set `rostooling-languages.java.home` to its folder, or make it the Java of your system, then reload the window.

**There are no diagnostics, completion or hover.** The language server is not running. Open the **ROS LSP** output channel to see why, for example a wrong Java. A custom JAR in `rostooling-languages.server.jarPath` that does not exist or is not a RosTooling language server causes the same. Clear the setting, or choose the bundled JAR with **RosTooling: Select Language Server JAR…**.

**Types such as `std_msgs/msg/String` are not found.** The catalogue is missing. The first download needs access to GitHub. Click **Sync Catalogue** in the Studio, see [Catalogue](catalogue.md), and check the output channel for errors.

**The Studio shows "Waiting for the GLSP server", or the diagram does not match the file.** Click the reload button on the canvas. If it persists, the server is not running, see above.

**Generation says the file is not inside a workspace folder.** The generated code goes to `src-gen/` of the workspace folder, so the model must be in a folder that you opened in VS Code. Open the folder with **File → Open Folder**.

**Generation writes nothing.** Look at the diagnostics: errors in the model stop the generation. Fix them and generate again, see [Validation and diagnostics](../docu/Validation.md).

**A plugin is not loaded.** Select it in the **RosTooling Plugins** view and choose **Show Load Error**. The reasons are a JAR that has not been approved or has changed since you approved it, a JAR without plugin metadata, or a version that does not fit, see [Loading and trust](../docu/PluginLoading.md). In a workspace that you have not trusted, the plugin settings of the workspace are ignored.

**The build command does not accept a folder.** The ROSSDL workspace must contain `src` and `install/setup.bash`: it must have been built before. The build workspace can be any ROS 2 workspace.
