# What's new in 2026

The core (RosTooling) and the VS Code extension have separate versions. Each extension release bundles one core release, so you only install the extension. The full lists are the `CHANGELOG.md` files of the two repositories.

## Overview

| Date | Core | Extension | Highlights |
| --- | --- | --- | --- |
| 2026-11 (next) | 3.6.0 | Unreleased | [Plugins](PluginConcepts.md) (Preview): `@` blocks, plugin generation, trust store |
| 2026-11-16 | 3.5.0 | 2.5.0 | Target distributions, auto-connect |
| 2026-11-13 | 3.4.0 | 2.4.0 | GLSP diagram server, MCP authoring tools |
| 2026-11-03 | 3.3.x | 2.3.0 | Qualified interface names, launch substitutions, validation before generation |
| 2026-09-16 | 3.2.x | 2.0.0 | RosTooling Studio in VS Code |

## Core 3.2 to 3.6

**Languages.** Connections in a `.rossystem` can name ports as `node::interface` and `subsystem::node::interface`, which also fixes two ports of the same name (3.3.1). Parameter values can be launch substitutions such as `$(find-pkg-share ...)` and `$(var ...)` (3.3.2), and the validator warns about a variable that is not declared (3.3.3). A package lists the `dependencies` it needs and they end up in `package.xml` (3.3.3).

**Generation.** Generation validates the model first and refuses to emit files for a model with errors (3.3.3). The new [target distribution](TargetDistributions.md) registry knows Humble, Jazzy, Kilted, Lyrical and Rolling, and the generated code follows what each one needs (3.5.0). Lifecycle nodes launch correctly on every distribution (3.5.0).

**Diagram server.** A GLSP server builds, lays out and edits the diagram of every model (3.4.0), see [Diagram server](GlspServer.md). Auto-connect proposes connections between the nodes of a system (3.5.0).

**Plugins.** A plugin interface lets JARs add `@` blocks to the models and files to the generated packages (3.6.0), see [Plugin concepts](PluginConcepts.md).

## Extension 2.0 to 2.5

- **2.0**: RosTooling Studio, the diagram editor for all four file types, see [RosTooling Studio](../vscode/studio.md).
- **2.3**: A new webview for the Studio, with the catalogue drawer and qualified interface names.
- **2.4**: The diagrams are built by the Java GLSP server, and the MCP server gets coordinate-free tools to author systems, packages and types, see [MCP server and agent skills](../vscode/mcp-and-agents.md).
- **2.5**: The auto-connect dialog and its MCP tool, and the choice of the ROS 2 distribution.
- **Next**: plugin management in VS Code, see [Plugins in VS Code](../vscode/plugins.md).

## Documentation

This site is built with Sphinx. It was rewritten for VS Code, and the Eclipse pages moved into legacy sections. The rossdl pages are archived because RosTooling has its own generators.

## Things that change when you update

- **The default distribution.** Without `$ROS_DISTRO`, generation targets the newest LTS distribution. It was Humble before 3.5.0, was Jazzy in 3.5.0 and is Lyrical now. Set `ROS_DISTRO` or pass the distribution to keep another one.
- **`ament_target_dependencies` is gone.** Generated build files link with `target_link_libraries(... ${pkg_TARGETS})`, because Lyrical and Rolling removed the old macro. Do not edit generated build files to bring it back.
- **Newer CMake minimums.** Packages for Kilted and later require CMake 3.20.
- **The plugin syntax changed in the Preview.** Plugin bases, libraries and slots use the syntax of [Plugin blocks](PluginBlocks.md). Models written for an earlier preview have to be adapted.
- **Models.** `.ros`, `.ros1`, `.ros2` and `.rossystem` files of earlier releases still parse.
