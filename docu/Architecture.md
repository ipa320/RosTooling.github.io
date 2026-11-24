# Architecture overview

RosTooling is a Java core with several thin clients on top. All the knowledge about the languages, the validation and the generators lives in the core, so every client shows the same diagnostics and produces the same code.

```{image} images/architecture.svg
:alt: The RosTooling core with its clients, plugins, model files and generated packages
:width: 100%
```

## The parts

**Languages.** Four Xtext languages describe a ROS project: `.ros` for message, service and action types, `.ros1` and `.ros2` for the nodes of a package, and `.rossystem` for the composition of nodes into a system. See [Packages](RosModelDescription.md) and [Systems](RosSystemModelDescription.md). The validators of these languages are the single source of the diagnostics, see [Validation](Validation.md).

**Generators.** The generators turn a validated model into files: C++ or Python wrappers for a node, and launch files and package files for a system. They take the target distribution into account, see [Code generation from a package model](Ros2CodeGeneration.md) and [Target distributions](TargetDistributions.md).

**Language server.** The core runs as a language server (LSP) over standard input and output. Besides the usual completion, hover and diagnostics it offers commands for generation and for plugins, see [LSP commands](LspCommands.md).

**Diagram server.** The same JAR can run as a GLSP server that builds the diagram of a model, lays it out and applies the edits you make in the diagram back to the text, see [Diagram server](GlspServer.md).

**Clients.**

- The [VS Code extension](../vscode/overview.md) starts both servers and shows the diagrams in RosTooling Studio.
- The MCP server of the extension lets AI agents author and generate models, see [MCP server and agent skills](../vscode/mcp-and-agents.md).
- Any editor that speaks LSP can use the language server on its own.
- The Eclipse application is the original client. It uses the same languages in an Eclipse editor and is described as [legacy](Installation.md#eclipse-legacy).

**Plugins (Preview).** A plugin is a JAR that the core loads when you enable it. It adds `@` blocks to the models and files to the generated packages, see [Plugin concepts](PluginConcepts.md). Unknown blocks only produce warnings, so a model stays usable where the plugin is absent.

## Where the data lives

| What | Where |
| --- | --- |
| Models | `.ros`, `.ros1`, `.ros2` and `.rossystem` files in your workspace. They are the only source of truth. |
| Diagram layout | A `.layout` file next to the model. It only stores positions. |
| Catalogue | A folder of ready-made models, by default `ipa320/RosModelsCatalog`, see [Catalogue](../vscode/catalogue.md). |
| Generated code | `src-gen/` in the workspace. |
| Plugin trust | `~/.rostooling/trusted-plugins.json` holds the SHA-256 of every approved plugin JAR, see [Loading and trusting plugins](PluginLoading.md). |

## Versions

The core and the extension have separate versions. The extension bundles a core, so you install one thing. [What's new](WhatsNew.md) lists which extension release contains which core release.
