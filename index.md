# RosTooling

RosTooling bridges the advantages of ROS with those of model-driven software development. At its core are models that describe the code ROS developers write every day, both for a single package and for a whole system. Around the models are tools: validators, code generators, a diagram editor and extractors that create models from existing code.

The source code is open: [RosTooling](https://github.com/ipa320/RosTooling) (languages and generators) and [RosTooling_Extension](https://github.com/ipa-esa/RosTooling_Extension) (VS Code extension). Contributions, feedback and suggestions are welcome.

::::{grid} 1 1 2 2
:gutter: 3

:::{grid-item-card} VS Code Extension
:link: vscode/overview
:link-type: doc

Edit models with language support, draw systems in RosTooling Studio, generate ROS 2 packages and let AI agents author models. **Start here.**
:::

:::{grid-item-card} RosTooling
:link: docu/Installation
:link-type: doc

The languages, the validation and the generators behind every client, and how to install them.
:::
::::

## Start

- [Install](docu/Installation.md) the extension and [try the whole path once](vscode/getting-started.md): open a system, connect nodes, generate a package, build it.
- Learn by [tutorials](docu/LearnRosModels.md): first models, a system, a publisher and subscriber, turtlesim and a mobile base.
- See [What's new in 2026](docu/WhatsNew.md) if you know an earlier version.

## The model layers

RosTooling describes ROS in three layers of models. Each has its own file type, and each layer refers to the one below.

| Layer | Files | Describes |
| --- | --- | --- |
| Communication objects | `.ros` | Message, service and action types, similar to `.msg`, `.srv` and `.action` files. See [Messages, services and actions](docu/NewCommunicationObjects.md). |
| Packages | `.ros2`, `.ros1` | A ROS package: its nodes, their publishers, subscribers, services, actions and parameters. See [Packages](docu/RosModelDescription.md). |
| Systems | `.rossystem` | Nodes in a system and the connections between them. See [Systems](docu/RosSystemModelDescription.md). |

## Where to go

| I want to | Go to |
| --- | --- |
| Generate code from a model | [Code generation](docu/Ros2CodeGeneration.md), [target distributions](docu/TargetDistributions.md) |
| Extend the generators with my own blocks (Preview) | [Plugin concepts](docu/PluginConcepts.md) |
| Create models from existing code | [Static code analysis](docu/StaticCodeAnalyis.md), [introspection](docu/ros2model.md) |
| Use RosTooling from another editor or tool | [LSP commands](docu/LspCommands.md), [diagram server](docu/GlspServer.md) |
| Understand how the parts fit | [Architecture overview](docu/Architecture.md) |
| Fix a problem | [Known issues](docu/RosTooling_issues.md) |
| Work on RosTooling itself | [Building, testing and releasing](docu/Release.md) |
| Use the Eclipse application | [Installation](docu/Installation.md#eclipse-legacy) (legacy) |

The tutorials were written on Linux. Those that run ROS 2 need a local installation of it. The models themselves can be edited on any operating system that runs VS Code and Java 21.

## Feedback and contact

Report problems on the [issue tracker of this site](https://github.com/ipa320/RosTooling.github.io/issues/new?template=bug_report.md) or contact Eshan Savla: [eshan.savla@ipa.fraunhofer.de](mailto:eshan.savla@ipa.fraunhofer.de).

```{toctree}
:hidden:
:caption: Get started

docu/Installation
docu/WhatsNew
docu/Architecture
```

```{toctree}
:hidden:
:caption: Languages

docu/RosModelDescription
docu/ParametersAPI
docu/NewCommunicationObjects
docu/RosSystemModelDescription
docu/Validation
```

```{toctree}
:hidden:
:caption: Code generation

docu/Ros2CodeGeneration
docu/CodeGeneration
docu/TargetDistributions
```

```{toctree}
:hidden:
:caption: Plugins (Preview)

docu/PluginConcepts
docu/PluginBlocks
docu/PluginLoading
docu/PluginDevelopment
docu/PluginBtExample
```

```{toctree}
:hidden:
:caption: Integration

docu/LspCommands
docu/GlspServer
```

```{toctree}
:hidden:
:caption: Models from code

docu/StaticCodeAnalyis
docu/ros2model
```

```{toctree}
:hidden:
:caption: Visualization

docu/SystemModelsVisualization
```

```{toctree}
:hidden:
:caption: Tutorials

docu/LearnRosModels
docu/LearnRosSystemModels
docu/LearnRosSystemModels2
docu/Example_PubSub
docu/Example_Turtlesim
docu/MobileBase_beginner
```

```{toctree}
:hidden:
:caption: Help

docu/RosTooling_issues
docu/Release
```

```{toctree}
:hidden:
:caption: Eclipse (legacy)

docu/Environment_setup
docu/eclipse_issues
docu/EclipsePlugin
```

```{toctree}
:hidden:
:caption: VS Code Extension

vscode/overview
vscode/getting-started
vscode/editing-models
vscode/studio
vscode/auto-connect
vscode/generate-and-build
vscode/catalogue
vscode/plugins
vscode/mcp-and-agents
vscode/reference
```
