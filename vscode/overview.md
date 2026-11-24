# VS Code Extension: overview and installation

The RosTooling extension for Visual Studio Code brings the RosTooling languages into the editor: `.ros`, `.ros1`, `.ros2` and `.rossystem` files. It combines three things.

- **Language support** for the four file types: highlighting, completion, validation while you type, hover text and navigation, from the same language server that all RosTooling tools use. See [Editing models](editing-models.md).
- **RosTooling Studio**, a diagram editor for the same files: the architecture of a system as a graph, the nodes of a package as cards, and the message types as schemas. Every change in the diagram is written to the text file. See [RosTooling Studio](studio.md).
- **Generation and agent tools**: generate ROS 2 packages from the models, build them, and let AI agents author models through an MCP server. See [Generate and build](generate-and-build.md) and [MCP server and agent skills](mcp-and-agents.md).

## Requirements

| | |
| --- | --- |
| Visual Studio Code | Version 1.107.0 or later. Other editors that run VS Code extensions, such as VSCodium, work in the same way. |
| Java | JDK 21 or later. The language server is a Java program. |
| Operating system | Linux, or macOS. |
| ROS 2 | Only to build and run the generated packages. Humble, Jazzy, Kilted, Lyrical and Rolling are supported, see [Target distributions](../docu/TargetDistributions.md). |

## Installation

### From the Open VSX Registry

The extension is published on the [Open VSX Registry](https://open-vsx.org/extension/RosTooling/rostooling-languages) as `RosTooling.rostooling-languages`. Editors that use Open VSX, such as VSCodium, find it in the Extensions view: search for `RosTooling Languages` and choose **Install**. From a terminal:

```bash
codium --install-extension RosTooling.rostooling-languages
```

The Microsoft build of VS Code uses another registry, so install it from a VSIX file instead.

### From a VSIX file

Every release on the [GitHub releases page](https://github.com/ipa-esa/RosTooling_Extension/releases) has a file `rostooling-languages-<version>.vsix`. Download it and install it with **Extensions: Install from VSIX…** in the Command Palette, with the **…** menu of the Extensions view, or from a terminal:

```bash
code --install-extension rostooling-languages-<version>.vsix
```

### From source

```bash
git clone https://github.com/ipa-esa/RosTooling_Extension.git
cd RosTooling_Extension
./gradlew installExtension
```

The command builds the language server, packages the extension and installs it.

### Pre-release versions

Pre-release builds let you try changes before they are released. In the Extensions view, open **RosTooling Languages** and choose **Switch to Pre-Release Version**. **Switch to Release Version** takes you back.

## The first start

When the extension starts it checks the Java version, starts the language server in the background, and downloads the default catalogue of ROS models if it is not on your computer yet, see [Catalogue](catalogue.md). Progress and problems are written to the **ROS LSP** output channel. If something does not work, see [Commands, settings and troubleshooting](reference.md).
