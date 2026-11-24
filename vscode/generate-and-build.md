# Generate and build

The extension generates ROS 2 code from your models and can build it. This page describes the commands. What is generated, and how to write your code in it, is on the pages about [ROS 2 code generation](../docu/Ros2CodeGeneration.md) and [System generation](../docu/CodeGeneration.md).

## Generate the nodes of a package

For a `.ros2` model, right-click the file in the Explorer or in the editor and choose **Generate ROS 2 Node Wrappers** (the command `ros2.generateWrappers`). In the Studio, the Component view has **Generate Wrappers** in its toolbar and **Generate ROS 2 Wrappers** at the bottom of its left side.

1. The extension saves the file if it has unsaved changes.
2. You choose the language. For a package with one node it is **C++** or **Python**. For a package with several nodes it is **All C++**, **All Python**, or **Configure per node…**, which asks for the language of each node. A package can have nodes in both languages.
3. The code is written to `src-gen/<package>/` in your workspace folder.

The result is a message with the number of files. Warnings and notices of the generator, including those of [plugins](plugins.md), are shown as messages too.

A node is implemented in one language. If you generate a node again in the other language, the files of the first language are removed. Your logic files, `*Algorithm.hpp` and `*_logic.py`, are never overwritten.

## Generate the package of a system

For a `.rossystem` model, use **Generate ROS 2 Package** in the toolbar of the Studio, or right-click the file and choose **Generate ROS2 Package** (the command `rossystem.triggerCodeGeneration`).

The extension saves the file, validates the model, and writes the package with the launch file, the parameter files, the README and the diagram to `src-gen/<system>/`. If the model has errors, nothing is written and the errors are shown.

The generator reads what is already in `src-gen/`, so it can extend a package that already exists: it adds what is missing and does not replace your files.

## The target distribution

The commands generate for the distribution of the environment variable `ROS_DISTRO`, as VS Code sees it, and for the newest LTS distribution when the variable is not set. To generate for another distribution, start VS Code from a shell in which that distribution is sourced, or use the MCP tool `generate_ros2_wrappers`, which takes the distribution as an argument. See [Target distributions](../docu/TargetDistributions.md).

## Old files

When a generator no longer produces a file that a plugin wrote before, the file is removed. This only happens in the folders that belong to the plugin, `config/<plugin>/`, `launch/<plugin>/` and `plugins/<plugin>/`, never for your own files.

## Build

Copy or link the generated packages into the `src/` folder of a ROS 2 workspace and build them in a terminal in which ROS 2 is sourced:

```bash
cd <workspace>
rosdep install --from-path src/ -i -y
colcon build --symlink-install
source install/setup.bash
```

The README of a generated system package has the same commands. An AI agent builds with the MCP tool `build_ros2_package`, which runs `colcon build --symlink-install` in the workspace.

The extension also has the older command **ROSSDL: Build ROS 2 Package**. It is a leftover of the tool that the code generators replace, and it is [archived](../docu/old/RossdlBuildCommand.md).
