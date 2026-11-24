---
orphan: true
---

:::{warning}
This page is archived. The command **ROSSDL: Build ROS 2 Package** is a leftover of the earlier `rossdl` tool, which the code generators of RosTooling replace. Build the generated packages with `colcon build` in a terminal, see [Generate and build](../../vscode/generate-and-build.md#build).
:::

# ROSSDL: Build ROS 2 Package

The command `rossdl.buildPackage` of the VS Code extension asks for two folders:

| Folder | Meaning |
| --- | --- |
| ROSSDL workspace | A ROS 2 workspace that has been built already: it must contain `src` and `install/setup.bash`. Its `setup.bash` is sourced before the build. |
| Build workspace | The workspace in which the packages are built. |

It then runs this in the build workspace and shows the output in the **ROS LSP** output channel:

```bash
source "<ROSSDL workspace>/install/setup.bash" && colcon build --symlink-install
```

If the ROSSDL workspace does not contain `src` or `install/setup.bash`, the command stops with an error that names the missing path.
