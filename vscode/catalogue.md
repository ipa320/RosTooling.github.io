# Catalogue

The catalogue is the library of ready models that RosTooling resolves your models against: the message, service and action types of ROS, and models of nodes, packages and whole systems that you can reuse. Types such as `std_msgs/msg/String` in a `type:` are found there, and **+ Import Node** in the Studio lists its nodes.

## What it contains

The extension keeps two repositories on your computer.

| Repository | Content |
| --- | --- |
| [RosCommonObjects](https://github.com/ipa320/RosCommonObjects) | The `.ros` models of the common message, service and action types. |
| [RosModelsCatalog](https://github.com/ipa-nhg/RosModelsCatalog) | `.ros2` and `.rossystem` models of existing ROS nodes, robots, drivers and systems. |

On the first start the extension downloads them, and shows the progress. If a copy of the repositories exists from another editor on the same computer, it is reused instead of downloaded again. After that they are updated in the background when the extension starts, and when you click **Sync Catalogue**. The files are stored in the folder `catalogue_repos` of the storage of the extension, or in `~/.rostooling/catalogue_repos` when the storage is not available.

If the download fails, the standard ROS types may not resolve until you sync again, and the extension says so.

## Browsing

**Catalogue** in the toolbar of the Studio opens the **RosTooling Model Catalogue**, a drawer on the side. Each entry shows the name, the package, where it comes from and how many interfaces it has, and an **Import** button. The tabs show how many entries they have. It has these tabs:

```{image} images/studio-catalogue.png
:alt: The catalogue drawer with the list of nodes
:width: 100%
```

| Tab | Content | Where |
| --- | --- | --- |
| Nodes | The nodes of the `.ros2` models in the catalogue and in your workspace, with their packages and interfaces. A search box and a filter by kind of interface narrow the list. | System view |
| Subsystems | The `.rossystem` models that can be added as subsystems. A subsystem can be opened in the Studio from here. | System view |
| Types | The message, service and action types. | all views |
| Sources | Where the catalogue comes from. | all views |

Choosing a node or subsystem adds it to your system. Models that come from the catalogue are read-only, and marked as such.

## Your own models

Your workspace is part of the catalogue: the `.ros2`, `.ros` and `.rossystem` files in it are found in the same way. To use models from a place outside of the workspace, open the **Sources** tab and click **Add Custom Folder…**. The folder is indexed and its models appear in the other tabs. The **Custom Folders** list of the tab has a button for removing a folder again. The folders are saved in a configuration file in the catalogue storage.

```{image} images/studio-catalogue-sources.png
:alt: The Sources tab of the catalogue drawer
:width: 100%
```

**Sync GitHub Catalogue** in the same tab updates the two repositories.

## Models of your package in the catalogue

A package model that the system refers to, as in `from: "my_package.my_node"`, has to be found by RosTooling. This is the case when it is in the workspace or in a source of the catalogue. If a reference stays unresolved, check the **Sources** tab, save the file that contains the model, and sync the catalogue.
