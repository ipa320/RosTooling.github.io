# Plugins: loading and trust

:::{note}
**Preview.** The plugin support described in this section is not released yet and may still change.
:::

A RosTooling plugin is a Java library in a JAR file. Enabling a plugin runs its code with your permissions, so RosTooling loads a plugin only when you have enabled it and approved that exact file. This page explains the steps and the checks behind them.

## What a plugin JAR contains

A plugin JAR has the file `META-INF/rostooling-plugin.properties`. RosTooling reads this file, and only this file, to list a plugin, without running any of its code.

| Key | Meaning |
| --- | --- |
| `id` | The unique id of the plugin, for example `de.fraunhofer.ipa.ros.plugin.bt`. |
| `version` | The version of the plugin. |
| `summary` | One line that says what the plugin does. |
| `pluginApiVersionRange` | The versions of the plugin interface that the plugin was written for, for example `[1.0.0,2.0.0)`. **Required.** |
| `supportedCoreVersionRange` | Optional. The RosTooling versions that the plugin needs, for a plugin that uses a newer language feature. |
| `documentation` | Optional. The path of a Markdown file inside the JAR that documents the plugin. |

A JAR without this file, or whose `pluginApiVersionRange` does not include the version of the plugin interface of your RosTooling, is not loaded, and the reason is shown. Everything else about the JAR stays untouched: it is not unpacked and no class from it is loaded while it is only listed.

## Enabling a plugin in VS Code

1. Open the Command Palette and run **RosTooling: Manage Plugins…**, or open the **RosTooling Plugins** view in the Explorer.
2. Choose **Add plugin JAR or folder…**, and select a `.jar` file or a folder that contains plugin JARs. This adds the path to the setting `rostooling-languages.plugins.paths`. Absolute paths, paths relative to the workspace and `${workspaceFolder}` can be used.
3. The view lists the plugins that it found, with their id, version and summary. Enable the plugin you want. Its id is added to the setting `rostooling-languages.plugins.enabled`.
4. VS Code shows the file name and the SHA-256 fingerprint of the JAR and asks you to approve it, see below.

When you enable a plugin, you choose whether the setting is saved for this workspace, which can be shared with your team, or for all your workspaces. RosTooling then restarts the language server and the diagram server, and updates the AI agent configurations that it manages.

A plugin that has documentation can be opened from the context menu of the view with **Open Documentation**. A plugin that cannot be loaded shows the reason, **Show Load Error**.

A plugin JAR without the properties file appears in the view with the note that it has no plugin metadata. It is not loaded.

## Approval

A plugin is loaded only when the SHA-256 fingerprint of its JAR is in the approval file `~/.rostooling/trusted-plugins.json`. The first time a JAR is enabled, and every time the file changes, VS Code asks you to confirm the fingerprint, and then records it:

```json
{
  "version": 1,
  "approved": [
    {
      "sha256": "a2c1e6354d7c09ea…",
      "fileName": "rostooling-bt-plugin-3.6.0.jar",
      "id": "de.fraunhofer.ipa.ros.plugin.bt",
      "approvedAt": "2026-11-19T10:32:22.220Z"
    }
  ]
}
```

Only the fingerprint decides. The file name and the id are for display and for explaining a rejected file: a JAR that has the name or id of an approved one but another fingerprint is shown as **changed**, and any other unapproved JAR is shown as **unapproved**. Neither is loaded. A rebuilt plugin therefore has to be approved again.

Approved JARs are not loaded from where they are. RosTooling reads the file, checks the fingerprint of the bytes it has read, and loads a copy from a private folder that only you can access. The file cannot change between the check and the use.

- **Untrusted workspaces:** the settings for plugins in `.vscode/settings.json` are ignored until you trust the workspace in VS Code.
- **Approval of unlabelled JARs:** a JAR that the view lists as needing approval has the action **Approve JAR** in its context menu.

Only enable plugins from sources that you trust.

## Using a plugin without VS Code

The MCP server and other tools that start the language server directly use the same rules, including the approval check. A plugin is given to them by path and by id:

```bash
rostooling-mcp \
  --plugins-dir /path/to/rostooling-bt-plugin-3.6.0.jar \
  --enable-plugins de.fraunhofer.ipa.ros.plugin.bt
```

The two options are the same as the environment variables `ROS_PLUGINS_DIR` (paths of JARs or folders) and `ROS_PLUGINS_ENABLED` (a list of plugin ids). The approval file is the same file, so a plugin that you approved in VS Code is approved for the MCP server too.

## Versions

Two versions decide whether a plugin can be loaded.

| Version | Meaning |
| --- | --- |
| Plugin interface | The version of the classes that a plugin uses. Currently 1.0.0. Adding something raises the minor version, removing or changing something raises the major version, and a plugin states which versions it accepts in `pluginApiVersionRange`. |
| RosTooling | A plugin can require a RosTooling version range for a language feature that it uses. |

## Plugins that are needed by others

A plugin can name other plugins that have to run before it. RosTooling orders the enabled plugins accordingly.
