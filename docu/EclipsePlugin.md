# Running a plugin inside Eclipse (legacy)

[Plugins](PluginConcepts.md) are loaded from JAR files by the language server of VS Code. In the Eclipse application there is a second way, which Eclipse developers know: a plugin is an OSGi bundle that registers a `RosToolingPlugin` as an OSGi service.

When the core runs inside Eclipse it asks the OSGi service registry for every registered `RosToolingPlugin`, next to the plugins found in folders. A plugin that is found this way is used like the others: it has an id, it adds its blocks to the models and its files to the generated packages. In a standalone run, outside OSGi, this lookup does nothing and does not need the OSGi classes.

To try a plugin in Eclipse:

1. Develop the plugin as described in [Plugin development](PluginDevelopment.md), but package it as an OSGi bundle with the `RosToolingPlugin` implementation registered as a service (declarative services or a `BundleActivator`).
2. Run the RosTooling application from your Eclipse workspace as an *Eclipse Application*, with the bundle of your plugin selected in the run configuration, see [Installation](Installation.md#install-from-source).
3. Open a model that uses one of its `@` blocks.

This page describes only what the core does. The way of working with plugins that is documented and tested is the VS Code one, see [Loading and trusting plugins](PluginLoading.md).
