# Plugins: writing a plugin

:::{note}
**Preview.** The plugin interface is version 1.0.0 and is not released yet. It may still change.
:::

This page is for developers who want to extend RosTooling with a plugin of their own: a Java library that adds blocks to the model language, checks them, generates code and file for them, and offers tools to editors and AI agents. For the models that describe pluginlib plugins, see [Plugins: concepts](PluginConcepts.md).

## What a plugin can do

| Part | Purpose |
| --- | --- |
| Blocks | Add `@<name>:` [blocks](PluginBlocks.md) with a schema of your own to the places of a model. |
| Validation | Check what the schema cannot express, after RosTooling has checked the schema. |
| Loaders | Provide a `loader:` for [plugin bases](PluginConcepts.md#loaders) and generate the registration code of the plugins that use it. |
| Package generation | Add files and build contributions to the package that is generated from a `.ros2` file. |
| System generation | Add files and launch includes to the package that is generated from a `.rossystem` file. |
| Tools | Add tools that editors and AI agents can call, for example to add a complete element to a model. |

A plugin has to follow the rules of RosTooling. It does not change the language, and it does not replace or edit what RosTooling generates. Everything a plugin generates is added to what RosTooling generates, in files that only the plugin writes.

## The project

A plugin is an ordinary Maven project that builds a JAR. RosTooling and the Eclipse libraries are provided by the host at run time, so they have `provided` scope:

```xml
<dependencies>
  <dependency>
    <groupId>de.fraunhofer.ipa.ros</groupId>
    <artifactId>de.fraunhofer.ipa.ros</artifactId>
    <version>3.6.0</version>
    <scope>provided</scope>
  </dependency>
  <dependency>
    <groupId>de.fraunhofer.ipa.ros</groupId>
    <artifactId>de.fraunhofer.ipa.ros2.distro</artifactId>
    <version>3.6.0</version>
    <scope>provided</scope>
  </dependency>
  <!-- org.eclipse.emf.ecore, org.eclipse.emf.common and org.eclipse.xtext: provided as well -->
</dependencies>
```

It needs Java 21. The RosTooling libraries are published to the GitHub Packages of the RosTooling repository.

Two files in `src/main/resources/META-INF` make the JAR a plugin:

`services/de.fraunhofer.ipa.ros.plugin.spi.RosToolingPlugin` contains the name of your plugin class, one line:

```text
de.example.probe.ProbePlugin
```

`rostooling-plugin.properties` describes the plugin without running it. See [Loading and trust](PluginLoading.md#what-a-plugin-jar-contains) for the keys:

```text
id=de.example.probe
version=1.0.0
summary=Adds the @probe block and a probe loader.
pluginApiVersionRange=[1.0.0,2.0.0)
```

The `id` and `version` must be the same as `id()` and `version()` of the plugin class.

## The plugin class

A plugin implements `RosToolingPlugin`. Only `id()`, `blocks()` and `validate(...)` are required, the rest has defaults.

| Method | Purpose |
| --- | --- |
| `id()`, `version()` | Identify the plugin. |
| `supportedCoreVersionRange()` | The RosTooling versions that the plugin needs. Empty means any. |
| `dependencies()` | The ids of plugins that must run before this one. |
| `documentationMarkdown()` | Documentation that the plugin manager shows. |
| `blocks()` | The `@` blocks of the plugin, as `BlockDescriptor`s. |
| `loaders()` | The loaders of the plugin, as `LoaderDescriptor`s. |
| `validate(ValidationContext)` | Semantic checks of a block. |
| `generatePackage(PackageGenerationContext)` | Phase B of the generation of a package. |
| `generateSystem(SystemGenerationContext)` | Phase B of the generation of a system. |
| `tools()` | The tools of the plugin, as `PluginToolDescriptor`s. |

This is a complete small plugin that is used in the tests of RosTooling. It has a block `@probe` for nodes and a loader `probe`.

```java
public final class ExternalProbePlugin implements RosToolingPlugin {
    private static final EClass SCHEMA = schema();

    private static EClass schema() {
        EcoreFactory f = EcoreFactory.eINSTANCE;
        EPackage pkg = f.createEPackage();
        pkg.setName("probe");
        pkg.setNsPrefix("probe");
        pkg.setNsURI("http://example.org/probe");
        EClass cls = f.createEClass();
        cls.setName("ProbeConfig");
        SchemaHints.set(cls, SchemaHints.LABEL, "Probe");
        SchemaHints.set(cls, SchemaHints.HELP, "Settings of the probe.");
        EAttribute mode = f.createEAttribute();
        mode.setName("mode");
        mode.setEType(EcorePackage.Literals.ESTRING);
        SchemaHints.set(mode, SchemaHints.LABEL, "Mode");
        SchemaHints.set(mode, SchemaHints.CHOICES, "once, always");
        EAttribute rate = f.createEAttribute();
        rate.setName("rate");
        rate.setEType(EcorePackage.Literals.EINT);
        cls.getEStructuralFeatures().add(mode);
        cls.getEStructuralFeatures().add(rate);
        pkg.getEClassifiers().add(cls);
        return cls;
    }

    @Override public String id() { return "de.example.probe"; }
    @Override public String version() { return "1.0.0"; }

    @Override public List<BlockDescriptor> blocks() {
        return List.of(new BlockDescriptor("probe", SCHEMA, EnumSet.of(HookLocation.NODE)));
    }

    @Override public List<LoaderDescriptor> loaders() {
        return List.of(new LoaderDescriptor("probe", "Loads probe plugins at start-up"));
    }

    @Override public void validate(ValidationContext context) { }

    @Override public void generatePackage(PackageGenerationContext context) {
        for (Plugin plugin : context.loaderPlugins()) {
            context.writeFile("probe/" + plugin.getName() + ".txt",
                              "probe registers " + plugin.getName() + "\n");
        }
    }
}
```

### Blocks and schemas

The schema of a block is an Ecore class. Its attributes are the entries of the block, with their types, and the type checks of the entries come from it. A `BlockDescriptor` joins the schema with the name of the block, the places where the block may appear (`SYSTEM`, `ROS_NODE`, `NODE`, `PLUGIN`, `ROS_INTERFACE`), optional rules for merging the block of a package with the block of a system (`MergeStrategy`), and a description.

Hints for editors are annotations on the schema, set with `SchemaHints`: a `LABEL`, a `HELP` text, an `ORDER` of the fields and a list of `CHOICES`. They are used for completion, hover text, the diagram inspector and the MCP tools. They never decide what is valid: a value that is not among the choices is accepted if the type fits.

### Validation

RosTooling checks the block against the schema first. `validate(...)` is called after that succeeds, for each block of the plugin, and reports what the schema cannot: a reference that must point to the right kind of interface, or entries that depend on each other. The `ValidationContext` gives the block, its element, the resource and the workspace folder, and has `error`, `warning` and `info` methods.

### Loaders

A loader says how the plugins of a base are registered. A base names a loader with `loader: <id>`. An id that is not `pluginlib` must be provided by exactly one enabled plugin. RosTooling builds the library of the plugins of such a base, and hands them to your `generatePackage` in `context.loaderPlugins()`. You generate their loading code.

### Generating files

`generatePackage` and `generateSystem` run in a second phase, after RosTooling has generated its own files. The context gives you what you need and enforces the rules:

- `writeFile(path, content)` writes a file, and replaces it every time you generate.
- `writeOnce(path, content)` writes a file for the user to edit, and never overwrites it. This is for code that the user completes, such as a `…Logic.hpp`.
- `deleteStaleFile(path)` removes a file of your own that is no longer needed.
- `readOwnedFile(path)` reads a file of your own from an earlier generation.
- `addNotice(text)` adds a message to the result of the generation, shown with the id of your plugin. Use it for what the user should know but is not an error.
- `ros2Distro()` is the [target distribution](TargetDistributions.md), which RosTooling resolved. Never resolve a distribution yourself.
- `typedBlock(...)` and `mergedBlock(...)` give your blocks as objects of your schema, merged between package and system.

A plugin never writes a file that RosTooling owns, a file that the user is protected from losing, or a file that another plugin owns. Such a write is an error that names the plugin. A plugin also does not depend on the content of the files of RosTooling.

The **package build** that a plugin adds to a package, with `context.packageBuild()`, consists of structured entries, not text: package dependencies, `find_package` calls, library and executable targets, install rules, test targets and `package.xml` exports. RosTooling checks and escapes them, and writes the `CMakeLists.txt` and the `package.xml` itself.

In a **system**, a plugin writes only into folders that are named after it, `config/<plugin>/`, `launch/<plugin>/` and `plugins/<plugin>/`. To start something, it adds a launch include with `addLaunchInclude(path, arguments)`. RosTooling puts it in its own scoped group in the launch file of the system, with the arguments given as literals, as launch arguments of the system or as substitutions.

### Tools

A `PluginToolDescriptor` has a name, a description, an input schema and a handler. The handler receives the model and the arguments and returns a `ToolResult`, with the changed text of the model, or an error. The tools are offered to editors and AI agents while the plugin is enabled.

## Testing

Test a plugin like any other generator: with models as text, the real RosTooling libraries on the test class path, and expected output files. The BehaviorTree plugin keeps golden files for every ROS 2 distribution and builds them in the official ROS images, see [the example](PluginBtExample.md).

## Compatibility

The classes that a plugin uses form the plugin interface, which has a version. RosTooling keeps a snapshot of its public methods and fails its build when they change without a new version. Your plugin states the versions it accepts in `pluginApiVersionRange`, and a RosTooling that has a version outside of it does not load the plugin.
