# Plugins: extension blocks

:::{note}
**Preview.** The plugin support described in this section is not released yet and may still change.
:::

A RosTooling plugin adds its own information to a model with an **extension block**. A block starts with `@` and a name, and holds entries in the language of the plugin, for example the ports of a BehaviorTree leaf. RosTooling parses and checks the block against the schema that the plugin provides, completes and explains its entries in the editor, and lets the diagram and AI agents edit it, without knowing anything about the plugin itself.

## Syntax

A block has a name and entries. The entries follow indented on the next lines, or between braces on the same line.

<!-- check: group=blocksyntax file=blocks_pkg.ros2 expect="Unknown extension block" -->
```text
blocks_pkg:
  artifacts:
    worker:
      node: worker
      publishers:
        status:
          type: "std_msgs/msg/String"
      @probe:
        mode: always
        threshold: 0.5
        enabled: true
        tags: [fast, local]
        limits:
          min: 1
          max: 10
        topic: pub-> status
```

An entry is `key: value`. A value is one of

| Value | Example |
| --- | --- |
| a scalar: text, a number or `true` and `false` | `mode: always`, `threshold: 0.5` |
| a list | `tags: [fast, local]`, or `[]` for an empty list |
| a map, indented or in braces | `limits:` with entries below it, or `limits: { min: 1, max: 10 }` |
| a reference to an interface or a parameter of the same element | `topic: pub-> status` |

The arrow of a reference says what it points to: `pub->` a publisher, `sub->` a subscriber, `ss->` a service server, `sc->` a service client, `as->` an action server, `ac->` an action client, and `param->` a parameter. RosTooling resolves the reference like any other, so a name that does not exist is reported, and the editor offers completion.

The braces form fits a short block on one line:

<!-- check: group=blockbraces file=braces_pkg.ros2 expect="Unknown extension block" -->
```text
braces_pkg:
  artifacts:
    worker:
      node: worker
      @probe: { mode: once, threshold: 0.5 }
```

## Where a block can be written

Each block says in which places it may appear. The places are:

| Place | In the model |
| --- | --- |
| Node | the node of an artifact, in a `.ros2` file |
| Plugin | a plugin of an artifact, in a `.ros2` file |
| System | the system itself, in a `.rossystem` file |
| RosNode | a node of a system |
| RosInterface | an interface entry of a node in a system |

<!-- check: group=blockplaces file=places_pkg.ros2 -->
```text
places_pkg:
  artifacts:
    talker:
      node: talker
      publishers:
        chatter:
          type: "std_msgs/msg/String"
```

<!-- check: group=blockplaces file=places.rossystem expect="Unknown extension block" -->
```text
places_system:
  @probe:
    mode: once
  nodes:
    "talker":
      from: "places_pkg.talker"
      interfaces:
        - "chatter": pub-> "talker::chatter"
          @probe:
            mode: always
      @probe:
        mode: always
```

A block in a `.ros2` file describes the node or plugin in general, and the same block in a system describes one use of it. RosTooling merges them for generation: for a value that is a single entry, the system's value replaces the one of the package, and lists are joined without duplicates. A plugin can choose another rule for a field, including reporting a conflict as an error.

## Names

A block is written with the name that the plugin gives it, such as `@bt`. If two enabled plugins use the same block name, the model has to say which one it means by writing the plugin id before the name, `@<plugin id>.<name>`, and the validator reports the name without it as an error.

## Models without the plugin

A block of a plugin that is not enabled is only a warning, "Unknown extension block", so a model stays valid and generates on a machine that does not have the plugin.

## Editing blocks

- In the **editor**, the plugin's schema provides completion, hover text and the validation of the entries.
- In the **diagram**, the inspector of an element lists its blocks. Each field has a label, help text and, where the plugin suggests values, a list of choices; you can type another value. The inspector also offers the blocks that the active plugins provide for that kind of element.
- An **AI agent** edits blocks with the MCP tools `ros_get_extension_block` and `ros_set_extension_block`. A plugin can add tools of its own, such as one that adds a complete BehaviorTree leaf.
