# Review system models

A system model, `.rossystem`, composes nodes and connects them. In this tutorial you build one step by step, with the help and the error messages of the editor, and then look at what is generated from it. It takes about 20 minutes.

**What you need:** the [VS Code extension](../vscode/overview.md), and the knowledge of the [previous tutorial](LearnRosModels.md). The reference for the language is [System language](RosSystemModelDescription.md).

## 1. The nodes to compose

A system needs package models for its nodes. Create two files, `awesome_pkg.ros2` and `awesome2_pkg.ros2`. The first node has two publishers with different types and one parameter of each type, the second has a subscriber and a string parameter. They describe invented nodes, as every part of this tutorial.

<!-- check: group=pkgs file=awesome_pkg.ros2 -->
```text
awesome_pkg:
  artifacts:
    awesome:
      node: awesome_node
      publishers:
        awesome_pub:
          type: "std_msgs/msg/String"
        hello_world_pub:
          type: "std_msgs/msg/Int32"
      parameters:
        awesome_array_param:
          type: Array [String]
        awesome_bool_param:
          type: Boolean
        awesome_double_param:
          type: Double
        awesome_integer_param:
          type: Integer
        awesome_string_param:
          type: String
        awesome_struct_param:
          type: Struct [first Integer, second Boolean]
```

<!-- check: group=pkgs file=awesome2_pkg.ros2 -->
```text
awesome2_pkg:
  artifacts:
    awesome2:
      node: awesome2_node
      subscribers:
        awesome2_sub:
          type: "std_msgs/msg/String"
      parameters:
        awesome2_string_param:
          type: String
```

## 2. Create the system file

::::{tab-set}
:::{tab-item} VS Code
Create a file named `my_awesome_system.rossystem` in the same folder. The first line of the file is the name of the system, followed by a colon. On the next line, indent, and press `Ctrl+Space` to see what is allowed.
:::
:::{tab-item} Eclipse
Use **File → New → Other → General → File**, and give the file the extension `.rossystem`.
:::
::::

## 3. Add the nodes

Under `nodes:` every node gets a label and a `from:`, which refers to the package model and the node in it, written `<package>.<node>`:

<!-- check: group=s1 file=my_awesome_system.rossystem with=tools/fixtures/awesome -->
```text
my_awesome_system:
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
    "node2":
      from: "awesome2_pkg.awesome2_node"
```

This is already a valid system of two nodes. Generate its package with **Generate ROS 2 Package** (in Eclipse the package is generated when you save the file). The result is in `src-gen/my_awesome_system/`, see [System generation](CodeGeneration.md): a launch file that starts both nodes, and the files to build it.

## 4. Expose the interfaces

A node has all the interfaces of its package model, but a system lists the ones that it uses on purpose, so that the architect of the system decides which ports are part of the design. Under `interfaces:` every entry has a label, an arrow for the kind of interface, and the reference `<artifact>::<interface>`:

<!-- check: group=s2 file=my_awesome_system.rossystem with=tools/fixtures/awesome -->
```text
my_awesome_system:
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
```

## 5. Connect the nodes

A connection joins a publisher to a subscriber of the same type. It refers to the labels:

<!-- check: group=s3 file=my_awesome_system.rossystem with=tools/fixtures/awesome -->
```text
my_awesome_system:
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
  connections:
    - ["my_pub", "my_sub"]
```

In the diagram of the Studio the same system looks like this:

```{image} images/tut_system_connected.png
:alt: The system with two nodes and one connection in RosTooling Studio
:width: 100%
```

### When a connection is wrong

Try to connect ports that do not fit. Here the publisher `int_pub` has the type `std_msgs/msg/Int32` and the subscriber has `std_msgs/msg/String`:

<!-- check: group=s4 file=my_awesome_system.rossystem with=tools/fixtures/awesome expect="have different types" -->
```text
my_awesome_system:
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
        - "int_pub": pub-> "awesome::hello_world_pub"
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
  connections:
    - ["int_pub", "my_sub"]
```

The validator reports "A connection can only be formed by interfaces with the same type", and no launch file is generated while the error is there. The same happens when you connect different kinds of ports, such as a topic with a service.

## 6. Namespaces and parameters

A node can have a namespace, and the system can set the values of the parameters of its nodes. The reference to a parameter is `<artifact>::<parameter>`, and `value:` follows on the next line:

<!-- check: group=s5 file=my_awesome_system.rossystem with=tools/fixtures/awesome -->
```text
my_awesome_system:
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
      namespace: "my_ns"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
      parameters:
        - "string_example": "awesome2::awesome2_string_param"
          value: "hello"
  connections:
    - ["my_pub", "my_sub"]
```

The value must fit the type of the parameter. A number for a string parameter is an error:

<!-- check: group=s6 file=my_awesome_system.rossystem with=tools/fixtures/awesome expect="Mismatched input" -->
```text
my_awesome_system:
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
      namespace: "my_ns"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
      parameters:
        - "string_example": "awesome2::awesome2_string_param"
          value: 5
  connections:
    - ["my_pub", "my_sub"]
```

The message is "Mismatched input ParameterInteger expecting ParameterStringType". See [Parameters](ParametersAPI.md) for all types and their values.

## 7. Parameters of all types

All types of values together, a list, a boolean, a double, an integer, a string and a struct:

<!-- check: group=s7 file=my_awesome_system.rossystem with=tools/fixtures/awesome -->
```text
my_awesome_system:
  nodes:
    "node1":
      from: "awesome_pkg.awesome_node"
      interfaces:
        - "my_pub": pub-> "awesome::awesome_pub"
      parameters:
        - "array_example": "awesome::awesome_array_param"
          value: ["hello", "hallo"]
        - "bool_example": "awesome::awesome_bool_param"
          value: true
        - "double_example": "awesome::awesome_double_param"
          value: 1.2
        - "integer_example": "awesome::awesome_integer_param"
          value: 2
        - "string_example": "awesome::awesome_string_param"
          value: "hallo"
        - "struct_example": "awesome::awesome_struct_param"
          value: [
            first: 2
            second: true]
    "node2":
      from: "awesome2_pkg.awesome2_node"
      interfaces:
        - "my_sub": sub-> "awesome2::awesome2_sub"
  connections:
    - ["my_pub", "my_sub"]
```

For every node with parameters the generator writes a parameter file, `config/<node>.yaml`, in the generated package. How the launch file uses the values depends on the node. A node with up to five parameters, none of them a struct, gets them as launch arguments, so they can be changed when the system starts: `ros2 launch my_awesome_system my_awesome_system.launch.py string_example:=hi`. A node with more than five parameters, or with a struct value, as in this example, has its launch file load the parameter file instead, which is easier to read for many values.

With this example you have used the main parts of the language. The next tutorial adds [subsystems](LearnRosSystemModels2.md).
