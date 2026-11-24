# Parameters

Parameters are declared in a package model, on a node, and can be given new values in a system model. The tooling checks that every value fits the declared type and uses the values when it generates code and launch files.

## Types

| Type | Value | Example |
| --- | --- | --- |
| `Boolean` | `true` or `false`, lowercase and without quotes | `true` |
| `Integer` | a whole number | `42` |
| `Double` | a number with a decimal point | `1.0` |
| `String` | text in double quotes | `"hello"` |
| `Base64` | binary data, written as `0b` followed by binary digits | `0b01000001` |
| `List [T1, T2, ...]` | a list with a fixed number of elements, each of its own type | `[1, 1, "hello"]` |
| `Array [T]` | a list of any length with elements of one type | `["a", "b", "c"]` |
| `Struct [name T, ...]` | named members, each with its own type | `hello: 1` and `what: "test"` on separate lines |

A `Double` needs the decimal point: `1` is an `Integer`, not a `Double`, and the validator rejects it as the value of a `Double` parameter.

## Declaring parameters in a package model

A parameter has a name and a `type:`. A `default:` can follow the type, and `value:` gives the value that the generated node starts with. Parameters can also have a [quality of service](RosModelDescription.md#quality-of-service) block.

<!-- check: group=declare file=params_example.ros2 -->
```text
params_example:
  artifacts:
    params_example:
      node: params_example
      parameters:
        string_test:
          type: String
          default: "hello"
        bool_test:
          type: Boolean
          default: true
        double_test:
          type: Double
          default: 1.5
        integer_test:
          type: Integer
          default: 3
        base64_test:
          type: Base64
        list_test:
          type: List [Integer, Integer, String]
        array_test:
          type: Array [String]
          default: ["a", "b"]
        struct_test:
          type: Struct [hello Integer, what String]
```

## Setting values in a system model

A [system model](RosSystemModelDescription.md#setting-a-parameter-of-a-node) overrides a parameter of a node. The entry has a label, a reference `"<artifact>::<parameter>"` and a `value:`. The value must fit the declared type.

<!-- check: group=override file=params_example.ros2 -->
```text
params_example:
  artifacts:
    params_example:
      node: params_example
      parameters:
        string_test:
          type: String
        bool_test:
          type: Boolean
        double_test:
          type: Double
        integer_test:
          type: Integer
        list_test:
          type: List [Integer, Integer, String]
        array_test:
          type: Array [String]
        struct_test:
          type: Struct [hello Integer, what String]
```

<!-- check: group=override file=params.rossystem -->
```text
params_system:
  nodes:
    "params_node":
      from: "params_example.params_example"
      parameters:
        - "test_s": "params_example::string_test"
          value: "hello"
        - "test_b": "params_example::bool_test"
          value: true
        - "test_d": "params_example::double_test"
          value: 1.1
        - "test_i": "params_example::integer_test"
          value: 1
        - "test_l": "params_example::list_test"
          value: [1, 1, "hello"]
        - "test_a": "params_example::array_test"
          value: ["hello", "hola", "hallo"]
        - "test_st": "params_example::struct_test"
          value: [
            hello: 1
            what: "test"]
```

A binary value for a `Base64` parameter is written like this:

:::{note}
Base64 values need a RosTooling version that is newer than the current release. The server of the current release rejects them.
:::

<!-- check: group=base64 file=base64_pkg.ros2 skip="Base64 values are rejected by the server" -->
```text
base64_pkg:
  artifacts:
    coder:
      node: coder
      parameters:
        payload:
          type: Base64
```

<!-- check: group=base64 file=base64.rossystem skip="Base64 values are rejected by the server" -->
```text
base64_system:
  nodes:
    "coder":
      from: "base64_pkg.coder"
      parameters:
        - "payload": "coder::payload"
          value: 0b01000001
```

A struct value is written in square brackets, with one `name: value` pair for each member on its own line, as in `test_st` above.

## Wrong values

The validator compares the value with the type and says what is wrong. For example, a list that has a different length than the declared `List` type, or an element of the wrong type:

<!-- check: group=wrong file=wrong_pkg.ros2 -->
```text
wrong_pkg:
  artifacts:
    wrong:
      node: wrong
      parameters:
        pair:
          type: List [Integer, String]
```

<!-- check: group=wrong file=wrong.rossystem expect="Expect a list of 2 elements" -->
```text
wrong_system:
  nodes:
    "wrong":
      from: "wrong_pkg.wrong"
      parameters:
        - "pair": "wrong::pair"
          value: [1, "two", 3]
```

## Parameters of the whole system

A system model can declare parameters of its own. They have a type and a value, and become launch arguments of the generated launch file. See [Parameters of the whole system](RosSystemModelDescription.md#parameters-of-the-whole-system).

## Launch substitutions

A `String` value can be computed when the system starts, with a launch substitution such as `$(var name)`, `$(env NAME)` or `$(find-pkg-share package)`. See [Launch substitutions](RosSystemModelDescription.md#launch-substitutions).

## Slot parameters

A plugin slot of a node declares its own parameter, which holds the plugins that a system attaches. That parameter must not be declared again under `parameters:` of the node. See [Plugins](RosModelDescription.md#plugins).
