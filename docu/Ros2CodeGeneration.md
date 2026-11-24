# ROS 2 code generation

From a [package model](RosModelDescription.md) (`.ros2`) RosTooling generates the ROS 2 code of its nodes: the publishers, subscribers, services, actions and parameters, the executables, and the build files. You write only the logic of each node, in one file that the generator never overwrites.

## Generating

**In VS Code**, right-click a `.ros2` file in the editor or the Explorer and choose **Generate ROS 2 Node Wrappers**. You are asked for the implementation language, C++ or Python, for the whole package or, if you prefer, for each node.

**With an AI agent**, the RosTooling MCP server offers the tool `generate_ros2_wrappers`:

| Argument | Meaning |
| --- | --- |
| `model_path` | the `.ros2` file |
| `target_nodes` | the nodes to generate; all nodes when left out |
| `language` | `cpp` (default), `python` or `both` |
| `ros_distro` | the [target distribution](TargetDistributions.md), `auto` by default |
| `workspace_dir` | the folder that contains `src-gen/`; the package folder by default |

The code is written to `src-gen/<package>/`. Copy or link that folder into the `src/` of a ROS 2 workspace and build it with `colcon build`.

## What is generated

The golden test model of RosTooling has three nodes, an `io_node` with publishers, subscribers and parameters, a lifecycle `service_node` with a service server and client, and an `action_node`. Generated as C++ for one distribution, it gives:

```text
golden_pkg/
├── CMakeLists.txt
├── package.xml
├── include/golden_pkg/
│   ├── IoNodeWrapper.hpp        ROS 2 side: interfaces and parameters
│   ├── IoNodeAlgorithm.hpp      your logic, written once
│   ├── IoNodeNode.hpp           connects wrapper and logic
│   ├── ServiceNode….hpp         the same three files for every node
│   └── ActionNode….hpp
└── src/
    ├── IoNodeWrapper.cpp
    ├── IoNodeNode.cpp
    ├── IoNodeRunner.cpp         main(): starts and spins the node
    └── …                        the same files for every node
```

As Python, each node has three files in a package folder, plus the setup files of a Python package:

```text
golden_pkg/
├── package.xml
├── setup.py, setup.cfg
├── resource/golden_pkg
└── golden_pkg/
    ├── io_node_wrapper.py       ROS 2 side: interfaces and parameters
    ├── io_node_logic.py         your logic, written once
    └── io_node_runner.py        main(): starts and spins the node
```

The file names follow the model: the C++ files use the artifact name in camel case, the Python files use the node name. The executable has the name of the artifact, so the node of the example above starts with `ros2 run golden_pkg io_node`.

## Which files are yours

| File | Generator behaviour |
| --- | --- |
| `…Algorithm.hpp`, `…_logic.py` | Created once. **Never overwritten**, so your code is safe when you change the model and generate again. |
| wrapper, node, runner, `CMakeLists.txt`, `package.xml`, setup files | Regenerated every time. Do not edit them. |

Keep all domain logic in the logic file. When you add an interface to the model, the regenerated wrapper expects the matching methods in your logic class. To get them, move your logic file aside, generate again, and merge your code into the new file.

A node is implemented in one language. If you generate a node again in the other language, the generator removes the generated files of the first language for that node.

## The logic class

The logic class contains only your code and plain C++ or Python types. It has no ROS 2 node, executor or middleware code, so you can test it without ROS 2. The generator gives it one method for each interface of the node:

| Interface | In the logic class |
| --- | --- |
| subscriber `scan_in` | you override `on_scan_in_received(msg)`, called for each message |
| publisher `status_out` | you call `publish_status_out(msg)` |
| service server `reset` | you override `handle_reset(request, response)` |
| service client `enable_client` | you call `call_enable_client_sync(request, timeout)` or `call_enable_client_async(request, callback)` |
| action server `lookup` | you override `handle_goal_lookup`, `handle_cancel_lookup` and `execute_lookup` |
| action client `lookup_client` | you call `send_lookup_client_goal_async(goal, …)` and may set a timeout |

This is the start of the generated C++ logic class of `io_node`:

```cpp
class IoNodeAlgorithm {
public:
    IoNodeAlgorithm() = default;
    virtual ~IoNodeAlgorithm() = default;

    // 1. INBOUND SUBSCRIBER MESSAGE HANDLERS
    virtual void on_scan_in_received(const sensor_msgs::msg::LaserScan & msg) {
        (void)msg;
        // TODO: Process incoming message data
    }

    // 4. OUTBOUND PUBLISHER METHODS
    void publish_status_out(const std_msgs::msg::String & msg) {
        if (publish_status_out_fn_) publish_status_out_fn_(msg);
    }
    // (more methods follow)
};
```

### Services of the platform

The logic class also has the functions that most nodes need, without using ROS 2 types:

| Function | Purpose |
| --- | --- |
| `ok()`, `shutdown(reason)` | check whether the node is still running, and ask the process to stop |
| `now()`, `now_seconds()`, `sleep_for(…)` | time that follows the ROS clock, including simulation time and bag replay |
| `create_rate(hz)`, `create_timer(period, callback)` | loops and timers that follow the same clock |
| `log_debug`, `log_info`, `log_warn`, `log_error` | logging through the ROS 2 logger |
| `get_parameter`, `set_parameter` | read and change parameters |
| `validate_parameter(name, value)` | accept or reject a parameter change that arrives at run time |
| `on_parameter_changed(name, value)` | react to a parameter that has changed |

The logic class has no start-up method. The node hands the logic its platform functions, the timer factory first, when it starts, so a node that has to start work by itself, such as a publisher with a timer, creates its timer in `set_timer_factory`, as in the [publisher and subscriber tutorial](Example_PubSub.md#4-write-the-logic). A lifecycle node starts its work in `on_activate`.

In a unit test these functions are not connected to ROS 2. Calling `ok`, `shutdown`, `now` or `sleep_for` without a test replacement raises an error, so a test cannot silently run against the wrong clock.

### Lifecycle nodes

A node with `lifecycle: true` is generated as a lifecycle node. Its logic class has the methods `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown` and `on_error`, in which you start and stop what the node needs in each state.

### Parameters

Each parameter of the model is declared with its default value. A parameter that is changed while the node runs goes through `validate_parameter`, which can reject it, and then through `on_parameter_changed`.

## Components

The C++ nodes are generated as ROS 2 components. The node class is registered as `<package>::<Artifact>Node`, so it can run as a normal executable or loaded into a component container. A [system model](RosSystemModelDescription.md#processes) with `processes:` uses this to start several nodes in one process.

## Build files

`package.xml` lists the dependencies of the nodes, the message packages their interfaces use, and the packages that the model's [`dependencies:`](RosModelDescription.md#structure) name. `CMakeLists.txt` builds one library and one executable per node. The C++ standard and the CMake version depend on the [target distribution](TargetDistributions.md).
