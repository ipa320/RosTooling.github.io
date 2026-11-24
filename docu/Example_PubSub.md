# Tutorial: a publisher and a subscriber

In this tutorial you model two nodes, a publisher of a "Hello World" message and a subscriber that receives it, compose them in a system, let RosTooling generate the code, write the few lines of logic that are yours, and run the result. It takes about 30 minutes and needs a ROS 2 installation (the commands use Jazzy).

The nodes are invented for the tutorial. The models of an earlier version are in the repository [ros-model-examples](https://github.com/ipa-nhg/ros-model-examples/tree/main/pub_sub_ros2).

**What you need:** the [VS Code extension](../vscode/overview.md), ROS 2, and the previous tutorials for the syntax: [package models](LearnRosModels.md) and [system models](LearnRosSystemModels.md).

## 1. The two nodes

Create a folder for the tutorial and open it in VS Code. Create the files `publisher.ros2` and `subscriber.ros2`.

<!-- check: group=pubsub file=publisher.ros2 -->
```text
publisher:
  artifacts:
    pub_artifact:
      node: pub_node
      publishers:
        my_pub:
          type: "std_msgs/msg/String"
```

<!-- check: group=pubsub file=subscriber.ros2 -->
```text
subscriber:
  artifacts:
    sub_artifact:
      node: sub_node
      subscribers:
        my_sub:
          type: "std_msgs/msg/String"
```

The package `publisher` has an artifact `pub_artifact`, which will be the executable, with the node `pub_node` and a publisher `my_pub` of the type `std_msgs/msg/String`. The package `subscriber` is its counterpart.

::::{tab-set}
:::{tab-item} VS Code
Press `Ctrl+Space` after the colon of a line to let the editor list what can follow, for example `artifacts:` below the package name.
:::
:::{tab-item} Eclipse
Create a project with a reference to `de.fraunhofer.ipa.ros.communication.objects` and a folder `rosnodes`, and create the files there with **File → New → Other → File**. Eclipse converts the project to an Xtext project when you create a `.ros2` file.
:::
::::

## 2. The system

The system composes the nodes and connects the interface of one to the interface of the other. Create `my_system.rossystem`:

<!-- check: group=pubsub file=my_system.rossystem -->
```text
my_system:
  nodes:
    "publisher":
      from: "publisher.pub_node"
      interfaces:
        - "HelloWorldPublisher": pub-> "pub_artifact::my_pub"
    "subscriber":
      from: "subscriber.sub_node"
      interfaces:
        - "HelloWorldSubscriber": sub-> "sub_artifact::my_sub"
  connections:
    - ["HelloWorldPublisher", "HelloWorldSubscriber"]
```

The `from:` entries are written `<package>.<node>`, the interface references `<artifact>::<interface>`, and the connection uses the labels. Open the file in RosTooling Studio to see the system:

```{image} images/tut_pubsub_system.png
:alt: The publisher and subscriber system in RosTooling Studio
:width: 100%
```

## 3. Generate the code of the nodes

Right-click `publisher.ros2` and choose **Generate ROS 2 Node Wrappers**, then **Python**. Do the same for `subscriber.ros2`. The code is written to `src-gen/publisher/` and `src-gen/subscriber/`. For each node there are three files, see [ROS 2 code generation](Ros2CodeGeneration.md):

```text
src-gen/publisher/publisher/
├── pub_node_wrapper.py    the ROS 2 side, regenerated every time
├── pub_node_runner.py     main(), regenerated every time
└── pub_node_logic.py      your code, written once
```

## 4. Write the logic

Only the `*_logic.py` files are yours. In `sub_node_logic.py`, the method `on_my_sub_received` is called for every message. Replace `pass` with a log line:

```python
    def on_my_sub_received(self, msg) -> None:
        """Called when a new message is received on topic 'my_sub'."""
        self.log_info(f"I heard: {msg.data}")
```

The publisher needs something to trigger it. In `pub_node_logic.py`, the node hands the logic a factory for timers when it starts, in `set_timer_factory`. Create the timer there and publish in its callback:

```python
    def set_timer_factory(self, factory: Callable[[float, Callable[[], None]], Any]) -> None:
        """Inject the platform/ROS timer creation factory."""
        self._timer_factory = factory
        # The node injects the factory when it starts: this is where the timer begins.
        self._count = 0
        self._timer = self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        msg = String()
        msg.data = f"Hello World: {self._count}"
        self._count += 1
        self.publish_my_pub(msg)
        self.log_info(f"Publishing: {msg.data}")
```

`String` is already imported at the top of the file, because the model says that `my_pub` publishes a `std_msgs/msg/String`. The logic has no ROS 2 code in it: `create_timer`, `publish_my_pub` and `log_info` are provided by the generated wrapper. When you generate the nodes again, these files are not touched.

## 5. Generate the package of the system

Right-click `my_system.rossystem` and choose **Generate ROS2 Package**. The package `my_system` with the launch file is written to `src-gen/my_system/`. Its launch file starts both executables:

```python
  publisher = Node(
    package="publisher",
    executable="pub_artifact",
    output='screen',
    name="publisher",
    remappings=[
      ("my_pub", "HelloWorldPublisher")]
  )
  subscriber = Node(
    package="subscriber",
    executable="sub_artifact",
    output='screen',
    name="subscriber",
    remappings=[
      ("my_sub", "HelloWorldPublisher")]
  )
```

The connection is the remapping: both nodes use the label `HelloWorldPublisher` as the name of the topic, see [System generation](CodeGeneration.md#connections).

## 6. Build and run

Copy or link the three packages from `src-gen/` into the `src/` folder of a ROS 2 workspace and build:

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
colcon build --symlink-install
source install/setup.bash
ros2 launch my_system my_system.launch.py
```

The nodes start and the subscriber receives what the publisher sends:

```text
[sub_artifact-2] [INFO] [subscriber]: I heard: Hello World: 0
[pub_artifact-1] [INFO] [publisher]: Publishing: Hello World: 0
[pub_artifact-1] [INFO] [publisher]: Publishing: Hello World: 1
[sub_artifact-2] [INFO] [subscriber]: I heard: Hello World: 1
```

Change the model, for example rename the label of the connection, generate again and run again. Your logic stays, and the launch file and the wrappers follow the model.
