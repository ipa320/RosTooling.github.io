# Target distributions

The code generators produce code for one ROS 2 distribution at a time. The generated C++ and Python code, the build files and the launch files are the same in structure for every distribution, and differ only where the distributions differ, for example in an API that changed or a minimum CMake version.

## Supported distributions

| Distribution | Release type | C++ standard | CMake minimum |
| --- | --- | --- | --- |
| Humble | LTS | C++17 | 3.8 |
| Jazzy | LTS | C++20 | 3.8 |
| Kilted | non-LTS | C++20 | 3.20 |
| Lyrical | LTS | C++20 | 3.20 |
| Rolling | rolling | C++20 | 3.20 |

Two more details depend on the distribution:

| Detail | Humble | Jazzy and later |
| --- | --- | --- |
| QoS of service servers and clients | `rmw_qos_profile_services_default` | `rclcpp::ServicesQoS()` |
| Loop rates (`create_rate`) in C++ | `Clock::sleep_until` | `rclcpp::Rate` with the node clock |

Generated packages do not use `ament_target_dependencies`, which Kilted deprecated and Lyrical and Rolling removed. They link with `target_link_libraries(… ${pkg_TARGETS})`, which works on all supported distributions.

## Choosing the distribution

If you do not choose, the generator uses `auto`:

1. the distribution in the environment variable `ROS_DISTRO`, which is set when you source a ROS 2 installation, or
2. the newest LTS distribution, currently **Lyrical**, when `ROS_DISTRO` is not set.

If `ROS_DISTRO` names a distribution that is not supported, the generator stops and asks you to name the target explicitly.

- **In VS Code**, the generate commands always use `auto`. To generate for Humble, start VS Code from a shell in which Humble is sourced, or `ROS_DISTRO=humble` is set.
- **With an AI agent**, pass the distribution as `ros_distro` to `generate_ros2_wrappers`, for example `humble`. An unknown name is answered with the list of the supported names.

The tool `list_ros2_distros` returns the supported distributions, the default, and what `auto` resolves to in the current environment. The generator reports the distribution it used with every result.

## Tested on every distribution

Every distribution of the table is tested in the official `ros:<distro>` image. The test builds the generated packages with deprecation warnings treated as errors, starts every executable and launches the generated system packages. A distribution that fails is visible on the status badges of the [RosTooling repository](https://github.com/ipa-esa/RosTooling). Rolling is allowed to fail, because it can break at any time.
