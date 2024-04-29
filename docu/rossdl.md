# Generate code from model description [![](images/Ros2_logo.png)]

This tutorial requires a local ROS 2 installation and some experience with ROS 2. It is based on the package [rossdl](https://github.com/CoreSenseEU/rossdl).

## Installation [![](images/Ros2_logo.png)]

Firstly, you have to create your ROS2 workspace, clone and compile the rossdl package:

```
mkdir -p ros2/ws/src
git clone https://github.com/CoreSenseEU/rossdl ros2/ws/src/rossdl
source /opt/ros/humble/setup.bash
cd ros2/ws
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```

## Code template [![](images/Ros2_logo.png)]


The rossdl repository contains a template example. It is available under **rossdl/rossdl_tests/basic_template**. For testing purposes, you can directly use this package, or copy and rename the folder and modify it.

The template is a common ROS package where the CMakeLists includes the generator command to be executed while building the package.

The line that enables the generator is the following one:

```
rossdl_generate_code(
  "rosnodes/node.ros2"
  ${dependencies}
)
```

Where the first argument is the relative path to the .ros2 model file that contains the description of the ROS 2 node.

For the previously shown example, the content of the file is:
```
basic_template:
  artifacts: 
    component:
     node: test
     publishers:
       string_pub:
         type: "std_msgs/msg/String"
```

Apart from that the user must add the dependencies to the required packages to both files, the package.xml and the CMakeLists. For this concrete case, the only extra dependency for the node is "std_msgs". This means the CMakeLists.txt looks like:

```
cmake_minimum_required(VERSION 3.8)
project(**PackageName**)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rossdl_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(backward_ros REQUIRED)
find_package(std_msgs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_components
  backward_ros
  std_msgs
)

rossdl_generate_code(
  "rosnodes/node.ros2"
  ${dependencies}
)

ament_package()
```

While the package.xml contains the following text:
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>**PackageName**</name>
  <version>0.0.0</version>
  <description>**My awesome package description**</description>
  <maintainer email=**Maintainer email**>**Maintainer name**</maintainer>
  <license>Apache 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>rossdl_cmake</depend>
  <depend>backward_ros</depend>
  <!--Add here further dependecies-->
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

By having this package structure and the ros2 file completed, you can easily compile the package with the common colcon command:
```
colcon build --symlink-install --packages-select rossdl_cmake basic_template
```

As result, automatically, in the build folder of your workspace, it will appear the code corresponding headers for the new node. In this case in **build/basic_template/ilnclude/basic_template**. For this basic example, the code will contain:

```
// generated from rossdl_cmake/resource/nodes.hpp.em
// generated code does not contain a copyright notice

#ifndef BASIC_TEMPLATE__NODES_HPP_
#define BASIC_TEMPLATE__NODES_HPP_

#include <string>
#include <optional>
#include <typeinfo>

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

namespace basic_template
{

class ComponentBase : public rclcpp::Node
{
public:
  explicit ComponentBase(const rclcpp::NodeOptions & options);

protected:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;


  template<typename T>
  typename rclcpp::Publisher<T>::SharedPtr
  get_publisher(const std::string & id)
  {
    auto ret_pub = std::dynamic_pointer_cast<typename rclcpp::Publisher<T>>(
      get_publisher_base(id));
    return ret_pub;
  }

  template<typename T>
  typename rclcpp::Subscription<T>::SharedPtr
  get_subscription(const std::string & id)
  {
    auto ret_sub = std::dynamic_pointer_cast<typename rclcpp::Subscription<T>>(
      get_subscription_base(id));
    return ret_sub;
  }

  typename rclcpp::PublisherBase::SharedPtr
  get_publisher_base(const std::string & id)
  {
    if (id == "string_pub") {
        return string_pub_;
    } 
    RCLCPP_ERROR(get_logger(), "Publisher [%s] not found", id.c_str());
    return nullptr;
  }

  typename rclcpp::SubscriptionBase::SharedPtr
  get_subscription_base(const std::string & id)
  {
    RCLCPP_ERROR(get_logger(), "Subscriber [%s] not found", id.c_str());
    return nullptr;
  }
};


}  // namespace basic_template

```

This is a base class whose classes can be used as interfaces to create your node under "ros2/ws/src/rossdl/basic_template/src". This method apart of saving effort by auto-generating code, drive the developer to create a code that correspond to the model representation.