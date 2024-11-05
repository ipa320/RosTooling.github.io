Generate code from model description [|image1|]
===============================================

This tutorial requires a local ROS 2 installation and some experience
with ROS 2. It is based on the package
`rossdl <https://github.com/CoreSenseEU/rossdl>`__.

Installation [|image2|]
-----------------------

Firstly, you have to create your ROS2 workspace, clone and compile the
rossdl package:

::

   mkdir -p ros2/ws/src
   git clone https://github.com/CoreSenseEU/rossdl ros2/ws/src/rossdl
   source /opt/ros/humble/setup.bash
   cd ros2/ws
   rosdep install --from-path src/ -i -y
   colcon build
   source install/setup.bash

Code template [|image3|]
------------------------

The rossdl repository contains some examples. The use of the rossdl
package starts by creating a clean ROS package that contains a ros2
description model. Let’s use the basic template available on
`GitHub <https://github.com/ipa-nhg/rossdl/tree/BasicExampleNHG/rossdl_tests/basic_template>`__.

The template is a common ROS package where the CMakeLists includes the
generator command to be executed while building the package.

The line that enables the generator is the following one:

::

   rossdl_generate_code(
     "rosnodes/node.ros2"
     ${dependencies}
   )

Where the first argument is the relative path to the .ros2 model file
that contains the description of the ROS 2 node.

Imagine we have the description of an image filter node, which model
looks like this:

::

   basic_template:
     artifacts: 
       image_filter:
        node: image_filter
        publishers:
          image_out:
            type: "sensor_msgs/msg/Image"
          description_out:
            type: "std_msgs/msg/String"
        subscribers:
          image_in:
            type: "sensor_msgs/msg/Image"
          laser_in:
            type: "sensor_msgs/msg/LaserScan"

Apart from that the user must add the dependencies to the required
packages to both files, the package.xml and the CMakeLists. For this
concrete case, the only extra dependencies for the node are “std_msgs”
and “sensor_msgs”. This means the CMakeLists.txt looks like:

::

   cmake_minimum_required(VERSION 3.8)
   project(basic_template)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(rossdl_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclcpp_components REQUIRED)
   find_package(backward_ros REQUIRED)
   # Add here further dependencies
   find_package(std_msgs REQUIRED)
   find_package(sensor_msgs REQUIRED)

   set(dependencies
     rclcpp
     rclcpp_components
     backward_ros
     # Add here further dependencies
     std_msgs
     sensor_msgs
   )

   rossdl_generate_code(
     "rosnodes/node.ros2"
     ${dependencies}
   )

   ament_package()

While the package.xml contains the following text:

::

   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>basic_template</name>
     <version>0.0.0</version>
     <description>This is a basic template for rossdl</description>
     <maintainer email="fmrico@gmail.com">fmrico</maintainer>
     <maintainer email="nadia.hammoudeh.garcia@ipa.fraunhofer.de">nhg</maintainer>
     <license>Apache 2.0</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <depend>rclcpp</depend>
     <depend>rclcpp_components</depend>
     <depend>rossdl_cmake</depend>
     <depend>backward_ros</depend>
     <!--Add here further dependecies-->
     <depend>std_msgs</depend>
     <depend>sensor_msgs</depend>

     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>

By having this package structure and the ros2 file completed, you can
easily compile the package with the common colcon command:

::

   colcon build --symlink-install --packages-select rossdl_cmake basic_template

As result, automatically, in the build folder of your workspace, it will
appear the code corresponding headers for the new node. In this case in
**build/basic_template/include/basic_template**. For this basic example,
the code will contain:

::

   // generated from rossdl_cmake/resource/nodes.hpp.em
   // generated code does not contain a copyright notice

   #ifndef BASIC_TEMPLATE__NODES_HPP_
   #define BASIC_TEMPLATE__NODES_HPP_

   #include <string>
   #include <optional>
   #include <typeinfo>

   #include "sensor_msgs/msg/laser_scan.hpp"
   #include "std_msgs/msg/string.hpp"
   #include "sensor_msgs/msg/image.hpp"

   #include "rclcpp/rclcpp.hpp"

   namespace basic_template
   {

   class ImageFilterBase : public rclcpp::Node
   {
   public:
     explicit ImageFilterBase(const rclcpp::NodeOptions & options);

   protected:
     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_out_;
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_out_;
     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_in_;
     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_in_;

     virtual void image_in_callback(sensor_msgs::msg::Image::SharedPtr msg) = 0;
     virtual void laser_in_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) = 0;

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
       if (id == "image_out") {
           return image_out_;
       } 
       if (id == "description_out") {
           return description_out_;
       } 
       RCLCPP_ERROR(get_logger(), "Publisher [%s] not found", id.c_str());
       return nullptr;
     }

     typename rclcpp::SubscriptionBase::SharedPtr
     get_subscription_base(const std::string & id)
     {
       if (id == "image_in") {
           return image_in_;
       } 
       if (id == "laser_in") {
           return laser_in_;
       } 
       RCLCPP_ERROR(get_logger(), "Subscriber [%s] not found", id.c_str());
       return nullptr;
     }
   };


   }  // namespace basic_template

   #endif  // BASIC_TEMPLATE__NODES_HPP_

This is a base class that can be used as interface to create your node
under “ros2/ws/src/rossdl/basic_template/src”. This method apart from
saving effort by auto-generating code, drives the developer to create a
code that corresponds to the model representation.

An implementation of a Cpp package using this class is publicly
available under
`rossdl_tests/rossdl_simple_test/src/rossdl_simple_test/ImageFilter.cpp <https://github.com/CoreSenseEU/rossdl/blob/main/rossdl_tests/rossdl_simple_test/src/rossdl_simple_test/ImageFilter.cpp#L26>`__.

.. |image1| image:: images/Ros2_logo.png
.. |image2| image:: images/Ros2_logo.png
.. |image3| image:: images/Ros2_logo.png
