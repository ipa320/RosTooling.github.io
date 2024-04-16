## RosTooling

Welcome to the RosTooling documentation page! :)

The RosTooling aims to bridge the advantages of ROS with those of model-driven software development. At its core are a set of models that describe "every day" code developed by ROS developers, both at the component and system levels. These models are supported with different types of tools including model extractors from code, validators and code generators.

We invite you to take a look at the tutorials to learn more about this tooling and its uses.

The [source code](https://github.com/ipa320/RosTooling) of the entire implementation of the tooling is completely open-source, contributions, feedback and suggestions are always welcome. 

### Installation

- [From Release](docu/Installation.md#option-1-using-the-release-version-recommended)
- [Setup](docu/Environment_setup.md#1-switch-to-the-ros-developer-perspective)

### Models Review

The RosTooling divides the ROS concepts into three layers of models:

- Ros objects models: they represent the interface types. In ROS world the message, services and action types. This file has the .ros extension and allows a yaml format similar to the ROS .msgs, .srvs and .action files.
- Ros packages (component) models:
- Ros System models.

#### Components (.ros1, .ros2)

#### Communication objects (.ros)

#### Systems (.rossystem)

### Tutorials

#### Components

##### Review models and use of textual editor

##### Create your own model

##### Extract models using static code analysis

##### Extract components models using introspection at runtime

#### Systems

##### Review models and use of textual editor

##### Create your own model

##### Models visualization

##### Code generator review

#### Real use cases execution

##### Hello world example

##### QR code detector

##### Mobile base


### Links and further examples

