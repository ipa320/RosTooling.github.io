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
- Ros packages (component) models: they represent the filesystem level of ROS packages, as well as the computational graph. In other words, the package description, the implemented artifacts and the communication ports the nodes offer. They have two different implementations, for ROS (1) (.ros1 extension) or for ROS 2 (.ros2 extension files.) 
- Ros System models: they represent the ROS nodes as components present in a system and the connections among them. The file extension of this type of model is .rossystem.

#### Components (.ros1, .ros2)

- [How to describe ROS nodes using models](docu/RosModelDescription.md)
- [Parameters API reference and examples](docu/ParametersAPI.md)

#### Communication objects (.ros)
- [Messages, services and actions generator](docu/NewCommunicationObjects.md)

#### Systems (.rossystem)
- [How to describe ROS systems using model](docu/RosSystemModelDescription.md)

### Tutorials

#### Components

- Review models and use of textual editor
- Create your own model
- Generate code from model description
- Extract models using static code analysis
- Extract component models using introspection at runtime

#### Systems

- Review models and use of textual editor
- [Create your own model](docu/Example_PubSub.md)
- Models visualization
- Code generator review
- Real use cases execution
- Hello world example
- [Turtlesim](docu/Example_Turtlesim.md)
- QR code detector
- Mobile base

### Links and further examples

