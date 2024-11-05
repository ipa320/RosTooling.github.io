RosTooling
==========

Welcome to the RosTooling documentation page! :)

The RosTooling aims to bridge the advantages of ROS with those of model-driven software development. At its core are a set of models that describe "every day" code developed by ROS developers, both at the component and system levels. These models are supported with different types of tools including model extractors from code, validators, and code generators.

We invite you to take a look at the tutorials to learn more about this tooling and its uses.

The `source code <https://github.com/ipa320/RosTooling>`_ of the entire implementation of the tooling is completely open-source. Contributions, feedback, and suggestions are always welcome.

Installation
------------

- `From Release <docu/Installation.rst#option-1-using-the-release-version-recommended>`_
- `Setup <docu/Environment_setup.rst#1-switch-to-the-ros-developer-perspective>`_

Models Review
-------------

The RosTooling divides the ROS concepts into three layers of models:

- **Ros objects models**: They represent the interface types. In ROS world, these are the message, services, and action types. This file has the `.ros` extension and allows a YAML format similar to the ROS `.msgs`, `.srvs`, and `.action` files.
- **Ros packages (component) models**: They represent the filesystem level of ROS packages, as well as the computational graph. In other words, the package description, the implemented artifacts, and the communication ports the nodes offer. They have two different implementations, for ROS (1) (`.ros1` extension) or for ROS 2 (`.ros2` extension files.)
- **Ros System models**: They represent the ROS nodes as components present in a system and the connections among them. The file extension of this type of model is `.rossystem`.

Components (.ros1, .ros2)
~~~~~~~~~~~~~~~~~~~~~~~~~

- `How to describe ROS nodes using models <docu/RosModelDescription.rst>`_
- `Parameters API reference and examples <docu/ParametersAPI.rst>`_

Communication objects (.ros)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- `Messages, services and actions generator <docu/NewCommunicationObjects.rst>`_

Systems (.rossystem)
~~~~~~~~~~~~~~~~~~~~

- `How to describe ROS systems using model <docu/RosSystemModelDescription.rst>`_

Tutorials
---------

.. image:: docu/images/Attention.png
   :alt: Attention

All tutorials were created on Linux. In addition, some of them require a local installation of ROS 2, which are marked with the logo:

.. image:: docu/images/Ros2_logo_mini.png
   :alt: ROS 2 logo

The modeling part should run on other OS as long as Eclipse is properly installed and also the required Java packages (JRE and JDK). The current version of the RosTooling requires Java 19 or higher.

In case you have already installed the RosTooling, we recommend always pulling the latest version. Go to "Help" -> "Check for Updates", and in case a new version is available, it will be proposed to be updated.

If you encounter issues during the execution of these tutorials, please report them under the following `GitHub Issues <https://github.com/ipa320/RosTooling.github.io/issues/new?assignees=&labels=&projects=&template=bug_report.md&title=>`_. If you don't have a GitHub account, please just send an email to nhg@ipa.fhg.de.

Also, we would be very thankful if once you complete the tutorials, you take 15 minutes to fill out our survey. This will help us to improve our solution: `Survey link <https://forms.office.com/e/2V5pPwcY7V>`_.

Many thanks for your help :)

Components
~~~~~~~~~~

- `Review models and use of textual editor <docu/LearnRosModels.rst>`_
- (Optional) `Create your own model <docu/CreateYourModel.rst>`_
- `Generate code from model description <docu/rossdl.rst>`_ .. image:: docu/images/Ros2_logo_mini.png
- `Extract models using static code analysis <docu/StaticCodeAnalyis.rst>`_ .. image:: docu/images/docker_logo.png
- `Extract component models using introspection at runtime <docu/ros2model.rst>`_ .. image:: docu/images/Ros2_logo_mini.png

Systems
~~~~~~~

- `Review models and use of textual editor <docu/LearnRosSystemModels.rst>`_
- `Other ways to define a system <docu/LearnRosSystemModels2.rst>`_
- `Code generator review <docu/CodeGeneration.rst>`_
- `System Models visualization <docu/SystemModelsVisualization.rst>`_
- Real use cases execution:

  - `Turtlesim <docu/Example_Turtlesim.rst>`_

  - `Beginner - Mobile base on simulation <docu/MobileBase_beginner.rst>`_

  - `TBD - Advanced - Manipulation <docu/Manipulation_advanced.rst>`_

.. |ros2| image:: docu/images/Ros2_logo_mini.png
   :alt: ROS2 logo
   :scale: 100%
   :background: white

Extra material
--------------

- `Eclipse known issues <docu/eclipse_issues.rst>`_
- `RosTooling Known issues <docu/RosTooling_issues.rst>`_
- `Create your own model from nodes <docu/Example_PubSub.rst>`_
