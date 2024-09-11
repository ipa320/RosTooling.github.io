Systems code generator review
=============================

For the System model, the generation of code is automatic. This means if your model is correct and follows the connection rules, Xtext will automatically generate a ROS package into the *src-gen* folder, which has the typical structure of a Python package meant to hold launch file artifacts. More concretely:

- **README file**: This file will always be generated, even in cases of a system that only points to an existing launch file. The README contains information about the design of the system, as well as how to install the dependencies, set up the workspace to hold the created implementation, and start the generated artifact.
  
- **setup.py script**: This is the standard script required for a Python package, specifically created for the structure of this package.
  
- **package.xml file**: A template for the `package.xml` of the package, which contains the specification of all the required dependencies to start the package. The user should complete the type of license, maintainer, owner, and description of the package.
  
- **CMakeLists.txt**: A template of the `package.xml`, which includes the required tags to install the newly created files.
  
- **launch/NameOfTheSystem.launch.py script**: This is the core of the generator. It is a **ready-to-execute** file based on the `.rossystem` description:
  - Add includes for all the defined nodes as components
  - Define and set the values of the parameters. For packages containing more than 5 parameters, the `.yaml` file under the "config" folder will be loaded and launched.
  - Remap topics to force connections (only working for nodes that are part of the system as components, not for subsystems). See `known issues <RosTooling_issues.md#code-generators>`_.
  - Include other launch files from subsystems.

- **resource/NameOfTheSystem.puml**: `An example of a PlantUML implementation of the described system <SystemModelsVisualization.md#open-the-plantuml-auto-generated-description>`_.

- **config/NameOfTheSystem.yaml**: This file contains the configuration of the parameters. It will only be used if there are 5 or more parameters.

With the current implementation of RosTooling, adding new code generators to the `.rossystem` representation is very easy. Please report on our survey's open-ended questions regarding your wishes, such as the generation of install scripts, Docker container configuration, or ROS1-ROS2 bridges.
