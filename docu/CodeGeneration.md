## Systems code generator review

For the System model, the generation of code is automatic, which means if your model is correct and follows the connection rules, Xtext will automatically generate into the *src-gen* folder a ROS package which has the typical structure of a python package meant to hold launch file artifacts. More concretely:

* README file: this file will always be generated even in the cases of a system that only points to an existing launch file. The README contains information about the design of the system. But also how to install the dependencies, set up the workspace to hold the created implementation and start the generated artifact.
* setup.py script: it is just the standard script required by a Python package which is specifically created for the structure of this package.
* package.xml file: a template for the package.xml of the package which contains the specification of all the required dependencies to start the package. The user should complete the type of the license, the maintainer and owner and the description of the package.
* CMakeLists.txt: a template of the package.xml which includes the required tags to install the newly created files.
* launch/NameOfTheSystem.launch.py script: This is the core of the generator. This is a **ready-to-execute** file that based on the .rossystem description:
  * Add includes to all the defined nodes as components
  * Define and set the value of the parameters. For packages containing more than 5 parameters an extra .yaml file will be created and launched from this launch file.
  * Remap topics to force connections. This is only working for nodes that are part of the system as components, not for subsystems. See [known issues](RosTooling_issues.md#code-generators)
  * Include other launch files from subsystems.
* resource/NameOfTheSystem.puml: [An example of a PlantUML implementation of the described system](SystemModelsVisualization.md#open-the-plantuml-auto-generated-description)

With the current implementation of RosTooling adding new code generators to the .rossystem representation is very easy, please report on our survey on the open-ended questions your wishes like for example, the generation of install scripts and/or Docker container configuration or ROS1-ROS2 bridges.