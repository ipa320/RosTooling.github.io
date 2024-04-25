## Systems code generator review

For the System model the generation of code is automatic, that means if your model is correct and does not infringe the connection rules (like join interfaces with mismatched types), Xtext will automatically generate into the *src-gen* folder a ROS package which has the typical strcuture of a python package meant to hold launch file artifacts. More concreteally:

* README file: this file will always be genarted even for the cases of a system that only points to an existing launch file. The README contains information about the design of the system. But also how to install the dependencies, setup the workspace to hold the created implementation and how to start the generated artifact.
* setup.py script: it is just the standard script required by a Python package which is specifically created for the strcuture of this package.
* package.xml file: a template for the package.xml of the package which contains the specification of all the required dependencies to start the package. The user should complete the type of the license, the maintainer and owner and the description of the package.
* CMakeLists.txt: a template of the package.xml which includes the requires tags to install the new created files.
* launch/NameOfTheSystem.launch.py script: This is the core of the generator. This is a ready-to-execute file which based on the .rossystem description:
  * Add includes to all the defined nodes as components
  * Define and set the value of the parameters. For packages containing more than 5 parameters an extra .yaml file will be creted and launched from this launch file.
  * Remap topics to force connections. This is only working for nodes that are part of the system as components, not for subsystems. See [known issues](RosTooling_issues.md#code-generators)
  * Include other launch files from subsystems.
* resource/NameOfTheSystem.puml: And example of a PlantUML implementation of the described system. This visualization has currently some [problems for large systems](SystemModelsVisualization.md#known-issues)

With the current implementation of the RosTooling add new code generators to the .rossystem representation is very easy, please report on our survey on the open-ended questions your wishes.
For example the generation of install scripts and/or Docker conatiners configuration is posible. Even the autogeneration of ROS1-ROS2 bridges is part of the roadmap.