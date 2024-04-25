# Eclipse "issues"

The current version of the RosTooling is built on top of Eclipse. Some issues related to this IDE can appear during the execution of these tutorials.

Before you start editing the models you must be sure that Eclipse is building automatically your workspace by every change you make. This means you must have enabled the option "Build automatically" under the menu "Project".

The most common issue is problems in compiling the models because of not-found references.

The .ros2 models have dependencies to .ros, and the .rossystem models to both of them and sometimes these references are defined on a different project that must be compiled in the correct order and sometimes the automatic build can't deal with it. For those cases, you can trigger a new build, for the full Workspace or only for the project with errors. This helps in the majority of the cases. This option is available under "Poroect" -> "Clean..".