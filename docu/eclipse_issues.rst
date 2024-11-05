Eclipse "issues"
=================

The current version of the RosTooling is built on top of Eclipse. Some issues related to this IDE may appear during the execution of these tutorials.

Before you start editing the models, ensure that Eclipse is automatically building your workspace with every change you make. To do this, you must enable the "Build automatically" option under the "Project" menu.

The most common issue is problems with compiling the models due to not-found references.

The `.ros2` models have dependencies on `.ros`, and the `.rossystem` models depend on both of them. Sometimes these references are defined in a different project, which must be compiled in the correct order, and occasionally the automatic build fails to manage this correctly. In such cases, you can trigger a new build for the full workspace or just for the project with errors. This helps in the majority of cases. The option is available under **Project -> Clean...**.
