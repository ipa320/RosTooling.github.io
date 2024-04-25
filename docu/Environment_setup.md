# Setup the environment and start the eclipse application

<!-- ### 0: Start the ROS tooling application

:bangbang::bangbang: This is only needed if you installed the toolig from [source](#option-2-using-the-eclipse-installer---source-installation-ros-tooling-developers), if you installed the release version please continue directly with the [step 1](#1-switch-to-the-ros-developer-perspective)

select de.fraunhofer.ipa.ros.plugin and press the button *Run*

![alt text](images/run_ros_tooling.png) -->

### 1: Switch to the ROS Developer perspective

Go to Menu Window -> Perspective -> Open Perspective -> Other... -> ROS developer. Your application toolbar will be automatically configured.

### 2: Import the common communication objects project

import the project located under the "ROSCommonObjects" folder of this repository to the workbench of your application:
```
de.fraunhofer.ipa.ros.communication.objects
```

If you have internet a button can clone from GitHub the objects to your workspace and import them automatically:

![alt text](images/clone_and_import.png)

### 3: Import the catalog

Also you can download and import a pre-existing catalog of examples. The catalog is publicly available on GitHub [ipa-nhg/RosModelsCatalog](https://github.com/ipa-nhg/RosModelsCatalog).

Please clone the repository and import it to your Eclipse workspace. This can be easily done by File -> Import -> General -> Existing Projects into Workspace. Then select the container folder of the cloned repository.

![alt text](images/01_mobile_base_b.gif)