# Extract component models using introspection at runtime

This tutorial requires a local ROS installation.

For the auto-generation of models another strategy, parallel to [static code analysis techniques](StaticCodeAnalyis.md) is the introspection of a running system.

The tool [ros2model](https://github.com/ipa320/ros2model) was created for that purpose. It can be installed as every common ROS 2 package by:

```
mkdir -p ros2/ws/src
git clone https://github.com/ipa320/ros2model ros2/ws/src/ros2model
source /opt/ros/humble/setup.bash
cd ros2/ws
rosdep install --from-path src/ -i -y
colcon build
source install/setup.bash
```

Then you can start the nodes you want to analyze. For example the common turtlesim tutorial:
```
ros2 launch turtlesim multisim.launch.py
```

And then just call the extractor to generate the models:
```
ros2 model node -o turtlesim/nodes
```
This command will create automatically under the selected folder (in this case turtlesim/nodes) the .ros2 files corresponding to all the running nodes. 


You can copy the files to the ROS project in Eclipse to verify the models.

Please note that some information can be wrong as from the runtime system we can not get the filesystem information like the name of the package and the name of the artifacts. This information must be reviewed by the user.