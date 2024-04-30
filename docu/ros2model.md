# Extract component models using introspection at runtime [![](images/Ros2_logo.png)]

This tutorial requires a local ROS installation.

For the auto-generation of models another strategy, parallel to [static code analysis techniques](StaticCodeAnalyis.md) is the introspection of a running system.

The tool [ros2model](https://github.com/ipa320/ros2model) was created for that purpose. It can be installed as every common ROS 2 package by:

1. clone this repository into the source folder in your workspace, such as:
   ```
   ws/src/ros2model
   ```
2. Source ROS workspace
   ```
   source /opt/ros/humble/setup.bash
   ```
3. Back to the folder "ws", create and active venv
   ```
   python3 -m venv venv --system-site-packages --symlinks
   source venv/bin/activate
   ```
4. Install poetry and install dependencies
   run
   ```
   pip install poetry
   poetry install -C src/ros2model/
   ```
5. compile it as ROS package
   ```
   python -m colcon build --packages-select ros2model --symlink-install
   ```
   ```
   source install/local_setup.bash
   ```
6. config python path
   in this case we are using python3.10 in venv
   ```
   export PYTHONPATH=$PYTHONPATH:$(pwd)/venv/lib/python3.10/site-packages/
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

Many of the models you will see during the rest of the tutorials were auto-generated using this method.