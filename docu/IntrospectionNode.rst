Create a ROS model from a deployed robot using our introspection at runtime tool (for ROS 2 systems)
===================================================================================================

Please ensure that the tool is installed and your workspace is set up. See the `installation guide <../README.rst>`_ for further details.

The tools documented here were conceived as a simple way to obtain models of systems that are already developed during their execution. This series of scripts uses the popular ROS `rosgraph` library to obtain a list of the interfaces present in the system at runtime.

You can install the tools directly in your workspace using the following command:

.. code-block:: bash

    cd YourRos2WS/src
    git clone git@github.com:ipa-nhg/ros2model.git
    cd YourRos2WS
    colcon build

To ask the monitoring module to capture all the nodes running on the system, use the following command:

.. code-block:: bash

    ros2 model running_node -ga -d ~/PathToModelsFolderOutput

The folder **PathToModelsFolderOutput** will contain all the model files.

For a single node, use the following command:

.. code-block:: bash

    ros2 model running_node [-o Outputfile] <node-name>

For further information, please check the `ros2model <https://github.com/ipa-cmh/ros2model>`_ repository.

The models you will see during the rest of the tutorials were mostly auto-generated using this method.
