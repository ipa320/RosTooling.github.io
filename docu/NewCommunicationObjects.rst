Messages, services and actions generator
========================================

Autogeneration tools ![](images/Ros2_logo.png)
----------------------------------------------

For the autogeneration of model objects, we provide a bash script:

.. code-block:: bash

    source /your_ROS_workspace
    wget https://raw.githubusercontent.com/ipa320/RosCommonObjects/main/de.fraunhofer.ipa.ros.communication.objects/basic_msgs/generate_messages_model_helper.sh
    chmod +x generate_messages_model_helper.sh
    ./generate_messages_model_helper.sh ROS_PACKAGE_NAME > ROS_PACKAGE_NAME.ros

The file generated, independently of the method, will have a `.ros` extension and can be copied to the folder "basic_msgs" of the "de.fraunhofer.ipa.ros.communication.objects" project. Please send us a pull request to our `repository <https://github.com/ipa320/RosCommonObjects.git>`_ with your models to include them in the base dictionary automatically.

Alternatively, for cases where the message types are specific to a particular component (like `ur_msgs` for `ur_driver` or message types defined within the same repository as the node), we recommend following the same approach as ROS does: define the communication object models together with the node description. For our tooling, that means copying the file to the project created in the next step for your node description.

.. image:: images/Attention.png
   :alt: Attention
   :scale: 50%

The autogenerator script does not support actions.

ROS communication objects model language grammar
------------------------------------------------

To modify the ROS models (`.ros`) manually, the ROS tooling provides a customized editor, which should be the default option to open the `.ros` extension files. Otherwise, it can be selected manually by right-clicking on the *MyFile.ros* file and choosing _"Open with.."_ and selecting *"ROS Editor"*.

This editor contains an autocomplete function (by pressing **Ctrl+Space**) and will report any errors made during editing. The first step is to define a **PackageSet** (this corresponds to a metapackage for ROS; this definition is optional, and its name can be kept empty). Then, the ROS package which contains the messages has to be defined, and within it, the option "spec" has to be selected to write down the objects. In practice, this means that the initial `.ros` file that describes ROS objects looks like:

.. code-block:: yaml

    ros_package_name:
      msgs:
        msg_name
          message
            type name

The grammar supports three types of communication objects: messages, services, and actions. Consequently, each of these three types supports different specification types:

- ROS messages (msgs)

.. code-block:: yaml

    ros_package_name:
      msgs:
        msg_name
          message
            ElementType ElementName

For example:

.. code-block:: yaml

    std_msgs:
      msgs:
        ColorRGBA
          message
            float32 r
            float32 g
            float32 b
            float32 a

- ROS services (srvs)

.. code-block:: yaml

    ros_package_name:
      srvs:
        srv_name
          request
            ElementType ElementName
          response
            ElementType ElementName

For example:

.. code-block:: yaml

    std_srvs:
      srvs:
        SetBool
          request
            bool data
          response
            bool success
            string message

- ROS actions

.. code-block:: yaml

    ros_package_name:
      actions:
        action_name
          goal
            ElementType ElementName
          result
          feedback
            ElementType ElementName

For example:

.. code-block:: yaml

    control_msgs:
      actions:
        PointHead
          goal
            'geometry_msgs/msg/PointStamped'[] target
            'geometry_msgs/msg/Vector3'[] pointing_axis
            string pointing_frame
            'builtin_interfaces/msg/Duration'[] min_duration
            float64 max_velocity
          result
          feedback
            float64 pointing_angle_error

As in ROS, the allowed element types are:

- **Primitives:**
  - bool
  - int8
  - uint8
  - int16
  - uint16
  - int32
  - uint32
  - int64
  - uint64
  - float32
  - float64
  - string
  - time
  - Header

- **Relative reference to another object:**
  - `NameOftheObject` (if it is described within the same ROS package) -> e.g., **Point32**
  - `'ROSPackage_name/NameOftheObject'` (if it is described in another ROS package) -> e.g., **'geometry_msgs/Point32'**

- **Arrays of element types:**
  - `ElementType[]` -> e.g., **string[]** or **Point32[]** or **'geometry_msgs/Point32'[]**

Additionally, the definition of constants with their values is supported and follows a pattern very similar to ROS:

.. code-block:: yaml

    constanttype1 CONSTANTNAME1=constantvalue1

For example:

.. code-block:: yaml

    byte OK=0 byte WARN=1 byte ERROR=2 byte STALE=3

The following extract shows the ROS model description corresponding to the `nav_msgs <http://wiki.ros.org/nav_msgs>`_ package:

.. code-block:: yaml

    nav_msgs:
      msgs:
        Path
          message
            'std_msgs/msg/Header'[] header
            'geometry_msgs/msg/PoseStamped'[] poses
        OccupancyGrid
          message
            'std_msgs/msg/Header'[] header
            'nav_msgs/msg/MapMetaData'[] info
            int8[] data
        Odometry
          message
            'std_msgs/msg/Header'[] header
            string child_frame_id
            'geometry_msgs/msg/PoseWithCovariance'[] pose
            'geometry_msgs/msg/TwistWithCovariance'[] twist
        GridCells
          message
            'std_msgs/msg/Header'[] header
            float32 cell_width
            float32 cell_height
            'geometry_msgs/msg/Point'[] cells
        MapMetaData
          message
            'builtin_interfaces/msg/Time'[] map_load_time
            float32 resolution
            uint32 width
            uint32 height
            'geometry_msgs/msg/Pose'[] origin
      srvs:
        SetMap
          request
            'nav_msgs/msg/OccupancyGrid'[] map
            'geometry_msgs/msg/PoseWithCovarianceStamped'[] initial_pose
          response
            bool success
        LoadMap
          request
            string map_url
          response
            'nav_msgs/msg/OccupancyGrid'[] map
            uint8 result
        GetPlan
          request
            'geometry_msgs/msg/PoseStamped'[] start
            'geometry_msgs/msg/PoseStamped'[] goal
            float32 tolerance
          response
            'nav_msgs/msg/Path'[] plan
        GetMap
          request
          response
            'nav_msgs/msg/OccupancyGrid'[] map

Known issues
------------

.. image:: images/Attention.png
   :alt: Attention
   :scale: 50%

This model does not allow the creation of two specifications with the same name, even if they have different types. For example, the following ROS model is not allowed:

.. code-block:: yaml

    my_msgs:
      msgs:
       hello:
        message
         String data 
      srvs:
       hello
         request
         response
          String data

The reason is that when one of these objects has to be referenced during the definition of a node, it will be impossible for the model to distinguish which one is correct (both are defined as `my_msgs/Hello` and within the same model file). For these cases, we recommend splitting the objects into two different model files.

The repository `RosCommonObjects <https://github.com/ipa320/RosCommonObjects>`_ holds further examples.
