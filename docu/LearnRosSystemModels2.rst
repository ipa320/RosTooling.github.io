Other ways to define a system
=============================

The `previous tutorial <LearnRosSystemModels.rst>`_ shows the description of typical systems created by composing nodes. With RosTooling, it is also possible to compose subsystems. 
Again, you can refer to the `RosSystem description <RosSystemModelDescription.rst>`_ as a reference.

Subsystems from model
----------------------

Under "de.fraunhofer.ipa.ros.communication.objects/BasicSpecs/Systems", you can find some other examples of subsystems. To compose them, let's imagine we have the file created within the `previous tutorial <LearnRosSystemModels.rst>`_, we can easily add the subsystems definition:

.. code-block:: yaml

    my_awesome_system:
      subSystems:
        my_subsystem
      nodes:
        node1:
          from: "my_awesome_pkg.awesome_node"
          namespace: "my_ns"
          interfaces:
            - my_pub: pub-> "awesome::awesome_pub"
        node2:
          from: "my_awesome2_pkg.awesome2_node"
          interfaces:
            - my_sub: sub-> "awesome2::awesome2_sub"
          parameters:
            - array_example: "awesome::awesome_array_param"
              value: ["hello", "hallo"]
            - bool_example: "awesome::awesome_bool_param"
              value: true
            - double_example: "awesome::awesome_double_param"
              value: 1.2
            - integer_example : "awesome::awesome_integer_param"
              value: 2
            - string_param : "awesome::awesome_string_param"
              value: "hallo"
            - struct_param: "awesome::awesome_struct_param"
              value: [
                first: 2
                second: true]
      connections:
        - [ my_pub, my_sub ]

.. image:: images/06_learn_rossystemmodels.gif
   :alt: Subsystems composition

The code generator will update not only the launch file but also the `package.xml` to add the dependency to the package that contains the subsystem definition.

Subsystems from launch files (existing ROS systems)
---------------------------------------------------

As RosTooling aims to support ROS developers in their typical workflow, it also allows the use of systems where a launch file already exists. It does not re-generate or duplicate code that already exists. This is the purpose of the "fromFile" attribute.

.. image:: images/07_learn_rossystemmodels.gif
   :alt: Subsystems from launch files

The compiler expects the package name followed by the relative path to the file, including the file extension. An example looks like this:

.. code-block:: yaml

    my_awesome_system:
      fromFile: "pkg_name/launch/my_file.launch.py"
      nodes:
        node1:
          from: "my_awesome_pkg.awesome_node"
          namespace: "my_ns"
          interfaces:
            - my_pub: pub-> "awesome::awesome_pub"
        node2:
          from: "my_awesome2_pkg.awesome2_node"
          interfaces:
            - my_sub: sub-> "awesome2::awesome2_sub"
          parameters:
            - array_example: "awesome::awesome_array_param"
              value: ["hello", "hallo"]
            - bool_example: "awesome::awesome_bool_param"
              value: true
            - double_example: "awesome::awesome_double_param"
              value: 1.2
            - integer_example : "awesome::awesome_integer_param"
              value: 2
            - string_param : "awesome::awesome_string_param"
              value: "hallo"
            - struct_param: "awesome::awesome_struct_param"
              value: [
                first: 2
                second: true]
      connections:
        - [ my_pub, my_sub ]

The code generator will not create the launch file in these cases, but it will update the README file with instructions on how to install and start the defined file.

It is strongly recommended to provide information about the components and interfaces within the system. This will help with the documentation and be considered in the system's visualization.
