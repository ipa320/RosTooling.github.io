Parameters API
==============

The types of parameters supported by the ROS tooling are the following:

- Boolean (true or false)
- Integer
- Double
- String
- Base64
- List
- Array
- Struct

Their description is allowed at both the ROS and ROSSystem model levels.

For the complete definition of parameters, the user has to open the ROS model editor, where the language format is the following:

.. code-block:: none

   **parameters:**
     ParameterName:
       **type:** ParameterType
       **value:** ParameterValue

For example (for a ``.ros2`` file):

.. code-block:: yaml

   test_parameters:
     artifacts: 
       test_parameters:
         node: params_example 
         parameters:
           string_test:
             type: String
           bool_test:
             type: Boolean
           array_test:
             type: Array [String]
           base64_test:
             type: Base64
           double_test:
             type: Double
           integer_test:
             type: Integer
           list_test:
             type: List [Integer,Integer,String]
           array_test:
             type: Array [String]
           struct_test:
             type: Struct [hello Integer, what String]

These parameters can be re-set at the ROSSystem level (for ROS developers, this means setting a new parameter value within a node included in a ROS launch file). 
For the tooling, the format is the following:

.. code-block:: none

   - ParameterName : ParameterReferenceInRos2File
     **value:** ParameterValue

Continuing with the previous example, the parameters redefinition looks like this:

.. code-block:: yaml

   test:
     nodes:
       params_node:
         from: "test_parameters.params_example"
         parameters:
           - test_s : "test_parameters::string_test"
             value: "hello"
           - test_b : "test_parameters::bool_test"
             value: true
           - test_d : "test_parameters::double_test"
             value: 1.1
           - test_i : "test_parameters::integer_test"
             value: 1
           - test_l: "test_parameters::list_test"
             value: [1,1,"hello"]
           - test_a:  "test_parameters::array_test"
             value: ["hello", "hola", "hallo"]
           - test_st: "test_parameters::struct_test"
             value: [
               hello: 1 
               what: "test"]

The model definition of parameters is also considered for the auto-generation of launch files.
Additionally, due to the complexity of the format, the ``.ros1``, ``.ros2``, and ``.rossystem`` language validators contain rules to ensure that the value given to the parameter has the correct type. 
These validators also provide helpful messages, and, along with the auto-complete function (**Ctrl+Space**), they assist the user in creating parameters.
