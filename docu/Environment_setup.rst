Setup the environment and start the Eclipse application
=======================================================

Switch to the ROS Developer perspective
---------------------------------------

Go to Menu *Window* -> *Perspective* -> *Open Perspective* -> *Other...* -> *ROS developer*. Your application toolbar will be automatically configured.

Import the common communication objects project
-----------------------------------------------

Import the project located under the "ROSCommonObjects" folder of this repository to the workbench of your application:

.. code-block:: none

   de.fraunhofer.ipa.ros.communication.objects

If you have an internet connection, you can use the button that clones the objects from GitHub into your workspace and imports them automatically:

.. image:: images/clone_and_import.png

.. image:: images/Attention.png

This button will only work on Linux machines. Alternatively, you can manually clone the `repository <https://github.com/ipa320/RosCommonObjects>`_ and import the project by navigating to *File* -> *Import* -> *General* -> *Existing Projects into Workspace*. 
Then select the container folder of the cloned repository.

Import the catalog
------------------

You can also download and import a pre-existing catalog of examples. The catalog is publicly available on GitHub: `ipa-nhg/RosModelsCatalog <https://github.com/ipa-nhg/RosModelsCatalog>`_.

Please clone the repository and import it into your Eclipse workspace. This can be easily done by navigating to *File* -> *Import* -> *General* -> *Existing Projects into Workspace*. 
Then select the container folder of the cloned repository.

.. image:: images/01_mobile_base_b.gif
