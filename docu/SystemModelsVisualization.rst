System visualization
====================

The RosTooling offers two visualization options: \* By the generation of
`PlantUML <https://plantuml.com/>`__ diagrams \* By using a
`Sirius <https://eclipse.dev/sirius/>`__ implementation as Eclipse
plugin.

PlantUML compiler
-----------------

Embedded the viewer on the Eclipse environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Eclipse Marketplace includes a PlantUML plugin. To install it go to
Eclipse to Help -> Eclipse Marketplace.. and search for “PlantUML”.
Select the Plugin and let Eclipse install it.

.. figure:: images/PlantUMLInstaller.gif
   :alt: alt text

   alt text

Once the installation is completed, Eclipse must be restarted.

To open the visualizer in Eclipse you can easily go to “Window”-> “Show
view” -> Other and select “PlantUML”.

.. figure:: images/PlantUMLView.gif
   :alt: alt text

   alt text

Open the PlantUML auto-generated description
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For this example we will use a template model available under the
default project “de.fraunhofer.ipa.ros.communication.objects”. See
instructions under `setup <Environment_setup.rst>`__. If the automatic
clone doesn’t work for you, please clone the repository
`RosCommonObjects <https://github.com/ipa320/RosCommonObjects>`__
manually and import it to your Eclipse workspace.

The example we will show is under
“BasicSpecs/Systems/ros_system_template.rossystem”.

By default, the RosTooling will automatically generate a PlantUML file
description of every valid RosSystem model. It will be held in the
“src-gen” folder of the corresponding ROS package. The folder
“resources” contains a file with the .puml extension. By opening it with
a text editor and having the PlantUML visualizer open the diagram will
appear.

.. figure:: images/PlantUMLViewSystemExample.png
   :alt: alt text

   alt text

Alternatively, you can copy the content of the generated file to an
online PlantUML editor tool like
`PlantText <https://www.planttext.com/>`__.

Known Issues
^^^^^^^^^^^^

|image1| Unfortunately, PlantUML, specifically the template here
generated is not supporting large systems. They show cut. Also sometimes
the generated file has issues because of duplicated ports. We are
considering the reimplementation of the template.

Sirius visualizer
-----------------

Install Sirius on Eclipse
~~~~~~~~~~~~~~~~~~~~~~~~~

Sirius can be easily installed using the Eclipse Marketplace. Go in
Eclipse to Help -> Eclipse Marketplace.. and search for “Sirius”. Select
the Plugin and let Eclipse install it.

.. figure:: images/SiriusInstaller.gif
   :alt: alt text

   alt text

Once the installation is completed, Eclipse must be restarted.

Create a representation view of your system
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For this example we will use a template model available under the
default project “de.fraunhofer.ipa.ros.communication.objects”. See
instructions under `setup <Environment_setup.rst>`__. If the automatic
clone doesn’t work for you, please clone the repository
`RosCommonObjects <https://github.com/ipa320/RosCommonObjects>`__
manually and import it to your Eclipse workspace.

The example we will show is under
“BasicSpecs/Systems/ros_system_template.rossystem”.

A representation file is a file in which Sirius stores all information
related to which representations you created, what appears on them, the
positions and colors of the elements, etc. These files have a .aird
extension (typically representations.aird). Representation files
reference the semantic model(s) for which they contain representations,
but your semantic models themselves are kept unaware (and unpolluted) of
any Sirius-specific data.

To create this file you can go to File -> New -> Other and then navigate
or search for “Representations File”.

We recommend selecting the initialization method “Initialization from a
semantic resource” and using the helper to navigate and select the
.rossystem file you would like to open, for example
“ros_system_template.rossystem”.

.. figure:: images/SiriusCreateRepresentationFile.gif
   :alt: alt text

   alt text

While opening the representation file with an “Aird Editor”, which
should be the default option.

In the menu Representations (it should be the menu in the middle) choose
the “RosSystem” option and press “New..” then choose the RosSystem
entity of your “\*.rossystem” model and press finish.

.. figure:: images/SiriusOpenSystemView.gif
   :alt: alt text

   alt text

Then you can use the mouse to move labels and components, interfaces or
labels to the desired position.

.. figure:: images/SiriusView.png
   :alt: alt text

   alt text

For complex systems, a nice feature of Sirius is the option to hide some
elements. For that purpose, you can use the tools panel or make a right
click in the Representation view and select “Show/Hide”. It also allows
to generate a JPG image of the system view.

.. figure:: images/SiriusFeatures.gif
   :alt: alt text

   alt text

.. |image1| image:: images/Attention.png
