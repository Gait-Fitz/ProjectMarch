How to create a URDF file with use of blender
=============================================
.. inclusion-introduction-start

This tutorial will describe how blender and some plugins can be used to systematically create a URDF.

.. inclusion-introduction-end

Introduction
------------

A URDF (Unified Robot Description Format) is a file that contains the physical description of a robot. To create this file in an easy way, blender can be used in combination with a plugin called phobos. To make it adaptable, we also use a python script that uses the Blender Python API (bpy) to perform all the steps in blender, instead of using the blender graphical interface.


Required software
-----------------
For this tutorial, the following software was used:

- Blender 3.1
- Phobos (forked to our gitlab)
- blender_autocomplete (forked to our gitlab)

Note: At moment of writing, phobos only had official support till blender 2.9 and blender_autocomplete till blender 3.0. However, the use with blender 3.1 did not result in problems.


Add blender python methods to IDE
---------------------------------

Blender has a python API called bpy, which makes it possible to perform almost every action in the blender graphical interface using a python script. To make the creation of a URDF easily adaptable and repeatable, this tutorial will explain how such a python script can be used. 

Blender uses its own python interpreter in which bpy is installed. This results in no code auto-completion, linting or analysis when writing a blender python script outside blender. To fix this, we can use blender_autocomplete, which contains (almost) all bpy methods:

Create a directory called 'urdf_with_blender' in your home directory and open it:
::
    
    mkdir ~/urdf_with_blender && cd ~/urdf_with_blender


Clone blender_autocomplete:
::
    
    git clone https://gitlab.com/project-march/urdf-with-blender/blender_autocomplete.git


Now we can add the bpy methods to make auto-completion, linting and analysis possible in your IDE. To do this for example with Visual Studio Code, open VS code from the 'urdf_with_blender' directory:
::
    
    code .

Open the settings (ctrl + ,) and switch to Workspace settings. Open the settings.json (top right corner). Add the following to the settings.json file:
::
    
    {
        "python.autoComplete.extraPaths": [
            "${env:HOME}/urdf_with_blender/blender_autocomplete/3.0",

        ],
        "python.linting.pylintArgs": [
            "--init-hook",
            "import sys; sys.path.append('${env:HOME}/urdf_with_blender/blender_autocomplete/3.0')",
        ],
        "python.analysis.extraPaths": [
            "${env:HOME}/urdf_with_blender/blender_autocomplete/3.0",
        ],
    }

To test if it works, open a file in your IDE in the workspace where blender_autocomplete was added and test if bpy is recognized by importing it:
::
    
    import bpy


Phobos plugin
-------------

To create a URDF from a blender model, we need a plugin called phobos. 

Installation
^^^^^^^^^^^^

Clone phobos in the 'urdf_with_blender' directory:
::
    
    cd ~/urdf_with_blender && git clone https://gitlab.com/project-march/urdf-with-blender/phobos.git
    
Open the phobos directory and zip the phobos directory that is inside it:
::
    
    cd ~/urdf_with_blender/phobos && zip -r phobos.zip phobos

Now we can install the zip in blender:
::
    
    blender->edit->preferences->addons->install

Select the zip file and install.
    
For more information about phobos, please read the wiki or watch this video tutorial (with an older version): https://www.youtube.com/playlist?list=PL4MdFHVi1I5W_hOJN5jAGn3JLWDL7q1Tr


Add phobos methods to IDE
^^^^^^^^^^^^^^^^^^^^^^^^^

Unfortunately does the blender_autocomplete only contain the default blender methods and no methods provided by plugins. To improve workability, a script was made that adds the phobos methods also to blender_autocomplete. To get this script, clone the URDF Creator repository in our workspace:
::
    
    cd ~/urdf_with_blender && git clone https://gitlab.com/project-march/urdf-with-blender/urdf-creator.git

In the urdf-creator repository, you will find a python script called 'add_phobos_methods_to_blender_autocomplete.py'. This script needs to know where it can find phobos, therefore we should add phobos also to our IDE. For Visual Studio Code, we can expand the settings.json we used before with as result:
::
    
    {
        "python.autoComplete.extraPaths": [
            "${env:HOME}/urdf_with_blender/blender_autocomplete/3.0",
            "${env:HOME}/urdf_with_blender/phobos"

        ],
        "python.linting.pylintArgs": [
            "--init-hook",
            "import sys; sys.path.append('${env:HOME}/urdf_with_blender/blender_autocomplete/3.0')",
            "import sys; sys.path.append('${env:HOME}/urdf_with_blender/phobos')",
        ],
        "python.analysis.extraPaths": [
            "${env:HOME}/urdf_with_blender/blender_autocomplete/3.0",
            "${env:HOME}/urdf_with_blender/phobos",
        ],
    }

Now we can have a look at the 'add_phobos_methods_to_blender_autocomplete.py' script. This script reads all the imported phobos operators and converts this to the definition style that blender_autocomplete uses and writes this to '~/urdf_with_blender/blender_autocomplete/3.0/bpy/ops/phobos.py'

After phobos.py is created, we need to add it to blender_autocomplete by adding
::
    
    from . import phobos

to '~/urdf_with_blender/blender_autocomplete/3.0/bpy/ops/__init__.py'.

We can check whether our IDE now recognizes phobos operators too by opening a test file in the workspace again, importing bpy and see if bpy.ops.phobos is recognized:
::
    
    import bpy
    bpy.ops.phobos


Create or adapt URDF creation script
------------------------------------

Now that we have installed everything and added all the methods to our IDE, we can easily create or adapt a URDF creation script with all the support from our IDE. The script that was used to create the march7 URDF is '~/urdf_with_blender/urdf-creator/create_march7_urdf.py'.

Run the script in blender
^^^^^^^^^^^^^^^^^^^^^^^^^

To run the script in blender, open blender and open scripting. Here we can select the script we would like to run. To easily change the script without reloading it in blender every time, there is also another script in '~/urdf_with_blender/urdf-creator' called 'script_runner.py'. We can define our actual URDF creation script in 'script_runner.py' and open 'script_runner.py' in blender. In blender we can execute the loaded script (alt + p) and the robot model will be loaded and exported to a URDF as defined in 'create_march7_urdf.py'.

.. figure:: images/blender_with_phobos.png
   :align: center

   The blender interface with the phobos plugin.

Blender has four important panels when using the phobos plugin in combination with scripting. Panel 1 shows the loaded script. In panel 3 and 4 we can perform phobos methods using the graphical interface of blender. (Almost) every phobos method that is performed using the graphical interface is also listed in panel 2 as the corresponding bpy command. Therefore you can first try things out in the interface and then copy the bpy command to your script.
