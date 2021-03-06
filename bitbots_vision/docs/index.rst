Welcome to |project|'s documentation!
================================================

Description
-----------

This is the vision ROS package of the Hamburg Bit-Bots.

The vision is able to detect balls, lines, the field itself, the field boundary, goal posts, teammates, enemies and other obstacles.

An earlier version of this pipeline is presented in our paper
`An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer
<https://robocup.informatik.uni-hamburg.de/wp-content/uploads/2019/06/vision_paper.pdf>`__.
When you use this pipeline or parts of it, please cite it.

::

   @inproceedings{vision2019,
   author={Fiedler, Niklas and Brandt, Hendrik and Gutsche, Jan and Vahl, Florian and Hagge, Jonas and Bestmann, Marc},
   year={2019},
   title={An Open Source Vision Pipeline Approach for RoboCup Humanoid Soccer},
   booktitle={RoboCup 2019: Robot World Cup XXIII},
   note = {Accepted},
   organization={Springer}
   }

The Bit-Bots vision pipeline architecture is modular allowing easy implementation of new approaches resulting in a high grade of customizability.

For ball detection, you can choose between an FCNN or multiple yolo implementations.
The goalpost detection also runs via yolo or a conventional detection method, which is also used for obstacle and robot detection.

The whole system is embedded in the ROS environment and able to run on many devices including the Intel Neural Compute Stick 2 (and the Nvidia Jetson TX2, legacy) in our Wolfgang robots.

In the context of the Hamburg Bit-Bots, the images are provided by a Basler industry grade ethernet camera.
The camera drivers are not included in this package for licensing reasons, but can be auto launched.
Every image source, that publishes a ``sensor_msgs/Image messages`` message is supported.

The ROS topics and many other parameters are defined in the ``visionparams.yaml`` config file.
All used parameters are also changeable during run-time using `ROS dynamic reconfigure <http://wiki.ros.org/dynamic_reconfigure>`_.
For simulation usage, different parameters can be defined in the ``simparam.yaml`` file which overrides the default params.

Neural network models are stored in the directory ``bitbots_vision/models``.
These models are not part of this repository.

For the field detection, which is needed for the field boundary and obstacle detection, the vision uses RGB lookup table provided by the ``colorpicker.py`` as in `Additional Scripts`_.
These color lookup tables can be improved and converted to a pickle file for faster loading times using the ``color_lookup_table_tool``.
The field color lookup table itself can be dynamically adapted in real-time using the dynamic color lookup table heuristic.
Therefore the vision gets more resistant to natural light conditions.

Launchscripts
-------------

To start the vision, use

::

   roslaunch bitbots_vision vision_startup.launch

The following parameters are available:

+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+
|Param                |Default  |Output                                                                                                                     |
+=====================+=========+===========================================================================================================================+
|``sim``              |``false``|Activate simulation time, switch to simulation color settings and deactivate launching of an image provider                |
+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+
|``camera``           |``true`` |Deactivate all image providers (e.g. for use with rosbags or in simulation)                                                |
+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+
|``basler``           |``true`` |Start the basler camera driver instead of the  `wolves_image_provider <https://github.com/bit-bots/wolves_image_provider>`_|
+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+
|``dummyball``        |``false``|NOT start the ball detection to save resources                                                                             |
+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+
|``debug``            |``false``|Activate publishing of several debug images which can be inspected in the rqt image view                                   |
+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+
|``use_game_settings``|``false``|Load additional game settings                                                                                              |
+---------------------+---------+---------------------------------------------------------------------------------------------------------------------------+


Color lookup table files
~~~~~~~~~~~~~~~~~~~~~~~~

The vision depends on the usage of color lookup table files which define color lookup tables primarily to detect the green field color and therefore lines and the field boundary.
These files are stored in the directory in ``/config/color_lookup_tables/``.
Use the provided ``colorpicker.py`` tool as in `Additional Scripts`_ to generate these files.

Currently, the Bit-Bots vision package supports two file types for color lookup tables:

-  ``.pickle``

   Generally, this is a generated binary representation of color lookup tables we use to prevent the long loading times of the ``.yaml`` format.

-  ``.yaml``

   This format is deprecated.
   This format provides a human readable representation of color lookup tables.
   See below for a example representation.

   ::

      color_values:
            greenField:
                red: [r_1,r_2,..., r_n]
                green: [g_1,g_2,..., g_n]
                blue: [b_1,b_2,..., b_n]

   Assume the following RGB color value ``(0, 153, 51)`` is part of the color lookup table.
   The correct representation is:

   ::

        r_i = 0
        g_i = 153
        b_i = 51

   For large color lookup tables, loading of such files takes a while.

White Balancer
--------------

This repository also includes the ``white_balancer``.
It is a ROS nodelet that color-corrects incoming images with a predefined light temperature.

Additional Scripts
------------------

In the bitbots_vision package, special tools for debugging/introspection purposes are provided.

-  ``/scripts/bitbots_imageloader.py``

   Loads a sequence of images from a directory and publishes them.

   This tool provides a help page ``-h`` for further details.

-  ``/scripts/colorpicker.py``

   A tool to create color lookup table files out of an video stream.

   This tool provides usage information on launch.
   Also have a look at the help page ``-h`` for more information on how to load
   existing color lookup table files or change the video input topic.

-  ``/scripts/color_lookup_table_tool.py``

   A small tool for color lookup table enhancement.

   The tool is able to find main clusters in the color lookup table,
   interpolate defined distances, add brightness thresholds and convert
   a yaml encoded to an pickle encoded color lookup table. It also visualizes
   the color lookup table in a browser based 3d graph.

   This tool provides a help page ``-h`` for further details.

-  ``/scripts/convert_to_image.py``

   This is a small script to
   convert ``RegionOfInterestWithImage`` of FCNNs to an ``Image``
   message for debug visualization.

-  ``scripts/extract_from_rosbag.py``

   This tool extracts ``Image`` messages of rosbags.

   Usage:

   ::

      rosrun bitbots_vision extract_from_rosbag.py -i /path/to/rosbag.bag -o /output/folder/

   The tool will guide you through the workflow. Optional parameters are
   prompted when not specified:

   -  ``-n N``: To select the frequency, every ``n``-th image will be
      saved
   -  ``-t TOPIC``: Topic of the ``image`` message

   Example:

   ::

      rosrun bitbots_vision extract_from_rosbag.py -i testdata.bag -o testdataset -t /camera/image_proc -n 3

   This will extract every third image from the ``testdata.bag`` on the
   ``/camera/image_proc`` message topic into the folder ``$PWD/testdataset``.

   This tool provides a help page ``-h`` for further details.

-  ``/scripts/imageclean.sh``

   This is a small bash script, to quickly sort a directory of images
   using feh with shortcuts. Start this inside the directory of images
   to sort.

   Press key:

   -  ``1`` -> move current image to **Trash** subdirectory
   -  ``2`` -> Move current image to **Balls** subdirectory
   -  ``3`` -> Move current image to **Goals** subdirectory
   - use ``arrow keys`` to navigate through images and keep the current image inside its original directory

- ``/scripts/rosbag_remapper.py``

   This script remaps the ``image_raw`` topic in old rosbags to the new ``camera/image_proc``. Input can either be a single
   bag or a folder containing bags. The new output bag or folder has an ``_updated`` appended to its name.

   Usage:

   ::

    rosrun bitbots_vision rosbag_remapper.py /path/to/rosbag_or_folder



.. toctree::
   :maxdepth: 2
   :caption: Interface documentation

   cppapi/library_root
   pyapi/modules

.. toctree::
    :maxdepth: 1
    :glob:
    :caption: Tutorials

    manual/tutorials/*

Indices and tables
==================

* :ref:`genindex`
* |modindex|
* :ref:`search`
