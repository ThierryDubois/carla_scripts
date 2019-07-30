**CARLA README BEGINS**
--------------


CARLA Simulator

CARLA is an open-source simulator for autonomous driving research. CARLA has
been developed from the ground up to support development, training, and
validation of autonomous driving systems. In addition to open-source code
and protocols, CARLA provides open digital assets (urban layouts, buildings,
vehicles) that were created for this purpose and can be used freely. The
simulation platform supports flexible specification of sensor suites and
environmental conditions.

Building CARLA
--------------

Use `git clone` or download the project from this page. Note that the master
branch contains the latest fixes and features, for the latest stable code may be
best to switch to the `stable` branch.

Then follow the instruction at [How to build on Linux][buildlinuxlink] or
[How to build on Windows][buildwindowslink].

[buildlinuxlink]: http://carla.readthedocs.io/en/latest/how_to_build_on_linux
[buildwindowslink]: http://carla.readthedocs.io/en/latest/how_to_build_on_windows
[issue150]: https://github.com/carla-simulator/carla/issues/150

F.A.Q.
------

If you run into problems, check our
[FAQ](http://carla.readthedocs.io/en/latest/faq/).

License
-------

CARLA specific code is distributed under MIT License.

CARLA specific assets are distributed under CC-BY License.

Note that UE4 itself follows its own license terms.

**CARLA README ENDS**
--------------


MOT DATA Script (Python 2)
-------
The 2D_BB_manuak_record.py script is a hybrid between the provided manual_control.py, spawn_npc.py, and client_bounding_boxes.py

It allows you to record camera images and output the annotations in a csv with the following format:

`<frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z> `

TRAINING DATA Script (Python 3)
-------
After the recording, all the frames will be saved in the `carla/PythonAPI/examples/_out` along with:

The CSV file for the recording

From this folder, the script `convert_training_data.py` will separate the recording in case there were errors (frame skips)

This way, all videos are continuous with no skipped frames.

The csv files are adjusted automatically and copied into subfolders with the related png files

If you want to make a video with all the frames of a subfolder, usse the following command:

`ffmpeg -framerate 10 -pattern_type glob -i '*.png' -c:v libx264 -pix_fmt yuv420p out.mp4`

DATA TRANFORMATION FOR USAGE WITH DATA GENERATOR
-------

To use this data with the data generator the `format_images_for_data_generator.py` script can be run from the root folder that contains the data in the following pattern

ROOT
    => Sets
        => Sequences
            => images

This script takes all the images and renames them so they follow the following pattern (like Caltech data):

set01_V02_3.png

Where 01 is the set ID, 02 is the video sequence ID and 3 is the frame number

ALl these images are now in the same folder, which can be referenced in in the contruction of a DataGenerator as seen in a script such as `generator_caltech_sliding_separate_output_skip_option.py`

Autopilot script
-------

This script records simulation frames without annotations. 