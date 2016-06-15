# duo_wrapper
A Wrapper that makes using the Duo Camera easier. Optionally ROS compatible

## Features

+ Simplifies DUO Camera usage by setting up interface class
+ Uses a Singleton class to manage duo
+ Optionally allows rectification with openCV via CPU or GPU
+ OpenCV calibration is stored in .yaml files, compatible with other opencv apps. Option to create these based on the calibration stored on the camera.
+ Optionally allows stereo matching with openCV GPU or Nvidia Visionworks
+ Save settings between uses (Gain, Exposure, LEDS)

This was developped because I got frustrated having so much overhead using the Duo camera. To use this in a new project you simply have to include the header and library (tbd)

## Future work

+ Neural network based stereo matching
 
## Installation

Simply download and compile with cmake. Settings files are included but will need to be adapted to your camera / specifications. These can be found under the cfg/ folder.
