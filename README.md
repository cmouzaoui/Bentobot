Bentobot Programs
===
Here, you will see all programs that were used and developed by the Bentobot team (Team 8 of Dr. Hong's mechanical engineering design class) for their launcher. The Bentobot launcher is designed to locate an object of a certain color, such as an orange balloon, triangulate its 3D coordinates, and then send a command to an Arduino Uno via serial that actuates motors that turn a Nerf launcher towrards the object, and shoots at it. 

Within each folder you will find the source code for a single program or for a closely related family of programs. In order to build any of the programs, type the following in a terminal:

        mkdir build
        cmake ..
        make

The CMakeLists.txt file assumes a Linux machine (such as a Raspberry Pi), so compilation on non-Linux machines are not guaranteed). Common dependencies for these programs are: cmake, OpenCV, the [wirinSerial library](http://wiringpi.com/reference/serial-library/), and the [Raspicam library](https://github.com/cedricve/raspicam). 

ArduinoCode
---
This folder contains all code that was uploaded to the Arduino. `Static_Arduino` receives a command to move and then moves to the appropriate position without interruption. `Moving_Arduino` constantly checks for new commands to move to move positions every time it moves the motors a few steps.

`Bentobot.cpp`
---
This was test code for having the Raspberry Pi communicate with the motors directly using its GPIO pins and the wiringPi library. This was abandoned due to timing issues on the Raspberry Pi.

CircleDetect, DisplayImage, ModifyTest, Reduction
---
All of these programs are simple reproductions of tutorials hosted on the OpenCV website. `CircleDetect` detects a circle in a test image using OpenCV. `DisplayImage` displays a test image using OpenCV. `ModifyTest` reads an image, changes it to grayscale, and saves it. `Reduction` is a simple exercise in accessing elements within `Mat` structures in OpenCV. 

`range-detector.py`
---
A program that helps find the HSV thresholds necessary to isolate an object of a certain color or range of colors from a video feed. Taken from [https://github.com/jrosebr1/imutils](https://github.com/jrosebr1/imutils). 

SC
---
A program that asks for the 3d coordinates of a point in space, and then sends a command to the Arduino to shoot at that point.

VideoDetect
---
Like `range-detector.py`, it helps find the HSV (Hue, Saturation, Value) thresholds necessary to isolate an object of a certain color or range of colors from a video feed. Pass the argument `0` to get the feed from an external camera, and `1` to get the feed from the internal Raspberry Pi camera, like so:
    ./VideoDetect 1
Saves the threshold in a threshold .yml file, named after the camera used to find the threshold. 

CameraCalibrate
---
`Calibrate_sample` finds the camera matrix and distortion coefficients for external cameras. It looks for a chessboard pattern in the frame. Pass the `--help` argument to know all of the different arguments that can be passed to it.

`Calibration_raspicam` does the same thing as `Calibrate_sample`, but for the Raspberry Pi camera.

`Triangulate` outputs the 3D location of the center of the largest contour that matches the thresholds in `threshold_0.yml` and `threshold_1.yml`. It looks for .yml files for the camera matrices and distortion coefiction of both cameras, for projection matrices and other rectification data in `camerapair.yml`, and for the relative location of the first camera in `pnp.yml`. Press `g` to re-enter stereo rectificaton mode, and `c` to capture images with a detected asymmetric circle grid from both cameras for stereo rectification when in that mode. Press `p` to find the relative location of the first camera from an image with a detected asymmetric circle grid from the first camera. Press `f` to find a scale factor for unit transformation from an image with an asymmetric circle grid in a special position. Press `t` to triangulate the location of the object from the centers of the contours.

Final
---
`Triangulation_withSC` combines the features of `SC` and `Triangulate` by triangulating a point, and then sending a command over to the Arduino. The same key commands in `Triangulate` apply here, except that `t` also shoots. 
