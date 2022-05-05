# Project on Localizate Aruco: elementaryBASIS

elementaryBASIS: robots localization based on ArUco markers

# Table of content
-  [General description](#general-description)
-  [Structure](#struct)
-  [Installation](#install)
-  [Running](#running)
-  [Closing remarks](#contact)

# General description <a name="general-description"></a>
The ROS package allows you to localize the robot on the playing field. Current package is intended for use in EUROBOT robotic competitions, but also can be used in other projects as well.
# Structure <a name="struct"></a>
Here are the structure of package:
- [**instructions**](instructions) !TODO! how to configure package for your field
- [**config**](config) all dynamic config files should be saved here
  - **c270.yaml** camera's intrinsics
  - **camera_pos.yaml** automatically generated file, appears after camera position calibration
- [**python scripts**](python_scripts) all localizer source are here
  - **main.py** here you can change topics names
  - **configuration.py** markers defenitions available here
# Installation <a name="install"></a>
```bash
mkdir src
cd src
git clone https://github.com/elemtaryBASIS/localizate_aruco/
cd ..
catkin build
```
# Running <a name="running"></a>
Before starting, you should make sure that you know the internal parameters of the camera (intrinsics), if this is not the case, please follow this [instruction](https://gist.github.com/atokagzx/65ba6a2041c47c257ae4d17b16374d15).
Save calibration file as **.yaml** to the [config](/config) folder. Example is given [here](/config/c270.yaml).
```bash
rosparam set /localizer/camera_params $(find localizate_aruco)/config/c270_1280x720.yaml
rosparam set /localizer/static_params $(find localizate_aruco)/config/camera_pos.yaml
rosrun localizate_aruco main.py __name:=localizer
```
##### Make sure that ros master has been started and camera stream is available at /camera/image_raw
# Contact info <a name="contact"></a>
Maintainer: [Yaroslav](mailto:y.savelev@edu.misis.ru)
