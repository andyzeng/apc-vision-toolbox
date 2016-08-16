# MIT-Princeton Vision Toolbox for the APC 2016

UNFINISHED ... PLEASE DO NOT USE

## Documentation
* [Realsense Standalone](#realsense-standalone)
* [Realsense ROS Package](#realsense-ros-package)

## Realsense Standalone

A standalone C++ executable for streaming and capturing data (RGB-D frames and 3D point clouds) in real-time using [librealsense](https://github.com/IntelRealSense/librealsense). Tested on Ubuntu 14.04 and 16.04 with an Intel® RealSense™ F200 Camera.

See `realsense_standalone`

### Dependencies

1. [librealsense](https://github.com/IntelRealSense/librealsense) (installation instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md))
 * Install with the Video4Linux backend

2. OpenCV (tested with OpenCV 3.1)
 * Used for saving images


### Compilation
```shell
cd realsense_standalone
./build.sh
```

### Usage

Run `./capture` to begin streaming RGB-D frames from the Realsense device. While the stream window is active, press the space-bar key to capture and save the current RGB-D frame to disk. Relevant camera information and captured RGB-D frames are saved to a randomly named folder under `data`. 

If your Realsense device is plugged in but remains undetected, try using a different USB port. If that fails, run the following script while the device is unplugged to refresh your USB ports:

```shell
sudo ./scripts/resetUSBports.sh
```

## Realsense ROS Package

Warning: unfinished!
TODO: Finish clean up, write documentation, and add branch with PCL/non-PCL support ...

A C++ ROS package for streaming and capturing data (RGB-D frames and 3D point clouds) in real-time using [librealsense](https://github.com/IntelRealSense/librealsense). Tested on Ubuntu 14.04 and 16.04 with an Intel® RealSense™ F200 Camera.

See `ros_packages/realsense_camera`

### Dependencies

1. [librealsense](https://github.com/IntelRealSense/librealsense) (installation instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md))
 * Install with the Video4Linux backend

2. OpenCV (tested with OpenCV 3.1)
 * Used for saving images

3. [Optional] Point Cloud Library (tested with PCL 1.7.2)
 * Used for saving point clouds

### Compilation
* Copy the ROS package `ros_packages/realsense_camera` into your catkin workspace source directory (e.g. `catkin_ws/src`)
* Configure `realsense_camera/CMakeLists.txt` according to the versions of your respective dependencies.
* In your catkin workspace, compile the package with `catkin_make` 
* Source `devel/setup.sh`

### Usage
* Start `roscore`
* Run `rosrun realsense_camera capture` to stream data from the sensor and start the data capture service:
  * `/realsense_camera` returns data from the sensor (response data format described in `realsense_camera/srv/StreamSensor.srv`)
  * if you need the GL window to see the data do `rosrun realsense_camera capture _useGL:=True`
 
