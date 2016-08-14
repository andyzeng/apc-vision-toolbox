# MIT-Princeton Vision Toolbox for the APC 2016

## Documentation
* [Realsense Standalone](#Realsense Standalone)
* [Realsense ROS Package](#Realsense ROS Package)

## Realsense Standalone

A standalone C++ executable for streaming and capturing data (RGB-D frames and 3D point clouds) in real-time using [librealsense](https://github.com/IntelRealSense/librealsense). Tested on Ubuntu 14.04 and 16.04 with an Intel® RealSense™ F200 Camera.

Found in `realsense_standalone`

### Dependencies

1. [librealsense](https://github.com/IntelRealSense/librealsense) (installation instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md))
 * Install with the Video4Linux backend

2. OpenCV (tested with OpenCV 3.1)
 * Used for saving images


### Compilation
`./build.sh
`

###Usage

Run `./capture` to begin streaming RGB-D frames from the Realsense device. While the stream window is active, press the space-bar key to capture and save the current RGB-D frame to disk. Relevant camera information and captured RGB-D frames are saved to a randomly named folder under `data`. 

If the Realsense device is undetected despite being plugged in, try using a different USB port. If that fails, run the following script while the device is unplugged to refresh your USB ports:

`sudo ./scripts/resetUSBports.sh
`

## Realsense ROS Package

A C++ ROS package for streaming and capturing data (RGB-D frames and 3D point clouds) in real-time using [librealsense](https://github.com/IntelRealSense/librealsense). Tested on Ubuntu 14.04 and 16.04 with an Intel® RealSense™ F200 Camera.

Found in `ros_packages/catkin_ws/src/realsense`

### Dependencies

1. [librealsense](https://github.com/IntelRealSense/librealsense) (installation instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md))
 * Install with the Video4Linux backend

2. OpenCV (tested with OpenCV 3.1)
 * Used for saving images

3. Point Cloud Library - PCL (tested with PCL 1.7)
 * Used for saving point clouds

### ROS Usage
* Clone ```apc_sensor``` into the catkin workspace source directory (e.g., ```catkin_ws/src```)
* Configure 
* In your catkin workspace, compile the package with `catkin_make` 
* Source `devel/setup.sh`
* Start `roscore`
* Run `rosrun apc_sensor capture` on the brix computer to stream data from the sensor and start the data capture service:
  * `/apc_sensor` returns data from the sensor (response data format described in `apc_vision/srv/StreamSensor.srv`)
  * if you need the GL window to see the data do `rosrun apc_sensor capture _useGL:=True`
 
