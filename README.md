# MIT-Princeton Vision Toolbox for the APC 2016



## Documentation
* [Realsense Standalone](#realsense-standalone)
* [Realsense ROS Package](#realsense-ros-package)
* [Deep Learning FCN ROS Package](#deep-learning-fcn-ros-package)
* [FCN Training with Marvin](#fcn-training-with-marvin)
* [Evaluation Code](#evaluation-code)

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

A C++ ROS package for streaming and capturing data (RGB-D frames and 3D point clouds) in real-time using [librealsense](https://github.com/IntelRealSense/librealsense). Tested on Ubuntu 14.04 and 16.04 with an Intel® RealSense™ F200 Camera. 

This ROS packages comes in two different versions. Which version is installed will depend on your system's available software:

* Version #1: only returns RGB-D frame data on service calls (does not require OpenCV or PCL)
* Version #2: returns RGB-D frame data on service calls and publishes 3D point clouds (requires OpenCV and PCL)

See `ros-packages/realsense_camera`

### Dependencies

1. [librealsense](https://github.com/IntelRealSense/librealsense) (installation instructions can be found [here](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md))
 * Install with the Video4Linux backend

2. [Optional] OpenCV (tested with OpenCV 2.4.11)
 * Used for saving images

3. [Optional] Point Cloud Library (tested with PCL 1.7.1)
 * Used for saving point clouds

### Compilation
1. Copy the ROS package `ros_packages/realsense_camera` into your catkin workspace source directory (e.g. `catkin_ws/src`)
2. If necessary, configure `realsense_camera/CMakeLists.txt` according to your respective dependencies
3. In your catkin workspace, compile the package with `catkin_make` 
4. Source `devel/setup.sh`

### Usage
* Start `roscore`
* To start the RGB-D data capture service and stream data from the sensor, run: 

```shell
rosrun realsense_camera capture
```

  * The service `/realsense_camera` returns data from the sensor (response data format described in `realsense_camera/srv/StreamSensor.srv`)
  * If you need a GL window to see the streamed RGB-D data, run `rosrun realsense_camera capture _display:=True`
 
## Deep Learning FCN ROS Package

A C++ ROS package for deep learning based object segmentation using [FCNs (Fully Convolutional Networks)](https://arxiv.org/abs/1411.4038) with [Marvin](http://marvin.is/), a lightweight GPU-only neural network framework. This package feeds RGB-D data forward through a pre-trained ConvNet to retrieve object segmentation results. The neural networks are trained offline with Marvin (see [FCN Training with Marvin](#fcn-training-with-marvin)).

See `ros-packages/marvin_convnet`

### Dependencies

1. [Realsense ROS Package](#realsense-ros-package) needs to be compiled first.

2. [CUDA 7.5](https://developer.nvidia.com/cuda-downloads) and [cuDNN 5](https://developer.nvidia.com/cudnn). You may need to register with NVIDIA. Below are some additional steps to set up cuDNN 5. **NOTE** We highly recommend that you install different versions of cuDNN to different directories (e.g., ```/usr/local/cudnn/vXX```) because different software packages may require different versions.

```shell
LIB_DIR=lib$([[ $(uname) == "Linux" ]] && echo 64)
CUDNN_LIB_DIR=/usr/local/cudnn/v5/$LIB_DIR
echo LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDNN_LIB_DIR >> ~/.profile && ~/.profile

tar zxvf cudnn*.tgz
sudo cp cuda/$LIB_DIR/* $CUDNN_LIB_DIR/
sudo cp cuda/include/* /usr/local/cudnn/v5/include/
```

3. OpenCV (tested with OpenCV 2.4.11)
 * Used for saving images

### Compilation
1. Copy the ROS package `ros_packages/marvin_fcn` into your catkin workspace source directory (e.g. `catkin_ws/src`)
2. If necessary, configure `realsense_camera/CMakeLists.txt` according to your respective dependencies
3. In your catkin workspace, compile the package with `catkin_make` 
4. Source `devel/setup.sh`

change location of where net is loaded

make sure where data is going to be read and written

ros package to compute hha 

`rosrun marvin_convnet detect _service_mode:=1 _write_directory:="/home/andyz/apc/toolbox/data/tmp"`

`rosrun marvin_convnet detect _service_mode:=2 _read_directory:="/home/andyz/apc/toolbox/data/tmp" _write_directory:="/home/andyz/apc/toolbox/data/tmp"`

`rosservice call /marvin_convnet ["elmers_washable_no_run_school_glue","expo_dry_erase_board_eraser"] 0 0`

## FCN Training with Marvin

Code and models for training object segmentation using [FCNs (Fully Convolutional Networks)](https://arxiv.org/abs/1411.4038) with [Marvin](http://marvin.is/), a lightweight GPU-only neural network framework. Includes network architecture .json files in `convnet-training/models` and a Marvin data layer in `convnet-training/apc.hpp` that randomly samples images (RGB and HHA) from the segmentation training dataset [here](http://www.cs.princeton.edu/~andyz/apc2016).

See `convnet-training`

### Dependencies

1. [CUDA 7.5](https://developer.nvidia.com/cuda-downloads) and [cuDNN 5](https://developer.nvidia.com/cudnn). You may need to register with NVIDIA. Below are some additional steps to set up cuDNN 5. **NOTE** We highly recommend that you install different versions of cuDNN to different directories (e.g., ```/usr/local/cudnn/vXX```) because different software packages may require different versions.

```shell
LIB_DIR=lib$([[ $(uname) == "Linux" ]] && echo 64)
CUDNN_LIB_DIR=/usr/local/cudnn/v5/$LIB_DIR
echo LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDNN_LIB_DIR >> ~/.profile && ~/.profile

tar zxvf cudnn*.tgz
sudo cp cuda/$LIB_DIR/* $CUDNN_LIB_DIR/
sudo cp cuda/include/* /usr/local/cudnn/v5/include/
```

2. OpenCV (tested with OpenCV 2.4.11)
 * Used for reading images

### Setup Instructions
1. Download segmentation training dataset from [here](http://www.cs.princeton.edu/~andyz/apc2016)
2. Specify training dataset filepath in APCData layer of network architecture in `convnet-training/models/train_shelf_color.json`
3. Navigate to `convnet-training/models/weights/` and run bash script `./download_weights.sh` to download VGG pre-trained weights on ImageNet (see [Marvin](http://marvin.is/) for more pre-trained weights)
4. Navigate to `convnet-training/` and run in terminal `./compile.sh` to compile Marvin.
5. Run in terminal `./marvin train models/rgb-fcn/train_shelf_color.json models/weights/vgg16_imagenet_half.marvin` to train a segmentation model on RGB-D data with objects in the shelf (for objects in the tote, use network architecture `models/rgb-fcn/train_shelf_color.json`).

## Evaluation Code
Code used to perform the experiments in the paper - tests the full vision system on the 'Shelf & Tote' benchmark dataset.

See `evaluation`

### Setup Instructions
1. Download the full 'Shelf & Tote' benchmark dataset from [here](http://www.cs.princeton.edu/~andyz/apc2016) and extract its contents to `apc-vision-toolbox/data/benchmark` (e.g. `apc-vision-toolbox/data/benchmark/office`, `apc-vision-toolbox/data/benchmark/warehouse', etc.)
2. In `evaluation/getError.m`, change the variable `benchmarkPath` to point to the filepath of your benchmark dataset directory
3. We have provided our vision system's predictions in a saved Matlab .mat file `evaluation/predictions.mat`. To compute the accuracy of these predictions against the ground truth labels of the 'Shelf & Tote' benchmark dataset, run `evaluation/getError.m`





