# MIT-Princeton Vision Toolbox for the APC 2016
Toolbox code for our vision system that took 3rd and 4th place at the Amazon Picking Challenge 2016. Includes RGB-D Realsense sensor drivers (standalone and ROS package), deep learning ROS package for 2D object segmentation (training and testing), ROS package for 6D pose estimation. This is the reference implementation of models and code for our paper:

### Multi-view Self-supervised Deep Learning for 6D Pose Estimation in the Amazon Picking Challenge ([pdf](),[arxiv](),[webpage](http://www.cs.princeton.edu/~andyz/apc2016))

*Andy Zeng, Kuan-Ting Yu, Shuran Song, Daniel Suo, Ed Walker Jr., Alberto Rodriguez and Jianxiong Xiao*

Warehouse automation has attracted significant interest in recent years, perhaps most visibly by the Amazon Picking Challenge (APC). Achieving a fully autonomous pick-and-place system requires a robust vision system that reliably recognizes objects and their 6D poses. However, a solution eludes the warehouse setting due to cluttered environments, self-occlusion, sensor noise, and a large variety of objects. In this paper, we present a vision system that took 3rd- and 4th- place in the stowing and picking tasks, respectively at APC 2016. Our approach leverages multi-view RGB-D data and data-driven, self-supervised learning to overcome the aforementioned difficulties. More specifically, we first segment and label multiple views of a scene with a fully convolutional neural network, and then fit pre-scanned 3D object models to the resulting segmentation to get the 6D object pose. Training a deep neural network for segmentation typically requires a large amount of training data with manual labels. We propose a self-supervised method to generate a large labeled dataset without tedious manual segmentation that could be scaled up to more object categories easily. We demonstrate that our system can reliably estimate the 6D pose of objects under a variety of scenarios.

![alt tag](https://github.com/andyzeng/apc-vision-toolbox/teaser.png)

#### Citing

If you find this code useful in your work, please consider citing:

```shell
@incollection{zeng2016apcvision,
  author = {Andy Zeng and Kuan-Ting Yu and Shuran Song and Daniel Suo and Ed Walker Jr. and Alberto Rodriguez and Jianxiong Xiao},
  title = {Multi-view Self-supervised Deep Learning for 6D Pose Estimation in the Amazon Picking Challenge},
  journal={arXiv},
  year={2016}
}
```

#### License

This code is released under the Simplified BSD License (refer to the LICENSE file for details).

#### Datasets
All relevant dataset information and downloads can be found [here](http://www.cs.princeton.edu/~andyz/apc2016).

#### Contact
If you have any questions or find any bugs, please let me know: [Andy Zeng](http://www.cs.princeton.edu/~andyz/) (andyz[at]princeton[dot]edu)

## Documentation
* [A Quick Start: Matlab Demo](#a-quick-start-matlab-demo)
* [6D Pose Estimation ROS Package](#6d-pose-estimation-ros-package)
* [Realsense Standalone](#realsense-standalone)
* [Realsense ROS Package](#realsense-ros-package)
* [Deep Learning FCN ROS Package](#deep-learning-fcn-ros-package)
* [FCN Training with Marvin](#fcn-training-with-marvin)
* [Evaluation Code](#evaluation-code)
* [RGB-D Annotator](#rgb-d-annotator)

## A Quick-Start: Matlab Demo
Estimates 6D object poses on the sample scene data (in `data/sample`) with pre-computed object segmentation results from [Deep Learning FCN ROS Package](#deep-learning-fcn-ros-package):

1. `git clone https://github.com/andyzeng/apc-vision-toolbox.git` (Note: source repository size is ~300mb, cloning may take a while)
2. `cd apc-vision-toolbox/ros-packages/catkin_ws/src/pose_estimation/src/`
3. Start Matlab and run `mdemo`

## 6D Pose Estimation ROS Package
A Matlab ROS Package for estimating 6D object poses by model-fitting with ICP on RGB-D object segmentation results.

### Dependencies
1. [Deep Learning FCN ROS Package](#deep-learning-fcn-ros-package) and all of its respective dependencies.
2. Matlab 2015b or later

### Compilation
1. Copy the ROS package `ros_packages/.../pose_estimation` into your catkin workspace source directory (e.g. `catkin_ws/src`)
2. Follow the instructions on the top of `pose_estimation/src/make.m` to compile ROS custom messages for Matlab
3. Compile a GPU CUDA kernel function in `pose_estimation/src`:
```shell
nvcc -ptx KNNSearch.cu
```

### Usage
* Start `roscore`
* To start the pose estimation service, run `pose_estimation/src/startService.m`. At each call (see service request format described in `pose_estimation/srv/EstimateObjectPose.srv`), the service:
 * Calibrates the camera poses of the scene using calibration data
 * Perform 3D background subtraction
 * For each object in the scene, use model-fitting to estimate its 6D pose

### Demo
1. Install all dependencies and compile this package
2. Start `roscore` in terminal
3. Create a temporary directory to be used by marvin_convnet for reading RGB-D data and saving segmentation masks
 * `mkdir /path/to/your/data/tmp`
4. `rosrun marvin_convnet detect _read_directory:="/path/to/your/data/tmp"`
5. Navigate to `pose_estimation/src`
6. Edit file paths and options on the top of `demo.m`
6. Open Matlab and run:
```shell
startService.m
demo.m
```

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
1. Copy the ROS package `ros_packages/.../realsense_camera` into your catkin workspace source directory (e.g. `catkin_ws/src`)
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
1. Copy the ROS package `ros_packages/.../marvin_convnet` into your catkin workspace source directory (e.g. `catkin_ws/src`)
2. If necessary, configure `realsense_camera/CMakeLists.txt` according to your respective dependencies
3. In your catkin workspace, compile the package with `catkin_make` 
4. Source `devel/setup.sh`

### Usage
* Navigate to `models/competition/` and run bash script `./download_weights.sh` to download our trained weights for object segmentation (trained on our [training dataset](http://www.cs.princeton.edu/~andyz/apc2016))
* Edit `marvin_convnet/src/detect.cu`: Towards the top of the file, specify the filepath to the network architecture .json file and .marvin weights.
* Create a folder called `tmp` in `apc-vision-toolbox/data` (e.g. `apc-vision-toolbox/data/tmp`). This where marvin_convnet will read/write RGB-D data. The format of the data in `tmp` follows the format of the scenes in our [datasets](http://www.cs.princeton.edu/~andyz/apc2016) and the format of the data saved by [Realsense Standalone](#realsense-standalone).
* marvin_convnet offers two services: `save_images` and `detect`. The former retrieves RGB-D data from the [Realsense ROS Package](#realsense-ros-package) and writes to disk in the `tmp` folder, while the latter reads from disk in the `tmp` folder and feeds the RGB-D data forward through the FCN and saves the response images to disk 
* To start the RGB-D data saving service, run: 

```shell
rosrun marvin_convnet save_images _write_directory:="/path/to/your/data/tmp" _camera_service_name:="/realsense_camera"
```

* To start the FCN service, run:

```shell
rosrun marvin_convnet detect _read_directory:="/path/to/your/data/tmp" _service_name:="/marvin_convnet"
```

* Example ROS service call to do object segmentation for glue bottle and expo marker box (assuming the scene's RGB-D data is in the `tmp` folder):

```shell
rosservice call /marvin_convnet ["elmers_washable_no_run_school_glue","expo_dry_erase_board_eraser"] 0 0
```

## FCN Training with Marvin

Code and models for training object segmentation using [FCNs (Fully Convolutional Networks)](https://arxiv.org/abs/1411.4038) with [Marvin](http://marvin.is/), a lightweight GPU-only neural network framework. Includes network architecture .json files in `convnet-training/models` and a Marvin data layer in `convnet-training/apc.hpp` that randomly samples RGB-D images (RGB and HHA) from our [segmentation training dataset](http://www.cs.princeton.edu/~andyz/apc2016).

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
1. Download our [segmentation training dataset](http://www.cs.princeton.edu/~andyz/apc2016)
2. Navigate to directory `convnet-training/`
2. Specify training dataset filepath in APCData layer of network architecture in `models/train_shelf_color.json`
3. Navigate to `models/weights/` and run bash script `./download_weights.sh` to download VGG pre-trained weights on ImageNet (see [Marvin](http://marvin.is/) for more pre-trained weights)
4. Navigate to `convnet-training/` and run in terminal `./compile.sh` to compile Marvin.
5. Run in terminal `./marvin train models/rgb-fcn/train_shelf_color.json models/weights/vgg16_imagenet_half.marvin` to train a segmentation model on RGB-D data with objects in the shelf (for objects in the tote, use network architecture `models/rgb-fcn/train_shelf_color.json`).

## Evaluation Code
Code used to perform the experiments in the paper; tests the full vision system on the 'Shelf & Tote' benchmark dataset.

See `evaluation`

### Setup Instructions
1. Download our 'Shelf & Tote' benchmark dataset from [here](http://www.cs.princeton.edu/~andyz/apc2016) and extract its contents to `apc-vision-toolbox/data/benchmark` (e.g. `apc-vision-toolbox/data/benchmark/office`, `apc-vision-toolbox/data/benchmark/warehouse', etc.)
2. In `evaluation/getError.m`, change the variable `benchmarkPath` to point to the filepath of your benchmark dataset directory
3. We have provided our vision system's predictions in a saved Matlab .mat file `evaluation/predictions.mat`. To compute the accuracy of these predictions against the ground truth labels of the 'Shelf & Tote' benchmark dataset, run `evaluation/getError.m`

## RGB-D Annotator
An online WebGL-based tool for annotating ground truth 6D object poses on RGB-D data. Follows an implementation of [RGB-D Annotator](https://github.com/danielsuo/rgbd-annotator) with light changes.



