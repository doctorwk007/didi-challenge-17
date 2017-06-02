# Didi Challenge 17
ROS Implementation of our solution to Didi Challenge 2017.

Package developed at [Intelligent Systems Laboratory](http://www.uc3m.es/islab), Universidad Carlos III de Madrid.


## Prerequisites
#### Software
* Ubuntu 14.04/16.04
* ROS Indigo/Kinetic
* OpenCV 3.2 with _opencv__contrib_ installed
* Velodyne ROS driver from [here](https://github.com/ros-drivers/velodyne)
* Python 2.7
* C++11
* CUDA

#### Hardware
* NVIDIA GPU for Deep Learning Classifier


## Setup
0) Clone this repository into your catkin workspace ;)

1) Fetch submodule dependencies:

    ```shell
    cd ~/catkin_ws/src/didi-challenge-17
    git submodule update --init --recursive
    ```

2) Follow these [instructions](TODO) to install and configure the classifier dependencies (Mainly _Caffe_ and _pycaffe_)

3) Compile the ROS nodes

    ```shell
    cd ~/catkin_ws/
    catkin_make
    ```

## How to run?
`roslaunch didi_utils awesome_classifier.launch`

## Known issues
If compiler complains about some OpenCv dependency, make sure CMakeLists.txt points to the actual OpenCv installation path in your computer (remember to install them along with _opencv__contrib_)
