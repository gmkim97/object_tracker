# object_tracker

## Overview
Object tracking using Intel depthcamera(d435i) and darknet_ros.

### Author:
- **[GyeongMin Kim](https://github.com/gmkim97), kgm04008@gmail.com**
- **[TaeHyeon Kim](https://github.com/QualiaT), qualiatxr@gmail.com**


## How to use? - in ROS melodic(Ubuntu 18.04)

### Dependencies
This software is built on the Robotic Operating System([ROS](http://wiki.ros.org/ROS/Installation)) melodic.

### For Realsense SDK 2.0 Installation
```
$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb http://librealsense.intel.com/Debian/apt-repo bionic main" -u

$ sudo apt-get install librealsense2-dkms
$ sudo apt-get install librealsense2-utils
$ sudo apt-get install librealsense2-dev
$ sudo apt-get install librealsense2-dbg

# test
$ realsense-viewer
```

### For Realsense ROS package Installation
```
$ export ROS_VER=melodic
$ sudo apt-get install ros-melodic-realsense2-camera
$ cd ~/catkin_ws/src/
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
$ cd ~/catkin_ws/
$ catkin_make
```

### For OpenCV4 Installation
```
$ sudo apt install build-essential cmake git pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev
$ mkdir ~/opencv && cd ~/opencv
$ git clone https://github.com/opencv/opencv.git
$ git clone https://github.com/opencv/opencv_contrib.git
$ cd ~/opencv/opencv
$ mkdir build && cd build
```

- If you don't have or use CUDA, then use following option.
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_C_EXAMPLES=ON \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib/modules \
  -D BUILD_EXAMPLES=ON ..
```

- Based on Geforce RTX 3070, I used CUDA 11.2 and option as follows
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D OPENCV_EXTRA_MODULES_PATH=~/opencv/opencv_contrib-4.5.0/modules \
-D INSTALL_C_EXAMPLES=ON \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D BUILD_EXAMPLES=ON \
-D BUILD_DOCS=OFF \
-D BUILD_SHARED_LIBS=ON \
-D BUILD_opencv_python2=ON \
-D BUILD_opencv_python3=ON \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D WITH_CUDA=ON \
-D WITH_CUBLAS=ON \
-D CUDA_FAST_MATH=1 \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.2 \
-D CUDA_ARCH_BIN=8.6 \
-D WITH_OPENCL=ON \
-D OPENCV_SKIP_PYTHON_LOADER=ON \..
```

- make
```
$ sudo make -j{# of CPU cores}
$ sudo make install

# If you don't know your # of CPU cores, then use the following code
$ cat /proc/cpuinfo | grep processor | wc -l
```

### darknet_ros Installation
- **[darknet_ros wiki](http://wiki.ros.org/darknet_ros)**
- **[darknet_ros github](https://github.com/leggedrobotics/darknet_ros)**


### Installation of object tracker
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/gmkim97/object_tracker.git
$ cd ~/catkin_ws/
$ catkin_make
```

### Installation of pyrealsense2 module
```
$ sudo apt-get install python-pip
$ pip install pyrealsense2
```

### How to start?
```
$ roslaunch object_tracker object_tracking_with_py2.launch 
```





## How to use? - in ROS noetic(Ubuntu 20.04)

### Dependencies
This software is built on the Robotic Operating System([ROS](http://wiki.ros.org/ROS/Installation)) noetic.

### For Realsense SDK 2.0 Installation
```
$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
$ sudo add-apt-repository "deb http://librealsense.intel.com/Debian/apt-repo focal main" -u

$ sudo apt-get install librealsense2-dkms
$ sudo apt-get install librealsense2-utils
$ sudo apt-get install librealsense2-dev
$ sudo apt-get install librealsense2-dbg

# test
$ realsense-viewer
```

### For Realsense ROS package Installation
```
$ export ROS_VER=noetic
$ sudo apt-get install ros-noetic-realsense2-camera
$ cd ~/catkin_ws/src/
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
$ cd ~/catkin_ws/
$ catkin_make
```

### darknet_ros Installation
- **[darknet_ros wiki](http://wiki.ros.org/darknet_ros)**
- **[darknet_ros github](https://github.com/leggedrobotics/darknet_ros)**


### Installation of object tracker
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/gmkim97/object_tracker.git
$ cd ~/catkin_ws/
$ catkin_make
```

### Installation of pyrealsense2 module
```
$ sudo apt-get install python-pip
$ pip install pyrealsense2
```

### How to start?
```
$ roslaunch object_tracker object_tracking_with_py3.launch 
```



## Demo
![01](https://user-images.githubusercontent.com/97157046/148672885-48a8e897-bfbe-4a2a-919a-4c2032764f13.png)
![02](https://user-images.githubusercontent.com/97157046/148672887-552248d8-7032-4856-9e5e-ca13c49af0f0.png)
![03](https://user-images.githubusercontent.com/97157046/148672888-1fe60315-9d30-4985-9e04-c274aca6f891.png)
