# I See You

Package that will allow you to control 4 separate dynamixels as if there were eyes! [Checkout the video [here](https://vimeo.com/user40509209/review/282109443/f283ae8254)]

Here's a sample pic of the eyes:

![](/pics/eyes.png)

## Hardware requirements
What you will need is:

* [Dynamixel AX-12A](https://www.trossenrobotics.com/dynamixel-ax-12-robot-actuator.aspx) (I used 4, but you can use as many as you want).
  
* [USB to Dynamixel converter](https://www.trossenrobotics.com/robotis-bioloid-usb2dynamixel.aspx).
  
* [6 Port AX/MX Power Hub](https://www.trossenrobotics.com/6-port-ax-mx-power-hub).
  
* 12 V power supply (at least 1 A).

* Webcam (The one used is a Logitech C930).

* Velodyne Lidar VL16 or Hokuyo Lidar UTM-30LX.

## Software requirements

This packages were tested with:

* Ubuntu 16.04 LTS.
  
* ROS kinetic with the following packages:

  * apriltags_ros
  
  * dynamixel control

  * pcl_ros

  * urg_node
  
  * velodyne

  * vision-opencv  

   
* CUDA 9.0.
  
* Tensorflow (version 1.5):

  * You will need to install [Tensorflow's Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection).

* OpenCV (for C++ and Python 2.7)
  
* Deep Learning Library ([DLib](https://sourceforge.net/projects/dclib/files/dlib/v19.10/dlib-19.10.tar.bz2/download)) for C++ (version 19.10).
  * If you want to use another version of DLib, you will need to change the CMakeLists.txt from eyecu package. Specifically change this line:
  ```cmake
  add_subdirectory($ENV{HOME}/cmake_workspace/dlib-<VERSION>/dlib dlib_build)
  ```

* Boost library.

* PointCloud Library (PCL).

### Notes on DLib
To compile the project with DLib, you'll need to do the following steps:

```console
foo@bar:~$ cd $HOME
foo@bar:~$ mkdir cmake_workspace && cd cmake_workspace
foo@bar:~$ wget http://dlib.net/files/dlib-19.15.tar.bz2
foo@bar:~$ tar -xf dlib-19.15.tar.bz2
foo@bar:~$ rm -rf dlib-19.15.tar.bz2 (optional)
```

## Launch the main file

Once you are able to compile, you can launch the whole system by typing:

```console
foo@bar:~$ roslaunch eyecu start_dynamixel_tracking_tensorflow.launch (for tensorflow)
or
foo@bar:~$ roslaunch eyecu start_dynamixel_tracking.launch (for OpenCV or Dlib)
```

## Notes on this version
 This version uses lidar (Velodyne VLP16 or Hokuyo UTM-30LX) to detect the distance of faces (or pedestrians). If you don't happen to have any of those, you can checkout **v1.4-color-ring**.

## Credits
This work was inspired by Lentin Joseph's [dynamixel tracking](https://github.com/qboticslabs/ros_robotics_projects/tree/master/chapter_2_codes) package.

