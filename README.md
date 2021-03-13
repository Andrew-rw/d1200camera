# d1200camera ROS node
ROS wrapper for MYNT EYE D-1200 stereo camera.
ROS package was built as a result of experiments with Mynt Eye D-1200 depth camera. Improvements and pull requests are welcome!
The package contains the only node - **mynteye_d1200_node**. 

# Published topics

* **camera_color** - (sensor_msgs/Image) RGB image stream
* **camera_depth** - (sensor_msgs/Image) Depth image stream
* **points** - (sensor_msgs/PointCloud2) Registered Pointcloud stream, produced from depth image. 

# Parameters

* **rgb_topic** - (string) RGB image stream topic name. Default - "/camera_color"
* **depth_topic** - (string) Depth image stream topic name. Default - "/camera_depth"
* **points_topic** - (string) Pointcloud stream topic name. Default - "". If the the prameter value is empty or not set, topic publishing is disabled.
* **ir_intensity** - (int) IR structured light projector intensity. Default - "0". 0 - projector is disabled, 10 - maximum intensity.
* **camera_factor** - (float) Camera factor. Default - "500.0"
* **low_resolution** - (bool) Swith camera to low resolution (640x480). High resolution is 1280 720. Default - "false". In low resolution field of view is lower but higher data publish rate.
* **rgb_frame** - (string) RGB stream TF frame name. Default - "color_frame"
* **depth_frame** - (string) Depth stream TF frame name. Default - "depth_frame"
* **points_frame** - (string) Points TF frame name. Default - "points_frame"

# Dependencies
The package depends on Mynt Eye D library and pcl-ros package.

# Installation
1) Download and install MYNT-EYE-D-SDK (https://github.com/slightech/MYNT-EYE-D-SDK)
2) Download and install ROS package:
```
$ mkdir -p mynteye_ws/src
$ cd mynteye_ws/src
$ git clone https://github.com/Andrew-rw/d1200camera.git
$ cd ..
$ rosdep update
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make
$ source /devel/setup.bash
```
3) Run demo:
```
$ roslaunch d1200camera demo_pointcloud.launch
```
4) To run RTABMAP demo install **rtabmap-ros** first 


# Links

* MYNT EYE D-1200 product page - https://www.mynteye.com/pages/d12000
* SDK documentation - https://mynt-eye-d-sdk.readthedocs.io/en/latest/
* MYNT EYE D-1200 sensor review - https://medium.com/robotics-weekends/mynt-eye-d1200-3d-camera-first-look-852da6389c42

