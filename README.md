# Dense Visual Odometry and SLAM (dvo_slam)

These packages provide an implementation of the rigid body motion estimation of an RGB-D camera from consecutive images.

 *  **dvo_core**: Core implementation of the motion estimation algorithm. 
    
 *  **dvo_ros**: Integration of *dvo_core* with ROS.
    
 *  **dvo_slam**: Pose graph SLAM system based on *dvo_core* and integration with ROS.
    
 *  **dvo_benchmark**: Integration of *dvo_slam* with TUM RGB-D benchmark, see [TUM RGB-D Dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset).

## Dependencies
### sophus
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout b474f05
mkdir build
cd build
cmake ..
make
sudo make install 
```

Note: As of Dec 2016, the master branch of Sophus seems no longer compatible for building DVO SLAM. Here we use an older version of Sophus, commit b474f05, to build.

### g2o 
Recent g2o starts using C++11, which causes trouble when DVO links to the default PCL provided by Ubuntu. This PCL build does not have C++11 support. 

It is best to use an old version of g2o instead, which does not require C++11, so we don't need to rebuild PCL with C++11. The suitable g2o version is commit 67d5fa7.
(To speed up the g2o compilation, you can skip building g2o apps and examples, which are not necessary in our case.)

```
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 67d5fa7
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_LGPL_SHARED_LIBS:BOOL=OFF -DG2O_BUILD_APPS:BOOL=OFF -DG2O_BUILD_EXAMPLES:BOOL=OFF
make
sudo make install
```

If your system has Eigen >= 3.3, don't use it with g2o. The old version of g2o is only compatible to Eigen 3.2.8. 
Download Eigen 3.2.8, put it into the EXTERNAL folder in g2o, and use this CMake command instead:

```
cmake .. -DBUILD_SHARED_LIBS:BOOL=OFF -DBUILD_LGPL_SHARED_LIBS:BOOL=OFF -DG2O_BUILD_APPS:BOOL=OFF -DG2O_BUILD_EXAMPLES:BOOL=OFF -DG2O_EIGEN3_INCLUDE=/home/sutd/Workspace/g2o/EXTERNAL/eigen3.2.8/

```
This will force g2o to use our Eigen library instead of the system Eigen.

We have to build g2o into static libraries 
and build DVO SLAM also as static libraries to avoid some weird undefined reference to g2o::csparse_extension. It could be possible to build as shared libraries (by default) but somehow it does not work on my machine.

## Installation

### Catkin workspace

This version uses catkin to build the code, which makes things much easier. 

The Catkin workspace is organized as follows: 

```
dvo_slam  (Catkin workspace root)
    src 
        dvo_core
            CMakeLists.txt
            src
        dvo_ros
            CMakeLists.txt
            src
        dvo_slam
            CMakeLists.txt
            src
        dvo_benchmark
            CMakeLists.txt
            src
    README.md
```

where dvo_slam is the root of the Catkin workspace and has 4 packages: dvo_core, dvo_ros, dvo_slam, and dvo_benchmark.

### Build

 *  **ROS Indigo**

	Go back to Catkin workspace's root folder and execute
	```
	git clone -b indigo-devel git://github.com/howard-mahe/dvo_slam.git
	cd dvo_slam
	catkin_make -DCMAKE_BUILD_TYPE=Release
	source devel/setup.bash
	```

	to build all packages. You can also build each package separately by 

	```
	catkin_make --pkg dvo_core -DCMAKE_BUILD_TYPE=Release
	catkin_make --pkg dvo_ros -DCMAKE_BUILD_TYPE=Release
	catkin_make --pkg dvo_slam -DCMAKE_BUILD_TYPE=Release
	catkin make --pkg dvo_benchmark -DCMAKE_BUILD_TYPE=Release
	```

## Usage

### Estimating the camera trajectory from a TUM RGB-D sequence

Go to *dvo_benchmark* source folder:
```
mkdir -p output
roslaunch launch/benchmark.launch dataset:=<RGBD dataset folder>
```

Be sure that you have an *assoc.txt* file in your `<RGBD dataset folder>`. Otherwise, you can download the [associate.py](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools) script on TUM RGB-D Dataset tools page.

### Estimating the camera trajectory from an RGB-D image stream

Considering you are already playing a rosbag or acquiring a RGB-D livestream. Run the **camera_keyframe_tracker** node:
```
rosrun dvo_slam camera_keyframe_tracker
```

**camera_keyframe_tracker** node expects following *sensors_msgs::Image* topics:
 *	*/camera/rgb/image_rect* (either grey or color)
 * 	*/camera/depth_registered/image_rect_raw*
so you may need usage of [<remap>](http://wiki.ros.org/roslaunch/XML/remap) tag in a launch file, i.e.
	```
	<!-- DVO SLAM node -->
  	<node pkg="dvo_slam" type="camera_keyframe_tracker" name="dvo_slam" output="screen">
    	<!-- Input remapping -->
    	<remap from="/camera/depth_registered/image_rect_raw" to="/camera/depth_registered/image_rect"/>
    	<remap from="/camera/rgb/image_rect" to="/camera/rgb/image_rect_color"/>
	</node>
	```


Dynamic reconfigure GUI will allow you to set true **run_dense_tracking** and **use_dense_tracking_estimate** parameters of */dvo_slam/tracking*.
```
rosrun rqt_reconfigure rqt_reconfigure
```
You could also set these parameters in a launch file with [dynamic_reconfigure/dynparam](http://wiki.ros.org/dynamic_reconfigure#dynamic_reconfigure.2BAC8-indigo.dynparam_command-line_tool) nodelet.

### Visualization
 *  Start RVIZ
 *  Set the *Target Frame* to `/world`
 *  Add an *Interactive Marker* display and set its *Update Topic* to `/dvo_vis/update` (`/update` with *camera_keyframe_tracker* node)
 *  Add a *PointCloud2* display and set its *Topic* to `/dvo_vis/cloud` (`/cloud` with *camera_keyframe_tracker* node)

The red camera shows the current camera position. The blue camera displays the initial camera position.

## Credits
This work is clone from jade-devel branch of **tum-vision/dvo_slam** repository, with refactoring of **songuke/dvo_slam** contributions to support ROS Indigo and Ubuntu Trusty 14.04 LTS.
 * [tum-vision/dvo_slam](https://github.com/tum-vision/dvo_slam/)
 * [songuke/dvo_slam](https://github.com/songuke/dvo_slam/)

Note: the graph solver type is *g2o::LinearSolverEigen* (as in **tum-vision/dvo_slam**) instead of *g2o::LinearSolverCSparse* (as in **songuke/dvo_slam**)
