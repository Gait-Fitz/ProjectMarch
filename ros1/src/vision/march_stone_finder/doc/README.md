# Stone finder

In the 2022 Cybathlon Challenge, M7 was given the challenge to walk on circular stones that were randomly placed on a grid.
One of the criteria was that the floor between the stones could not be touched. 
As the distances between the stones were variable, this package was created to detect the centers of the circular stones in 3D space.
Using dynamic gaits based on these centers, the exoskeleton can perform the variable steps between the stones. 

The package makes use of RealSense cameras and uses RGB color frames, as well as depth frames.
Image processing is first performed on the RGB color frames using the `opencv-python` library to detect ellipses of certain colors.
The actual position of the ellipse centers is computed from the pointclouds in the depth frames. 

## ROS API
### How to build

```bash
source /opt/ros/noetic/local_setup.bash
cd /march/ros1/
colcon build --packages-select march_stone_finder
```  

### How to run

The node is automatically turned off, and can be turned on with the ROS1 launch argument

```bash
stone_finder:=true
```

The node uses two RealSense cameras specified by two serial numbers. These numbers are defined in the `stone_finder_node.py` file as:
```python
LEFT_SERIAL_NUMBER = "944622074337"
RIGHT_SERIAL_NUMBER = "944622071535"
```
Replace these serial numbers and build the package if you plan to use different cameras.

### Nodes
`march_stone_finder` <br>
Responsible for obtaining the depth and color frames, and processing them to find the centers of ellipses. 

### Published topics
`"/march/foot_position/(left|right)"  (FootPosition)` <br>
The node publishes the centers of ellipses as 3D points on this topic.

`"/march/stone_finder/camera_(left|right)/found_points"  (MarkerArray)` <br>
Markers for visualizing where the points are located in rviz. 

`"/march/stone_finder/pointcloud/(left|right)"  (PointCloud2)` <br>
The pointclouds as seen by the depth cameras, for visualization in rviz.

## Functionality

The process as described below is how the node searches for ellipse centers using one camera, and is thus used to find points for one leg.
In parallel, the same process in run using the other camera for the other leg.

### 1. Preprocessing
When depth frames and color frames are collected, they are first aligned so that RGB pixels and depth pixels overlap. The depth frame is afterwards converted to a point cloud, and the color frame is preprocessed with a Gaussian blur filter.
  
### 2. Color Segmentation
Color segmentation is used to *select* pixels that belong to the stones, and is used to *remove* pixels that belong to the base plate.
Selection of colors is done with the `cv2.inRange(image, lower_range, upper_range)` function, with which colors between a certain HSV range can be detected.  
This process results in a black and white image, where all pixels with stone colors are white, and all other pixels are black.

### 3. Connected Components
Afterwards, connected components of at least a certain pixel size (such as 1000) are computed from the color segmented image. This results in clusters of pixels that probably represent the same object.

### 4. Contour Finding
Contour finding is performed on the connected components, to determine the outlines of the components.

### 5. Ellipse Fitting
Ellipse functions are fit to the found contours of the connected components. As not all components are ellipse shaped, we use a similarity metric to determine to what extent a contour of a component is ellipse shaped. Components with a high ellipse similarity are filtered out, and their centers are stored as depth points.

### 6. Point Filtering
Finally, we determine which center is closest to the depth camera, as this center usually represents the stone that is the closest. This center is published on a ROS topic, so the dynamic gaits module can use it.


## Tutorials
These tutorials use the convenient aliases.

### Running in simulation with real cameras

This package can be run in simulation, as long as both the RealSense cameras with the correct serial numbers are connected to the computer.

It is recommended to run all commands in separate terminals, to prevent sourcing problems between ROS1 & ROS2.
```bash
march_run_ros1_sim stone_finder:=true realsense_simulation:=false # T1
march_run_bridge # T2
march_run_ros2_sim #T3
```

This will start rviz, where you can visualize what the cameras see. The centers of ellipses found with the cameras can be visualized by adding a `MarkerArray` entity, subscribed to the topics `"/march/stone_finder/camera_(left|right)/found_points"`.
The point clouds of the cameras can also be visualized by adding a `PointCloud2` entity, subscribed on the topics `"/march/stone_finder/pointcloud/(left|right)"`.

### Running on a MARCH exoskeleton using real cameras
Launch by:
```bash
march_run_ros1_(airgait|groundgait|koengait) stone_finder:=true # T1
march_run_bridge # T2
march_run_ros2_training # T3
```

To visualise the packages debug output, start rviz:
```bash
snoe && sros1 && rosrun rviz rviz
```
and add the same entities as described in the previous paragraph.
The exoskeleton can perform steps towards the found points, as long as dynamic gaits are enabled.
