## uplan_visualization
This is a ROS package for visualization of Duckietown maps and duckiebots for Planning with Uncertainty project.

## Features
- [x] Visualization of maps from duckietown-world
- [x] Realtime visualization of duckiebots
- [x] Visualization of 10 future steps of duckiebot trajectory
- [x] Visualizing velocity profiler
- [x] Visualizing uncertainty probability field

## Prerequisites
- Desktop-Full installation of ROS
- [duckietown-world](https://github.com/duckietown/duckietown-world)
- [geometry](https://github.com/AndreaCensi/geometry)
- [duckietown-visualization] (https://github.com/surirohit/duckietown-visualization)
- [duckietown-uplan] (https://github.com/duckietown/duckietown-uplan)
- Other Python Packages: pip install colour

## Installing
From the `src` directory of your ROS Workspace, run
```
$ git clone (https://github.com/duckietown/duckietown-uplan)
```
From your workspace directory, run
```
$ catkin build 
```
Run `catkin_make` instead if you don't use `python-catkin-tools`.

Next, source your workspace using
```
## Running the map visualization
Run the visualization of the `robotarium1` map, which is currently the default 
by using
```
$ roslaunch uplan_visualization example.launch
```
[ You can specify different map names to be loaded according to the maps in 
`duckietown-world`.]

## How it works

To understand the working of this package, you need a basic understanding of the
[ROS Transform library](http://wiki.ros.org/tf2) and 
[RViz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker). This package reads
the map having name `map_name` from `duckietown-world` and broadcasts markers
([MESH_RESOURCE](http://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D)) 
for each element of the map. Each class of elements (tiles, road signs, etc.) is 
published in a different namespace under the transform `/map` to provide the 
feature of turning off a certain category of map elements.
[ROS Custom Messages] The package uses two custom messages: duckieData.msg and DuckieStruct.msg
More information found here: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

## Using uplan_visualization with your pipeline

Go through the publish_duckieData.py in the package ros-uplan folder to understand
how to use 'uplan-visualization`. Here you will populate the duckie trajectory/velocity and uncertainty values and publish them as markers to RViz, using uncertainty_planning package.

- First, create a list of duckiebot names which you want to visualize in a yaml 
file similar to `uplan-visualization/config/example.yaml`
- For each duckiebot, publish a transform from frame `duckiebot_link` to that 
duckiebot. `duckiebot_link` is a fixed frame which is the origin of your 
measurements for each duckiebot.
- In the publish_trajec.py and publish_uncertainty.py ROS nodes, populate the duckieStruct meesages and publish on corresponding topics.
- The trajectory is plotted using the line marker_array and uncertainty using Cubelist marker_array.









