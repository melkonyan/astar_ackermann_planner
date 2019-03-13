An implementation of A* path planning algorithm that takes into account non-holonomic constraints of the robot. Ideas behind it are described in the Stanford [paper](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf). This repositroy also contains a plugin that allows to use the planner with [ROS navigation stack](http://wiki.ros.org/navigation)

## Requirements
To build this repository you need ROS, the code has been tested with *kinetic* version. Also, you need to install ROS Navigation Stack, to install on a system level, run `sudo apt-get install ros-kinetic-navigation` ` 

## Setup
1. Copy this repository into your catkin workspace.
2. Build your workspace and `source devel/setup.bash`
3. Check that the plugin exists: 


```rospack plugins --attrib=plugin nav_core | grep "astar_ackermann_planner"```

## Usage 
1. Create a config file for the planner, an example:
```python
AStarAckermannPlanner:
  default_tolerance: 0.5
  turning_radius: 8.0
  step_size: 0.2
  max_allowed_time: 20
```
2. In your launch file 
```xml
<launch>
  ...
  <node pkg="move_base" type="move_base" name="move_base">
    ...
    <rosparam file="path-to-planner-config-file.yaml" command="load"/>
    <param name="base_global_planner" value="astar_ackermann_planner::AStarAckermannPlanner"/>
</node>
</launch>
```
## Testing
**Note**: make sure that roscore is running, otherwise testing will run forever.
 
Run ```catkin build astar_ackermann_planner --no-deps --catkin-make-args run_tests```
 
To check results run ```catkin_test_results build```
 

