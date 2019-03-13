Testing:
 
 ```catkin build astar_planner --no-deps --catkin-make-args run_tests```
 
 ```catkin_test_results build```
 
Note: make sure that roscore is running, otherwise testing will run forever.

Using: 
1. check that plugin exists: `rospack plugins --attrib=plugin nav_core | grep "astar_planner"` 
2. In move_base node: 