# Task Details
## Corrections in Robot Description file
1. The lidar orientation is shifted 90 degrees. Set it to 0 yaw.
2. The wheel joints are defined as fixed type of joints. Corrected to continous joints and added axis of rotation (y-axis).
3. The base_footprint can be added to avoid KDL tree warning stating base node(frame) cannot consists of inertia.

## Mapping the environment
- Used **gmapping** package to map the environment.
- Used **teleop** to move robot around the environnment.
- Used **map_server** package to save the map.
- Later will edit the map to avoid unneccesary path or behaviours while planning.

## Naviagtion Stack
- Used **AMCL** package to localize the robot.
- Used **move_base** package as navigation stack.
- Used **DWA Local Planner** as local planner.
- Defined local costmap params, gloabal costmap params, move_base_params all in yaml file. Need to tune these parameters as per techinical details of robot.
- Robot footprint is defined.

## Script for Goal Publishing
- Created a C++ node for publishing the goals in sequential order (goal_publisher_node.cpp ) which are stored in 2D vector. 
- Will update to ros parameter so can send dynamically.

## Script for visualization markers RVIZ
- Created a C++ node (path_marker.cpp), which marks its path in green rectangular area. 
- The old path will be cleared once a new goal is recieved.
