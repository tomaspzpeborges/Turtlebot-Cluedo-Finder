#!/bin/sh

#warning: moving this file will mess up the paths

#assumptions:
# all projectX.world and input_pointsX.yaml files are under catkin_ws/src/group_project/world
# all project_mapX (.yaml and .png) are under catkin_ws/src/group_project/world/map
# TURTLEBOT_GAZEBO_WORLD_FILE points to project.world

# relaunch gazebo after running script

read -p "enter map number (1-4, takes care of .world, mapping and input points): " number

cd $(pwd)/../../world/

if [ \( "$number" != "1" \) -a \( "$number" != "2" \) -a \( "$number" != "3" \) -a \( "$number" != "4" \) ]
then
   echo "Please enter a valid character"
else
    cp project$number.world project.world
    cp input_points$number.yaml input_points.yaml
    cd $(pwd)/../../world/map/
    cp project_map$number.yaml project_map.yaml
fi