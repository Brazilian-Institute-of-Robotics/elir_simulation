#!/bin/bash

#It is necessary to go robot urdf folder

export MYROBOT_NAME="elir"
#export URDF_LOCATION="/home/mega/catkin_ws/src/elir_tests/elir_description_openrave/urdf"
#export DAE_LOCATION="/home/mega/catkin_ws/src/elir_tests/elir_description_openrave/urdf"
rosrun xacro xacro --inorder -o "$MYROBOT_NAME".urdf "$MYROBOT_NAME".urdf.xacro
rosrun collada_urdf urdf_to_collada "$MYROBOT_NAME".urdf "$MYROBOT_NAME".dae
