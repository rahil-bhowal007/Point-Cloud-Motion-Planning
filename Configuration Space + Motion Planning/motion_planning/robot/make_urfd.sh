#!/bin/bash

#THE .urdf SHOULD ALREADY EXIST, IT SHOULD NOT BE NECESSARY TO RUN THIS
#runs command to build the urdf from the xacro
#requires ros
xacro simple.xacro > simple.urdf
#to visualize the results you can follow the ros urdf_tutorials
# roslaunch urdf_tutorial display.launch model:=simple.urdf
