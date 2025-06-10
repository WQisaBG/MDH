#!/bin/bash

cd ros_ws

sudo rm -rf build/ install/ logs/ 


colcon build 
