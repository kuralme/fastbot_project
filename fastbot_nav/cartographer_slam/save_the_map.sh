#!/bin/bash
cd ../map_server/config
ros2 run nav2_map_server map_saver_cli -f apartment_sim --ros-args -p save_map_timeout:=10000.0 -p resolution:=0.03