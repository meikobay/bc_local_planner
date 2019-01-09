# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/indigo/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/indigo/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/toby/catkin_ws_hu/devel;/program/robot_behavior/car_sim_ws/devel;/program/robot_behavior/jdrobot_ws/devel;/program/robot_behavior/robot_lfd/devel;/program/robot_behavior/cartographer_turtlebot_ws/devel;/program/robot_behavior/cartographer_ros_ws/install_isolated;/program/robot_arm/devel;/program/sim_ugv_ws/devel;/program/AiRobot/devel;/opt/ros/indigo".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/program/sim_ugv_ws/src/sim_ugv_master/4Path_Planning/bc_local_planner/build/devel/env.sh')

output_filename = '/program/sim_ugv_ws/src/sim_ugv_master/4Path_Planning/bc_local_planner/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
