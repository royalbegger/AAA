#!/home/zoow/miniconda3/envs/barn/bin/python
import time
import argparse
import subprocess
import os
from os.path import join

import numpy as np
import rospy
import rospkg

from gazebo_simulation import GazeboSimulation

#INIT_POSITION = [-2, 3, 1.57]  # in world frame -2 3 1.57
#GOAL_POSITION = [0, 10]  # relative to the initial position

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'test BARN navigation challenge')
    parser.add_argument('--world_idx', type=int, default=0)
    parser.add_argument('--gui', action="store_true")
    parser.add_argument('--out', type=str, default="VFHout.txt")
    args = parser.parse_args()
    
    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################
    
    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"
    
    if args.world_idx < 300:  # static environment
        world_name = "BARN/world_%d.world" % args.world_idx
        INIT_POSITION = [-2, 3, 1.57]  # in world frame
        GOAL_POSITION = [0, 10]  # relative to the initial position

        rospack = rospkg.RosPack()
        base_path = rospack.get_path('jackal_helper')
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")

    elif 300 <= args.world_idx <= 319:  # dynamic easy
        world_name = "DynaBARN/easy/world_%d.world" % (args.world_idx - 300)
        INIT_POSITION = [-9, 0, 3.14]
        GOAL_POSITION = [18, 0]

        rospack = rospkg.RosPack()
        base_path = rospack.get_path('jackal_helper')
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins", "easy")

    elif 320 <= args.world_idx <= 339:  # dynamic hard
        world_name = "DynaBARN/hard/world_%d.world" % (args.world_idx - 320)
        INIT_POSITION = [-9, 0, 3.14]
        GOAL_POSITION = [18, 0]


        rospack = rospkg.RosPack()
        base_path = rospack.get_path('jackal_helper')
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins", "hard")

    elif 340 <= args.world_idx <= 359:  # dynamic crazy
        world_name = "DynaBARN/crazy/world_%d.world" % (args.world_idx - 340)
        INIT_POSITION = [12, 0, 3.14]
        GOAL_POSITION = [-20, 0]


        rospack = rospkg.RosPack()
        base_path = rospack.get_path('jackal_helper')
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins", "crazy")

    else:
        raise ValueError("World index %d does not exist" % args.world_idx)


    print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" % world_name)

    launch_file = join(base_path, 'launch', 'gazebo_launch.launch')
    world_name = join(base_path, "worlds", world_name)
    
    gazebo_process = subprocess.Popen([
        'roslaunch',
        launch_file,
        'world_name:=' + world_name,
        'gui:=' + ("true" if args.gui else "false")
    ])
    time.sleep(5)  # sleep to wait until the gazebo being created
    
    rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
    rospy.set_param('/use_sim_time', True)
    # *** Set the initial and goal positions as ROS parameters ***
    rospy.set_param('init_position', INIT_POSITION)
    rospy.set_param('goal_position', GOAL_POSITION)

    # GazeboSimulation provides useful interface to communicate with gazebo  
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    
    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
    
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)
    collided = True
    
    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset() # Reset to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        time.sleep(1)
    
    # gazebo_process.terminate()
    # gazebo_process.wait()

