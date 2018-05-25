#import cv2
#import cv2.aruco as aruco
import numpy as np


import time
import timeit
import rpi_abb_irc5

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import rospkg
import numpy as np
from tf.transformations import *
from tf2_msgs.msg import TFMessage
import copy


from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    SetControllerMode, SetControllerModeRequest, SetControllerModeResponse



if __name__ == '__main__':
    print "============ Starting setup"
    Force_Measurement = 0
    P = [[ 1.8288, -0.0447, 1.237]]
    Q = [0.718181636243,-0.0836401543762,0.687115714468,0.0713544453462]
    
    Robot_Pos = []
    Robot_Joint = []
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('collision_checker','move_group_python_interface_tutorial',
                  anonymous=True)

  ## MoveIt! Initialization
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("move_group")
    group.set_goal_position_tolerance(0.04)
    group.allow_replanning(True)
    group.set_planner_id("RRTConnectkConfigDefault") #RRTConnectkConfigDefault/SBLkConfigDefault/KPIECEkConfigDefault/BKPIECEkConfigDefault/LBKPIECEkConfigDefault/
    group.set_num_planning_attempts(5)
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)    
    rospy.sleep(2)

    print "============ Printing robot Pose"
    print group.get_current_pose().pose

            

    tic = timeit.default_timer()
    dt = 0
    while dt< 3:
        toc = timeit.default_timer()
        dt = toc - tic
    print 'Start'
    
    set_controller_mode=rospy.ServiceProxy('set_controller_mode', SetControllerMode)

    req=SetControllerModeRequest()
    req.mode.mode=0
    req.speed_scalar=0.5
    req.force_torque_stop_threshold=[]

    req=SetControllerModeRequest()
    req.mode.mode=4
    req.speed_scalar=0.5
    req.force_torque_stop_threshold=[]

    res=set_controller_mode(req)
    if (not res.success): raise Exception("Could not set controller mode")
   
    print "============ Printing robot Pose"
    print group.get_current_pose()  
    #print robot.get_current_state().joint_state.position
    print "============ Generating plan 1"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = Q[1]
    pose_target.orientation.y = Q[2]
    pose_target.orientation.z = Q[3]
    pose_target.orientation.w = Q[0]
    pose_target.position.x = P[0][0]
    pose_target.position.y = P[0][1]
    pose_target.position.z = P[0][2]
    group.set_pose_target(pose_target)


    plan1 = group.plan()
    print plan1
    cnt = 0
    while( (not plan1.joint_trajectory.points) and (cnt<3)):
        print "============ Generating plan 1"
        plan1 = group.plan()
        cnt = cnt+1
        
    time.sleep(5)    
    print "============ Executing plan1"
    group.execute(plan1)
    print 'Execution Finished.'
  
       
