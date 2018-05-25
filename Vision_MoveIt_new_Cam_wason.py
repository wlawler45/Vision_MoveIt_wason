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

import rpi_ati_net_ft
from CameraService import *

from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    SetControllerMode, SetControllerModeRequest, SetControllerModeResponse
    
from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    RapidStart, RapidStartRequest, RapidStartResponse, \
    RapidStop, RapidStopRequest, RapidStopResponse, \
    RapidGetStatus, RapidGetStatusRequest, RapidGetStatusResponse, \
    RapidGetDigitalIO, RapidGetDigitalIORequest, RapidGetDigitalIOResponse, \
    RapidSetDigitalIO, RapidSetDigitalIORequest, RapidSetDigitalIOResponse, \
    RapidReadEventLog, RapidReadEventLogRequest, RapidReadEventLogResponse

ft_threshold=[250,250,250,250,250,250]

if __name__ == '__main__':
    print "============ Starting setup"
    Force_Measurement = 0
    P,Q = CameraService()
    '''
    if(Force_Measurement):
        if (len(sys.argv) < 2):
            raise Exception('IP address of ATI Net F/T sensor required')
        host=sys.argv[1]
        netft=rpi_ati_net_ft.NET_FT(host)
        netft.set_tare_from_ft()
        print netft.try_read_ft_http()

        netft.start_streaming()
        FTtime = []
        FTread = []
    '''
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
    #ROS service calls to check system variables
    set_controller_mode=rospy.ServiceProxy('set_controller_mode', SetControllerMode)
    set_digital_io=rospy.ServiceProxy('rapid/set_digital_io', RapidSetDigitalIO)
    
    req=SetControllerModeRequest()
    req.mode.mode=4
    req.speed_scalar=1
    req.force_torque_stop_threshold=ft_threshold
    
    res=set_controller_mode(req)
    if (not res.success): raise Exception("Could not set controller mode")
        
    rospy.sleep(2)

    print "============ Printing robot Pose"
    print group.get_current_pose().pose

            

    tic = timeit.default_timer()
    dt = 0
    while dt< 3:
        toc = timeit.default_timer()
        dt = toc - tic
    print 'Start'
       
	
    if (1):

    
        print "============ Printing robot Pose"
        print group.get_current_pose()  
        #print robot.get_current_state().joint_state.position
        print "============ Generating plan 1"
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = Q[1]
        pose_target.orientation.y = Q[2]#0.707
        pose_target.orientation.z = Q[3]#0.707
        pose_target.orientation.w = Q[0]#qoa[3] #0#0
        pose_target.position.x = P[0][0]
        pose_target.position.y = P[0][1]#-2.02630600362
        pose_target.position.z = P[0][2] + 0.3
        group.set_pose_target(pose_target)

        '''
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = -0.02892
        pose_target.position.y = -1.79035
        pose_target.position.z = 0.914083
        pose_target.orientation.x = 0.426720076618065
        pose_target.orientation.y = 0.5339800423981502
        pose_target.orientation.z = -0.4531605121430878
        pose_target.orientation.w = 0.5722069911891644

        group.set_pose_target(pose_target)
        '''
        print 'Target:',pose_target

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
        
        plan1 = group.plan()
        print plan1
        cnt = 0
        while( (not plan1.joint_trajectory.points) and (cnt<3)):
            print "============ Generating plan 1"
            plan1 = group.plan()
            cnt = cnt+1
            
        raw_input("Press Enter to continue")    
        print "============ Executing plan1"
        group.execute(plan1)
        print 'Execution Finished.'
        
        ########## Vertical Path ############

        req=SetControllerModeRequest()
        req.mode.mode=4
        req.speed_scalar=0.4
        req.force_torque_stop_threshold=ft_threshold
        
        res=set_controller_mode(req)
        if (not res.success): raise Exception("Could not set controller mode")

        print "============ Printing robot Pose"
        print group.get_current_pose()  
        print "============ Generating plan 2"
        """pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = Q[1]
        pose_target.orientation.y = Q[2]#0.707
        pose_target.orientation.z = Q[3]#0.707
        pose_target.orientation.w = Q[0]#qoa[3] #0#0
        pose_target.position.x = P[0][0]
        pose_target.position.y = P[0][1]#-2.02630600362
        pose_target.position.z = P[0][2] + 0.2"""

        pose_target2 = copy.deepcopy(pose_target)
        pose_target2.position.z -= 0.45

        group.set_pose_target(pose_target2)
        
        print 'Target:',pose_target2

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
        
        plan2 = group.plan()
        cnt = 0
        while( (not plan2.joint_trajectory.points) and (cnt<3)):
            print "============ Generating plan 2"
            plan2 = group.plan()
            cnt = cnt+1
            
        time.sleep(1)    
        print "============ Executing plan2"
        group.execute(plan2)
        print 'Execution Finished.'

        req=SetControllerModeRequest()
        req.mode.mode=4
        req.speed_scalar=0.7
        req.force_torque_stop_threshold=[]
        
        res=set_controller_mode(req)
        if (not res.success): raise Exception("Could not set controller mode")

        print "============ Lift panel!"
        
        req=RapidSetDigitalIORequest()
        req.signal="Vacuum_enable"
        req.lvalue=1
        set_digital_io(req)
        
        pose_target3 = copy.deepcopy(pose_target)
        pose_target3.position.z += 0.25

        group.set_pose_target(pose_target3)
        
        print 'Target:',pose_target3

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
        
        plan3 = group.plan()
        cnt = 0
        while( (not plan3.joint_trajectory.points) and (cnt<3)):
            print "============ Generating plan 3"
            plan3 = group.plan()
            cnt = cnt+1
            
        time.sleep(1)    
        print "============ Executing plan3"
        group.execute(plan3)
        print 'Execution Finished.'


        '''
        tic = timeit.default_timer()
        dt = 0
        while dt< 3:
            toc = timeit.default_timer()
            dt = toc - tic
            a_pos = group.get_current_pose().pose.position
            a_ori = group.get_current_pose().pose.orientation
            a_joint = robot.get_current_state().joint_state.position
            
            if(Force_Measurement):
                s = netft.try_read_ft_streaming(.1)
                FTread.append(s[1])
                FTtime.append(time.time())
		        
            Robot_Pos.append([a_pos.x, a_pos.y, a_pos.z,a_ori.x, a_ori.y, a_ori.z,a_ori.w])
            Robot_Joint.append(a_joint[0:6])
            
    # When everything done, release the capture
    if (Force_Measurement):
                np.savetxt('FTtime.out', FTtime, delimiter=', ')
                np.savetxt('FTread.out', FTread, delimiter=', ')
    np.savetxt('Robot_Pos.out', Robot_Pos, delimiter=', ')
    np.savetxt('Robot_Joint.out', Robot_Joint, delimiter=', ')
    '''

