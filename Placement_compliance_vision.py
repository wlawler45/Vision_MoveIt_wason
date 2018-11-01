import rospy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import numpy as np

import time

import general_robotics_toolbox as rox
from general_robotics_toolbox import urdf

from arm_composites_manufacturing_process import PayloadTransformListener
import tf
import actionlib
import copy
import general_robotics_toolbox.ros_msg as rox_msg
import sys

import rpi_abb_irc5.ros.rapid_commander as rapid_node_pkg
import safe_kinematic_controller.ros.commander as controller_commander_pkg
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal
from safe_kinematic_controller.msg import ControllerState 

FTdata = None
ft_threshold_place=[700,700,700,700,700,700]

        
def callback_function(data):
    global FTdata
    FTdata = np.array([data.ft_wrench.torque.x,data.ft_wrench.torque.y,data.ft_wrench.torque.z,\
    data.ft_wrench.force.x,data.ft_wrench.force.y,data.ft_wrench.force.z])
    
    
def main():
    step_ts = 0.004
    rospy.init_node("test_moveit_commander_custom_trajectory", anonymous=True)
    rospy.Subscriber("controller_state", ControllerState, callback_function)
    robot = urdf.robot_from_parameter_server()
    controller_commander=controller_commander_pkg.ControllerCommander()
    
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, [], [])
    time.sleep(0.5)
    tran = np.array([2.197026484647054, 1.2179574262842452, 0.12376598588449844])
    rot = np.array([[-0.99804142,  0.00642963,  0.06222524], [ 0.00583933,  0.99993626, -0.00966372], [-0.06228341, -0.00928144, -0.99801535]])
    pose_target2 = rox.Transform(rot, tran)
    pose_target2.p[2] += 0.20
    
    print 'Target:',pose_target2    
    print "============ Move Close to Panel"
    controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
    
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 1.0, [], [])
    time.sleep(1.0)
    Kc = 0.004
    time_save = []
    FTdata_save = []
    Tran_z = np.array([[0,0,-1],[0,-1,0],[1,0,0]])    
    Vec_wrench = 100*np.array([0.019296738361905,0.056232033265447,0.088644197659430,    
    0.620524934626544,-0.517896661195076,0.279323567303444,-0.059640563813256,   
    0.631460085138371,-0.151143175570223,-6.018321330845553]).transpose()
    
    listener=PayloadTransformListener()
    rapid_node = rapid_node_pkg.RAPIDCommander()
    #controller_commander=controller_commander_pkg.arm_composites_manufacturing_controller_commander()
    time.sleep(1.0)

    FTdata_0 = FTdata
    T = listener.lookupTransform("base", "link_6", rospy.Time(0))
    rg = 9.8*np.matmul(np.matmul(T.R,Tran_z).transpose(),np.array([0,0,1]).transpose())
    A1 = np.hstack([rox.hat(rg).transpose(),np.zeros([3,1]),np.eye(3),np.zeros([3,3])])
    A2 = np.hstack([np.zeros([3,3]),rg.reshape([3,1]),np.zeros([3,3]),np.eye(3)])
    A = np.vstack([A1,A2])
    FTdata_0est = np.matmul(A,Vec_wrench)
    #print 'Test4:',controller_commander.ControllerState

    for i in range(400):  
        tic = time.time()              
        plan=RobotTrajectory()    
        plan.joint_trajectory.joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        current_joint_angles = controller_commander.get_current_joint_values()
        
        plan.joint_trajectory.header.frame_id='/world'
        p1=JointTrajectoryPoint()
        p1.positions = current_joint_angles
        p1.velocities = np.zeros((6,))
        p1.accelerations = np.zeros((6,))
        p1.time_from_start = rospy.Duration(0)
        

        
        
        T = listener.lookupTransform("base", "link_6", rospy.Time(0))
        rg = 9.8*np.matmul(np.matmul(T.R,Tran_z).transpose(),np.array([0,0,1]).transpose())
        A1 = np.hstack([rox.hat(rg).transpose(),np.zeros([3,1]),np.eye(3),np.zeros([3,3])])
        A2 = np.hstack([np.zeros([3,3]),rg.reshape([3,1]),np.zeros([3,3]),np.eye(3)])
        A = np.vstack([A1,A2])
        FTdata_est = np.matmul(A,Vec_wrench)

        #print 'Test0:',FTdata,FTdata_0,FTdata_est,FTdata_0est
        FTread = FTdata-FTdata_0-FTdata_est+FTdata_0est
        print 'FTread:',FTread
        
        print 'FT:',FTdata
        print 'Z',FTread[-1]
        if FTread[-1]>-200:
            F_d = -350
        else:
            F_d = -400

        
        J = rox.robotjacobian(robot, current_joint_angles)
        Vz = Kc*(F_d - FTread[-1])
        joints_vel = np.linalg.pinv(J).dot(np.array([0,0,0,0,0,Vz]+dx))
        print 'Joint_command:',joints_vel.dot(step_ts)
                 
        p2=JointTrajectoryPoint()
        p2.positions = np.array(p1.positions) + joints_vel.dot(step_ts)#np.array([0,np.deg2rad(-2),0,0,0,0])
        p2.velocities = np.zeros((6,))
        p2.accelerations = np.zeros((6,))
        p2.time_from_start = rospy.Duration(step_ts)
        

        
        plan.joint_trajectory.points.append(p1)
        plan.joint_trajectory.points.append(p2)

        
        controller_commander.execute(plan)
        print 'Time:', time.time()-tic

        time_save.append(time.time())
        FTdata_save.append(FTread)
    
    filename = "FTdata.txt"
    f_handle = file(filename, 'a')
    np.savetxt(f_handle, FTdata_save)
    f_handle.close()   
    

    filename = "Time.txt"
    f_handle = file(filename, 'a')
    np.savetxt(f_handle, time_save)
    f_handle.close() 

if __name__ == '__main__':
    main()
