import numpy as np
import copy
import rospy
import actionlib

import general_robotics_toolbox as rox
import general_robotics_toolbox.urdf as urdf
import general_robotics_toolbox.ros_msg as rox_msg
from general_robotics_toolbox import ros_tf as tf

import abb_irc5_rapid_node_commander as rapid_node_pkg
import arm_composites_manufacturing_controller_commander as controller_commander_pkg

from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal

import time
import sys

ft_threshold=[250,250,250,250,250,250]

class ObjectRecognitionCommander(object):
    def __init__(self):
        self.client=actionlib.SimpleActionClient("recognize_objects", ObjectRecognitionAction)
        self.listener=tf.TransformListener()
    
    def get_object_pose(self, key):
        self.client.wait_for_server()
        
        goal=ObjectRecognitionGoal(False, [-1e10,1e10,-1e10,1e10])
        self.client.send_goal(goal)
        self.client.wait_for_result()
        ret=self.client.get_result()
        
        for r in ret.recognized_objects.objects:
            if r.type.key == key:
                return rox_msg.msg2transform(r.pose.pose.pose)
            
        raise Exception("Requested object not found")
    
    def get_object_gripper_target_pose(self, key):
        
        object_pose=self.get_object_pose(key)
                
        tag_rel_pose = self.listener.lookupTransform(key, key + "_gripper_target", rospy.Time(0))        
        return object_pose * tag_rel_pose
        

def main():
            
    t1=time.time()
    
    do_place="place-panel" in sys.argv
    
    if not "disable-ft" in sys.argv:
        ft_threshold1=ft_threshold
    else:
        ft_threshold1=[]
    
    rospy.init_node('Vision_MoveIt_new_Cam_wason2', anonymous=True)
    
    print "============ Starting setup"   
    
    listener=tf.TransformListener()
    
    rapid_node = rapid_node_pkg.AbbIrc5RAPIDNodeCommander()
    controller_commander=controller_commander_pkg.arm_composites_manufacturing_controller_commander()
    
    object_commander=ObjectRecognitionCommander()
    
    object_target=object_commander.get_object_gripper_target_pose("leeward_mid_panel")
    
    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 1.2, ft_threshold1)  
    
    print "============ Printing robot Pose"
    print controller_commander.get_current_pose_msg()  
    #print robot.get_current_state().joint_state.position
    print "============ Generating plan 1"
    
    pose_target=copy.deepcopy(object_target)
    pose_target.p[2] += 0.5
    
    print 'Target:',pose_target
    
    print "============ Executing plan1"
    controller_commander.plan_and_move(pose_target)        
    print 'Execution Finished.'
    
    ########## Vertical Path 1 ############

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.8, ft_threshold1)
    
    print "============ Printing robot Pose"
    print controller_commander.get_current_pose_msg()  
    print "============ Generating plan 2"

    pose_target2=copy.deepcopy(object_target)
    pose_target2.p[2] += 0.15
    
    print 'Target:',pose_target2
    
    print "============ Executing plan2"
    controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
    print 'Execution Finished.'
        
    ########## Vertical Path 2 ############

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, ft_threshold1)
    
    print "============ Printing robot Pose"
    print controller_commander.get_current_pose_msg()  
    print "============ Generating plan 3"

    pose_target2=copy.deepcopy(object_target)
    pose_target2.p[2] -= 0.15
    
    print 'Target:',pose_target2
    
    print "============ Executing plan3"
    try:
        controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
    except:
        pass
    print 'Execution Finished.'
    
    ########## Lift Path ############

    controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.7, [])

    print "============ Lift panel!"
            
    rapid_node.set_digital_io("Vacuum_enable", 1)
    time.sleep(0.5)
    pose_target3=copy.deepcopy(object_target)
    pose_target3.p[2] += 0.5
            
    
    print 'Target:',pose_target3
    
    print "============ Executing plan4"
    controller_commander.compute_cartesian_path_and_move(pose_target3, avoid_collisions=False)
    
        
    
    if (do_place):
        print "=========== Do place!"
        print ""
        
        controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 1.2, ft_threshold1)
        
        print "============ Generating plan 5"
        
        panel_target_pose = listener.lookupTransform("world", "panel_nest_leeward_mid_panel_target", rospy.Time(0))        
        panel_gripper_target_pose = listener.lookupTransform("leeward_mid_panel", "leeward_mid_panel_gripper_target", rospy.Time(0))        
        pose_target=panel_target_pose * panel_gripper_target_pose
        print pose_target.R
        print pose_target.p               
        
        pose_target2=copy.deepcopy(pose_target)
        pose_target2.p[2] += 0.5
        
        print "============ Executing plan 5"
        controller_commander.plan_and_move(pose_target2)
        
        controller_commander.set_controller_mode(controller_commander.MODE_AUTO_TRAJECTORY, 0.4, ft_threshold1)
    
        print "============ Printing robot Pose"
        print controller_commander.get_current_pose_msg()  
        print "============ Generating plan 3"
    
        pose_target2=copy.deepcopy(pose_target)
        pose_target2.p[2] -= 0.15
        
        print 'Target:',pose_target2
        
        print "============ Executing plan3"
        try:
            controller_commander.compute_cartesian_path_and_move(pose_target2, avoid_collisions=False)
        except:
            pass
        print 'Execution Finished.'      
        
        
        print "============ Lift gripper!"
            
        rapid_node.set_digital_io("Vacuum_enable", 0)
        time.sleep(0.5)
        pose_target3=copy.deepcopy(pose_target)
        pose_target3.p[2] += 0.5
                
        
        print 'Target:',pose_target3
        
        print "============ Executing plan4"
        controller_commander.compute_cartesian_path_and_move(pose_target3, avoid_collisions=False)
        
        
        
    t2=time.time()
    print 'Execution Finished.'
    print "Execution time: " + str(t2-t1) + " seconds"

if __name__ == '__main__':
    main()
