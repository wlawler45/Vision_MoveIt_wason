import rospy
import numpy as np
import general_robotics_toolbox as rox
import tf
import time

def main():
    rospy.init_node("compute_leeward_mid_panel_target", anonymous=True)
    
    listener = tf.TransformListener()
    time.sleep(1)

    def tf_lookup(target_frame, source_frame):
        (trans1,rot1) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        return rox.Pose(rox.q2R([rot1[3], rot1[0], rot1[1], rot1[2]]), trans1)
       
    nest_pose=tf_lookup("world","panel_nest")
    gripper_pose=tf_lookup("world", "vacuum_gripper_tool")
    panel_rel_target_pose=tf_lookup( "leeward_mid_panel", "leeward_mid_panel_gripper_target")
    
    panel_target_pose=gripper_pose*(panel_rel_target_pose.inv())
    #print panel_target_pose.R
    #print panel_target_pose.p
    
    nest_target_pose=(nest_pose.inv())*panel_target_pose
    print "<origin xyz=\"" + ' '.join([str(x) for x in nest_target_pose.p]) + "\" rpy=\"" \
      + ' '.join([str(x) for x in tf.transformations.euler_from_matrix(nest_target_pose.R)]) + "\"/>"

if __name__ == '__main__':
    main()