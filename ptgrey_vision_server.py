import rospy
from actionlib import SimpleActionServer
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionResult, \
     RecognizedObject, RecognizedObjectArray
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from sensor_msgs.msg import CompressedImage, CameraInfo
import numpy as np
import cv2
from std_srvs.srv import Trigger
import time
import general_robotics_toolbox as rox
from general_robotics_toolbox import ros_msg as rox_msg

class SimulatedVisionServer(object):
    
    def ros_image_cb(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.ros_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.ros_image_stamp = ros_data.header.stamp
        
    def ros_cam_info_cb(self, ros_data):
        self.ros_cam_info=ros_data
        self.camMatrix=np.reshape(ros_data.K,(3,3))
        self.distCoeffs=np.array(ros_data.D)
        if len(self.distCoeffs) < 4:
            self.distCoeffs=np.array([[0,0,0,0,0]])      
    
    def __init__(self, object_names, frame_id="world", action_ns="recognize_objects"):        
                
        self.server=SimpleActionServer(action_ns, ObjectRecognitionAction, execute_cb=self.execute_callback)
        self.recognized_objects=dict()
        self.object_names=object_names
        self.listener=tf.TransformListener()
        self.frame_id="world"
        self.ros_image=None
        self.ros_image_stamp=rospy.Time(0)
        self.last_ros_image_stamp=rospy.Time(0)
        self.ros_cam_info=None
        self.ros_img_sub=rospy.Subscriber('/rviz_sim_cameras/overhead_camera/image/compressed', CompressedImage, self.ros_image_cb)
        self.ros_cam_trigger=rospy.ServiceProxy('/rviz_sim_cameras/overhead_camera/camera_trigger', Trigger)
        self.ros_cam_info_sub=rospy.Subscriber('/rviz_sim_cameras/overhead_camera/camera_info', CameraInfo, self.ros_cam_info_cb)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.gripper_board=cv2.aruco.GridBoard_create(2, 2, .0972, .005, self.aruco_dict, 1)
        self.panel_board=cv2.aruco.GridBoard_create(2, 2, .0972, .005, self.aruco_dict, 5)
        self.boards=[self.gripper_board, self.panel_board]
        self.object_ids=["vacuum_gripper_tool", "leeward_mid_panel"]
        self.tag_ids=["vacuum_gripper_marker_1","leeward_mid_panel_marker_1"]
        
    def execute_callback(self, goal):
        
        now=rospy.Time.now()
        
        if self.ros_cam_info is None:
            raise Exception("Camera Info not received")
        
        try:
            self.ros_cam_trigger.wait_for_service(timeout=0.1)
            self.ros_cam_trigger()            
        except:
            pass
        
        wait_count=0
        while self.ros_image is None or self.ros_image_stamp == self.last_ros_image_stamp:
            if wait_count > 250:
                raise Exception("Image receive timeout")
            time.sleep(0.25)
            wait_count += 1
        
        img=self.ros_image
        self.last_ros_image=img
                
        if self.ros_image is None:
            raise Exception("Camera image data not received")
               
        r_array=RecognizedObjectArray()
        r_array.header.stamp=now
        r_array.header.frame_id=self.frame_id
        
        if goal.use_roi:
            raise Warning("use_roi in ObjectRecognitionRequest ignored")
        
        
        
        (c_trans,c_rot) = self.listener.lookupTransform("/world", self.ros_cam_info.header.frame_id, rospy.Time(0))
        
        c_pose=rox.Pose(rox.q2R([c_rot[3], c_rot[0], c_rot[1], c_rot[2]]), c_trans)
        
        parameters =  cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementWinSize=32
        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_CONTOUR        
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(self.ros_image, self.aruco_dict, parameters=parameters)
        for board, object_id, tag_id in zip(self.boards, self.object_ids, self.tag_ids):            
        
            retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, self.camMatrix, self.distCoeffs)
            if (retval > 0):
                print "Found tag: " + tag_id
        
                try:
                    (a_trans,a_rot) = self.listener.lookupTransform(tag_id, object_id, rospy.Time(0))
                    print object_id
                    print a_trans
                    print a_rot
                    print ""
                    
                    o_pose=rox.Pose(rox.q2R([a_rot[3], a_rot[0], a_rot[1], a_rot[2]]), a_trans)
                    Ra, b = cv2.Rodrigues(rvec)
                    a_pose=rox.Pose(Ra,tvec)
                    
                    object_pose=c_pose*(a_pose*o_pose)
                                    
                    p=PoseWithCovarianceStamped()                
                    p.pose.pose=rox_msg.pose2msg(object_pose)
                    p.header.stamp=now
                    p.header.frame_id=self.frame_id
                    
                    r=RecognizedObject()
                    r.header.stamp=now
                    r.header.frame_id=self.frame_id
                    r.type.key=object_id
                    r.confidence=1
                    r.pose=p
                    
                    r_array.objects.append(r)
                    
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
        
        """for o in self.object_names:
            try:
                (trans,rot) = self.listener.lookupTransform("/world", o, rospy.Time(0))
                print o
                print trans
                print rot
                print ""
                                
                p=PoseWithCovarianceStamped()                
                p.pose.pose=Pose(Point(trans[0], trans[1], trans[2]), Quaternion(rot[0], rot[1], rot[2], rot[3]))
                p.header.stamp=now
                p.header.frame_id=self.frame_id
                
                r=RecognizedObject()
                r.header.stamp=now
                r.header.frame_id=self.frame_id
                r.type.key=o
                r.confidence=1
                r.pose=p
                
                r_array.objects.append(r)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue"""
                        
        result = ObjectRecognitionResult()
        result.recognized_objects=r_array
        
        self.server.set_succeeded(result=result)
    
def main():
    
    rospy.init_node('simulated_vision_server')
    
    s=SimulatedVisionServer(["vacuum_gripper_tool", "pickup_nest", "panel_nest", "leeward_mid_panel", "leeward_tip_panel"], "world")
    
    rospy.spin()
    


if __name__ == '__main__':
    main()
