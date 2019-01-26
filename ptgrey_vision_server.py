import rospy
from actionlib import SimpleActionServer
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionResult, \
     RecognizedObject, RecognizedObjectArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
import numpy as np
import cv2
from std_srvs.srv import Trigger
import time
import general_robotics_toolbox as rox
from general_robotics_toolbox import ros_msg as rox_msg
from general_robotics_toolbox import ros_tf as tf
from industrial_payload_manager.payload_transform_listener import PayloadTransformListener
from industrial_payload_manager.msg import PayloadArray
from cv_bridge import CvBridge, CvBridgeError
import sys
import threading

class CameraInfoStorage(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.ros_data=None
        self.camMatrix=None
        self.distCoeffs=None
        self.ros_image=None
    def fill_data(self,ros_data):
        self.ros_data=ros_data
        self.camMatrix=np.reshape(ros_data.K,(3,3))
        self.distCoeffs=np.array(ros_data.D)
        if len(self.distCoeffs) < 4:
            self.distCoeffs=np.array([[0,0,0,0,0]])
    def take_image(self,ros_image_data):
        ros_image1 = self.bridge.imgmsg_to_cv2(ros_image_data, desired_encoding="passthrough")
        #print type(ros_image1)
        #if len(ros_image1) > 2 and ros_image1.shape[2] == 4:
            #    self.ros_image=cv2.cvtColor(ros_image1, cv2.COLOR_BGRA2BGR)
        #else:
        self.ros_image=ros_image1

class SimulatedVisionServer(object):
    
    def ros_image_cb(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        ros_image1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if len(ros_image1) > 2 and ros_image1.shape[2] == 4:
            self.ros_image=cv2.cvtColor(ros_image1, cv2.COLOR_BGRA2BGR)
        else:
            self.ros_image=ros_image1
            self.ros_image_stamp = ros_data.header.stamp

    def ros_raw_overhead_image_cb(self, ros_data):
        self.ros_overhead_cam_info.take_image(ros_data)
        self.ros_image_stamp = ros_data.header.stamp

    def ros_raw_gripper_1_image_cb(self, ros_data):
        self.ros_gripper_1_cam_info.take_image(ros_data)
        self.ros_image_stamp = ros_data.header.stamp

    def ros_raw_gripper_2_image_cb(self, ros_data):
        self.ros_gripper_2_cam_info.take_image(ros_data)
        self.ros_image_stamp = ros_data.header.stamp

    def ros_cam_info_cb(self, ros_data):
        if(ros_data.header.frame_id=="overhead_camera"):
            self.ros_overhead_cam_info.fill_data(ros_data)
        if(ros_data.header.frame_id=="gripper_camera_1"):
            self.ros_gripper_1_cam_info.fill_data(ros_data)
        if(ros_data.header.frame_id=="gripper_camera_2"):
            self.ros_gripper_2_cam_info.fill_data(ros_data)

    
    # TODO: handle different action namespaces, camera namespaces, etc
    def __init__(self, frame_id="world", action_ns_1="recognize_objects", action_ns_2="recognize_objects_gripper"):
        
        self.bridge = CvBridge()
        self.overhead_server=SimpleActionServer(action_ns_1, ObjectRecognitionAction, execute_cb=self.execute_overhead_callback)
        self.gripper_server=SimpleActionServer(action_ns_2, ObjectRecognitionAction, execute_cb=self.execute_gripper_callback)
        self.recognized_objects=dict()

        self.listener=PayloadTransformListener()
        self.frame_id="world"
        self.ros_image=None
        self.ros_image_stamp=rospy.Time(0)
        self.last_ros_image_stamp=rospy.Time(0)
        self.ros_overhead_cam_info=CameraInfoStorage()
        self.ros_gripper_1_cam_info=CameraInfoStorage()
        self.ros_gripper_2_cam_info=CameraInfoStorage()

        #TODO: Handle compressed images vs uncompressed images better. Note that
        #the uncompressed 20 MP color images don't seem to be received properly.
        if not "compressed-image" in sys.argv:
            self.ros_overhead_img_sub=rospy.Subscriber('/overhead_camera/image', Image, self.ros_raw_overhead_image_cb)
            self.ros_gripper_1_img_sub=rospy.Subscriber('/gripper_camera_1/image', Image, self.ros_raw_gripper_1_image_cb)
            self.ros_gripper_2_img_sub=rospy.Subscriber('/gripper_camera_2/image', Image, self.ros_raw_gripper_2_image_cb)
        else:
            self.ros_overhead_img_sub=rospy.Subscriber('/overhead_camera/image/compressed', CompressedImage, self.ros_image_cb)
            self.ros_gripper_1_img_sub=rospy.Subscriber('/gripper_camera_1/image/compressed', CompressedImage, self.ros_image_cb)
            self.ros_gripper_2_img_sub=rospy.Subscriber('/gripper_camera_2/image/compressed', CompressedImage, self.ros_image_cb)
        self.ros_overhead_trigger=rospy.ServiceProxy('/overhead_camera/trigger', Trigger)

        self.ros_overhead_cam_info_sub=rospy.Subscriber('/overhead_camera/camera_info', CameraInfo, self.ros_cam_info_cb)

        self.ros_gripper_1_trigger=rospy.ServiceProxy('/gripper_camera_1/trigger', Trigger)
        self.ros_gripper_2_trigger=rospy.ServiceProxy('/gripper_camera_2/trigger', Trigger)
        self.ros_gripper_1_cam_info_sub=rospy.Subscriber('/gripper_camera_1/camera_info', CameraInfo, self.ros_cam_info_cb)
        self.ros_gripper_2_cam_info_sub=rospy.Subscriber('/gripper_camera_2/camera_info', CameraInfo, self.ros_cam_info_cb)

        #self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

        self.payloads=dict()
        self.link_markers=dict()
        self.payloads_lock=threading.Lock()
        self.payloads_sub=rospy.Subscriber("payload", PayloadArray, self._payload_msg_cb)
        
    def _payload_msg_cb(self, msg):
        with self.payloads_lock:
            for p in msg.payloads:
                if p.name not in self.payloads:                
                    self.payloads[p.name]=p                    
                else:
                    payload = self.payloads[p.name]
                    #ignore stale data
                    if payload.header.stamp > p.header.stamp:
                        continue                    
                    self.payloads[p.name]=p
                
            for l in msg.link_markers:
                if l.header.frame_id not in self.link_markers:                
                    self.link_markers[l.header.frame_id]=l                    
                else:
                    ll = self.link_markers[l.header.frame_id]
                    #ignore stale data
                    if ll.header.stamp > l.header.stamp:
                        continue                    
                    self.link_markers[l.header.frame_id]=l
            
            for d in msg.delete_payloads:
                if d in self.payloads:                    

                    del self.payloads[d]
                if d in self.link_markers:
                    del self.link_markers[d]
        
        
    def execute_overhead_callback(self, goal):
        
        now=rospy.Time.now()

        if self.ros_overhead_cam_info is None:
            raise Exception("Camera Info not received")

        self.last_ros_image_stamp=self.ros_image_stamp

        try:
            self.ros_overhead_trigger.wait_for_service(timeout=0.1)
            self.ros_overhead_trigger()
        except:
            pass

        wait_count=0
        while self.ros_overhead_cam_info.ros_image is None or self.ros_image_stamp == self.last_ros_image_stamp:
            if wait_count > 250:
                raise Exception("Image receive timeout")
            time.sleep(0.25)
            wait_count += 1

        print "Past timeout"
        #stored data in local function variable, so changed all references off of self.ros_image
        img=self.ros_overhead_cam_info.ros_image


        if img is None:
            raise Exception("Camera image data not received")
        print "image received"
        r_array=RecognizedObjectArray()
        r_array.header.stamp=now
        r_array.header.frame_id=self.frame_id

        if goal.use_roi:
            raise Warning("use_roi in ObjectRecognitionRequest ignored")
        
        
        search_objects=dict()
        with self.payloads_lock:
            for _, p in self.payloads.items():
                search_objects[p.name] = p.markers
            for _, l in self.link_markers.items():
                search_objects[l.header.frame_id] = l.markers
        
                #TODO: handle multiple aruco dictionaries, currently only use one
        print "past payload lock"
        aruco_dicts=set()
        for m in search_objects.itervalues():
            for m2 in m:
                aruco_dicts.add(m2.marker.dictionary)
        print len(aruco_dicts)
        aruco_dicts.add("DICT_ARUCO_ORIGINAL")
        assert len(aruco_dicts) == 1, "Currently all tags must be from the same dictionary"

        if not hasattr(cv2.aruco, next(iter(aruco_dicts))):
            raise ValueError("Invalid aruco-dict value")
        aruco_dict_id=getattr(cv2.aruco, next(iter(aruco_dicts)))
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)

        c_pose = self.listener.lookupTransform("/world", self.ros_overhead_cam_info.ros_data.header.frame_id, rospy.Time(0))
        print "c_pose"
        print c_pose.p
        print c_pose.R
        parameters =  cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementWinSize=32

        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(img, aruco_dict, parameters=parameters)
        print ids
        for object_name, payload_markers in search_objects.items():

            tag_object_poses=dict()

            for m in payload_markers:
                board = get_aruco_gridboard(m.marker, aruco_dict)
                #board = cv2.aruco.GridBoard_create(4, 4, .04, .0075, aruco_dict, 16)


                retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, self.ros_overhead_cam_info.camMatrix, self.ros_overhead_cam_info.distCoeffs)
                print retval
                print rvec
                print tvec
                if (retval > 0):
                    if(m.name=="leeward_mid_panel_marker_1" or m.name=="leeward_tip_panel_marker_1"):
                        print "Found tag: " + m.name

                        try:
                            #o_pose = self.listener.lookupTransform(m.name, object_name, rospy.Time(0))
                            o_pose = rox_msg.msg2transform(m.pose).inv()
                            print object_name
                            print o_pose
                            print ""
                            print "o_pose"
                            print o_pose.p
                            print o_pose.R
                            Ra, b = cv2.Rodrigues(rvec)
                            a_pose=rox.Transform(Ra,tvec)
                            print "a_pose"
                            print a_pose.p
                            print a_pose.R
                            tag_object_poses[m.name]=c_pose*(a_pose*o_pose)

                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue
            
                    #TODO: merge tag data if multiple tags detected
            if len(tag_object_poses) > 0:
                p=PoseWithCovarianceStamped()
                p.pose.pose=rox_msg.transform2pose_msg(tag_object_poses.itervalues().next())
                p.header.stamp=now
                p.header.frame_id=self.frame_id

                r=RecognizedObject()
                r.header.stamp=now
                r.header.frame_id=self.frame_id
                r.type.key=object_name
                r.confidence=1
                r.pose=p

                r_array.objects.append(r)
        
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
        print "Succeeded in finding tags"
        result = ObjectRecognitionResult()
        result.recognized_objects=r_array

        self.overhead_server.set_succeeded(result=result)

    def execute_gripper_callback(self, goal):

        now=rospy.Time.now()

        if self.ros_gripper_1_cam_info is None or self.ros_gripper_2_cam_info is None:
            raise Exception("Camera Info not received")

        self.last_ros_image_stamp=self.ros_image_stamp

        try:
            self.ros_gripper_1_trigger.wait_for_service(timeout=0.1)
            self.ros_gripper_1_trigger()
            self.ros_gripper_2_trigger.wait_for_service(timeout=0.1)
            self.ros_gripper_2_trigger()
        except:
            pass

        wait_count=0
        while ((self.ros_gripper_1_cam_info.ros_image is None and self.ros_gripper_2_cam_info.ros_image is None) or self.ros_image_stamp == self.last_ros_image_stamp):
            if wait_count > 250:
                raise Exception("Image receive timeout")
            time.sleep(0.25)
            wait_count += 1

        print "Past timeout"
        gripper_1_img=ros_gripper_1_cam_info.ros_image
        gripper_2_img=ros_gripper_1_cam_info.ros_image


        if gripper_1_img is None or gripper_2_img is None:
            raise Exception("Camera image data not received")
        print "image received"
        #TODO replace code with gripper camera code for placement
        '''
        r_array=RecognizedObjectArray()
        r_array.header.stamp=now
        r_array.header.frame_id=self.frame_id

        if goal.use_roi:
            raise Warning("use_roi in ObjectRecognitionRequest ignored")


        search_objects=dict()
        with self.payloads_lock:
            for _, p in self.payloads.items():
                search_objects[p.name] = p.markers
            for _, l in self.link_markers.items():
                search_objects[l.header.frame_id] = l.markers

                #TODO: handle multiple aruco dictionaries, currently only use one
        print "past payload lock"
        aruco_dicts=set()
        for m in search_objects.itervalues():
            for m2 in m:
                aruco_dicts.add(m2.marker.dictionary)
        print len(aruco_dicts)

        assert len(aruco_dicts) == 1, "Currently all tags must be from the same dictionary"

        if not hasattr(cv2.aruco, next(iter(aruco_dicts))):
            raise ValueError("Invalid aruco-dict value")
        aruco_dict_id=getattr(cv2.aruco, next(iter(aruco_dicts)))
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_id)

        c_pose = self.listener.lookupTransform("/world", self.ros_overhead_cam_info.ros_data.header.frame_id, rospy.Time(0))
        print "c_pose"
        print c_pose.p
        print c_pose.R
        parameters =  cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementWinSize=32
        parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_CONTOUR
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(self.ros_image, aruco_dict, parameters=parameters)
        for object_name, payload_markers in search_objects.items():

            tag_object_poses=dict()

            for m in payload_markers:
                board = get_aruco_gridboard(m.marker, aruco_dict)
                retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, self.ros_overhead_cam_info.camMatrix, self.ros_overhead_cam_info.distCoeffs)
                if (retval > 0):
                    if(m.name=="leeward_mid_panel_marker_5"):
                        print "Found tag: " + m.name

                        try:
                            #o_pose = self.listener.lookupTransform(m.name, object_name, rospy.Time(0))
                            o_pose = rox_msg.msg2transform(m.pose).inv()
                            print object_name
                            print o_pose
                            print ""
                            print "o_pose"
                            print o_pose.p
                            print o_pose.R
                            Ra, b = cv2.Rodrigues(rvec)
                            a_pose=rox.Transform(Ra,tvec)
                            print "a_pose"
                            print a_pose.p
                            print a_pose.R
                            tag_object_poses[m.name]=c_pose*(a_pose*o_pose)

                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue

                    #TODO: merge tag data if multiple tags detected
            if len(tag_object_poses) > 0:
                p=PoseWithCovarianceStamped()
                p.pose.pose=rox_msg.transform2pose_msg(tag_object_poses.itervalues().next())
                p.header.stamp=now
                p.header.frame_id=self.frame_id

                r=RecognizedObject()
                r.header.stamp=now
                r.header.frame_id=self.frker, aruco_dict)
                board = cv2.aruco.GridBoard_createame_id
                r.type.key=object_name
                r.confidence=1
                r.pose=p

                r_array.objects.append(r)

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
        '''
        self.gripper_server.set_succeeded(result=result)

def get_aruco_gridboard(marker, aruco_dict):
    #Create grid board representing the calibration target
    if isinstance(marker.dictionary,basestring):
        if not marker.dictionary.startswith('DICT_'):
            raise ValueError("Invalid aruco-dict value")
    
        
    elif isinstance(marker.dictionary,int):
        aruco_dict = cv2.aruco.Dictionary_get(marker.dictionary)
    else:
        aruco_dict_id=marker.dictionary
    board=cv2.aruco.GridBoard_create(marker.markersX, marker.markersY, \
                                     marker.markerLength, marker.markerSpacing, aruco_dict,\
                                     marker.firstMarker)
    return board
    
def main():
    
    rospy.init_node('simulated_vision_server')
    
    s=SimulatedVisionServer( "world")
    
    rospy.spin()
    


if __name__ == '__main__':
    main()
