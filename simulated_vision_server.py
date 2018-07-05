import rospy
from actionlib import SimpleActionServer
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionResult, \
     RecognizedObject, RecognizedObjectArray
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion

class SimulatedVisionServer(object):
    def __init__(self, object_names, frame_id="world", action_ns="recognize_objects"):        
                
        self.server=SimpleActionServer(action_ns, ObjectRecognitionAction, execute_cb=self.execute_callback)
        self.recognized_objects=dict()
        self.object_names=object_names
        self.listener=tf.TransformListener()
        self.frame_id="world"
        
    def execute_callback(self, goal):
        
        now=rospy.Time.now()
        
        r_array=RecognizedObjectArray()
        r_array.header.stamp=now
        r_array.header.frame_id=self.frame_id
        
        if goal.use_roi:
            raise Warning("use_roi in ObjectRecognitionRequest ignored")
        
        for o in self.object_names:
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
                continue
                        
        result = ObjectRecognitionResult()
        result.recognized_objects=r_array
        
        self.server.set_succeeded(result=result)
    
def main():
    
    rospy.init_node('simulated_vision_server')
    
    s=SimulatedVisionServer(["vacuum_gripper_tool", "pickup_nest", "panel_nest", "leeward_mid_panel", "leeward_tip_panel"], "world")
    
    rospy.spin()
    


if __name__ == '__main__':
    main()
