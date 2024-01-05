#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def video_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Received video frame")

def video_listener():
    try:
        rospy.init_node('video_listener', anonymous=True)
        rospy.Subscriber('/camera/motion_video', CompressedImage, video_callback)
    except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))
            
    rospy.spin()

if _name_ == '_main_':
    video_listener()
