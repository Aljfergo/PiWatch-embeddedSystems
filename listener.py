#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoListener:
    def __init__(self):
        rospy.init_node('video_listener', anonymous=True)
        self.bridge = CvBridge()
        self.video_subscriber = rospy.Subscriber('/camera/motion_video', Image, self.video_callback)

    def video_callback(self, msg):
        try:
            # Convertir el mensaje de imagen a un frame de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Mostrar el video
            cv2.imshow("Motion Video", cv_image)
            cv2.waitKey(1)  # Esperar un milisegundo para permitir que la interfaz gráfica se actualice

        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        video_listener = VideoListener()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
