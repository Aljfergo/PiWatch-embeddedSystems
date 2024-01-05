#!/usr/bin/env python
import rospy
import time
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from datetime import datetime


def move_robot(pub, rotation, distance):
    twist = Twist()

    # Giro
    twist.angular.z = rotation  # Velocidad angular; positiva para giro a la derecha
    pub.publish(twist)
    time.sleep(abs(rotation / 0.5))  # Tiempo para completar el giro, ajustar según sea necesario

    # Avance
    twist.angular.z = 0
    twist.linear.x = distance  # Velocidad lineal; hacia adelante
    pub.publish(twist)
    time.sleep(abs(distance / 0.5))  # Tiempo para completar el avance, ajustar según sea necesario

    # Detener el robot
    twist.linear.x = 0
    pub.publish(twist)

def detect_and_record(pub):
    motion_detected = False 

    if motion_detected:
        # Iniciar grabación de video
        for _ in range(6):  # Grabar durante 6 segundos
            frame = None  # Capturar frame de la cámara
            pub.publish(frame)  # Publicar frame en el topic
            time.sleep(1)  # Intervalo entre frames, ajustar según la tasa de frames deseada
            video_talker()
        return True

    return False

def motion_detected():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame1 = cap.read()
        time.sleep(0.5)
        ret, frame2 = cap.read()

        diff = cv2.absdiff(frame1, frame2)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            (x, y, w, h) = cv2.boundingRect(contour)
            if cv2.contourArea(contour) < 700:
                detect_and_record(True)
            
        
#def navigation_goal_callback():
    #nav = rospy.Publisher('/move_base_simple/goal',)


def video_talker():
    pub = rospy.Publisher('/camera/motion_video', CompressedImage, queue_size=10)
    rospy.init_node('video_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Here, you would add your code to capture and publish video frames.
    cap = cv2.VideoCapture('/path/to/your/video_file.avi')
    # For example, using OpenCV to capture video from a camera.
    if not cap.isOpened():
        rospy.logerr("Error opening video file")
        return

    #bridge = CvBridge()

    while not rospy.is_shutdown()and cap.isOpened():
        # frame = capture_frame_from_camera() # pseudcocodigo
        frame = None  # Replace with actual frame capture code
        rospy.loginfo("Publishing video frame")
        pub.publish(frame)
        rate.sleep()

def main():
    start_time = rospy.get_time()
    now = rospy.Time.now()
    #one_hour = rospy.Duration(60*60)
    start = rospy.Time.from_sec(32400)#9 de la mañana
    end = rospy.Time.from_sec(46800)#13 de la tarde
    if now > start and now < end :
        rospy.init_node('robotnik_controller', anonymous=True)

        # Publicador para enviar comandos de movimiento.
        pub_move = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Publicador para el video.
        pub_video = rospy.Publisher('/camera/motion_video', Image, queue_size=10)
        # Suscriptor para la posición actual.
        rospy.Subscriber('navigationGoal', String, navigation_goal_callback)

        rate = rospy.Rate(10)  # 10 Hz

        for i in range(4):
            move_robot(pub_move, 90, 1)   # Gira 90 grados y avanza 1 metro.
            move_robot(pub_move, 135, 0)  # Gira 135 grados.
            while rospy.get_time() - start_time < 300:  # Espera 5 minutos.
                if detect_and_record(pub_video):        # Si detecta movimiento, graba video.
                    break
            rate.sleep()
    
if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass