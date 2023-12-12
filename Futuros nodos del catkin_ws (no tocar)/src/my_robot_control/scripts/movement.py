import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import RPi.GPIO as GPIO
from picamera import PiCamera
import cv2
from test_model import TestModel 

class ObstacleAvoidanceNode:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_msg = Twist()
        self.obstacle_detected = False
        self.start_time = None
        self.pir_pin = 17  
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pir_pin, GPIO.IN)
        self.camera = PiCamera()    
        self.tf_model = TestModel()  


    def laser_callback(self, scan_msg):
        
        if min(scan_msg.ranges) < 0.6:  # Considera que hay un obstáculo si la distancia es menor a 0.6 metro
            self.obstacle_detected = True
            self.cmd_vel_msg.linear.x = 0.0  # Detén el avance
            self.cmd_vel_msg.angular.z = 0.1  # Gira para evitar el obstáculo
        else:
            self.obstacle_detected = False
            self.cmd_vel_msg.linear.x = 0.2  # Avanza si no hay obstáculos
            self.cmd_vel_msg.angular.z = 0.0

    def execute_behavior(self):
        if self.obstacle_detected:
            # Paso 2: Parar y girar 135 grados
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 2.35619 
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            rospy.sleep(3.0)  

           # Paso 3: Esperar 5 minutos
            rospy.loginfo("Esquina alcanzada esperando 5 minutos...")

            self.start_time = rospy.Time.now()
            while rospy.Time.now() - self.start_time < rospy.Duration(300.0):  #Tiempo en secs 300s/5 min
                
                if GPIO.input(self.pir_pin) == GPIO.HIGH:
                    rospy.loginfo("¡Movimiento detectado! Tomando una foto.")
                    file_name = self.take_photo()

                    # Analizar imagen
                    self.process_image(file_name)

                rospy.sleep(1.0)

            # Paso 4: Girar -45 grados (se queda en 90)
            self.cmd_vel_msg.angular.z = -0.7854  
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            rospy.sleep(1.5)  # Ajusta el tiempo de giro según tu necesidad

            # Reiniciar el proceso (Volver al paso 1)
            self.obstacle_detected = False

    def take_photo(self):
        # Tomar una foto con la cámara y guardarla
        file_name = f"motion_detected_{rospy.Time.now().to_sec()}.jpg"
        self.camera.capture(file_name)
        rospy.loginfo(f"Foto tomada: {file_name}")
        return file_name
    
    def process_image(self, file_name):
        image = cv2.imread(file_name)
        result = self.tf_model.predict(image)
        rospy.loginfo(f"Resultado del modelo: {result}")
    

    
    def run(self):
        rate = rospy.Rate(10)  # Cada cuanto publica (Diego revisa esto)
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            self.execute_behavior()
            rate.sleep()

if __name__ == '__main__':
    try:
        avoidance_node = ObstacleAvoidanceNode()
        avoidance_node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        
        GPIO.cleanup()

