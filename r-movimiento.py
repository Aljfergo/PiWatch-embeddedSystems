import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import telepot

class Robot:
    def init(self):
        self.name = "Robotnik"
        self.position = (0, 0, 0)
        self.bridge = CvBridge()
        self.video_publisher = rospy.Publisher('/camera/motion_video', Image, queue_size=10)
        self.hora_inicio_vigilancia = 8
        self.hora_fin_vigilancia = 20

    def set_position(self, x, y, z):
        self.position = (x, y, z)
        return f"{self.name} está ahora en la posición {self.position}"

    def in_modo_vigilancia(self):
        tiempo_actual = time.localtime()
        return self.hora_inicio_vigilancia <= tiempo_actual.tm_hour < self.hora_fin_vigilancia

    def aviso(self):
        if self.in_modo_vigilancia():
            print("¡Se ha detectado movimiento! ¡El robot está alerta!")
            self.guardar_video()


    def guardar_video(self):
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                break

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.video_publisher.publish(img_msg)

            # Publicar con una frecuencia de 10 Hz (ajusta según sea necesario)
            rospy.sleep(0.1)

        cap.release()
        
    #def enviar_alerta(self):
        # Envía una alerta a Telegram con el enlace al video
     #   mensaje = f"¡Se ha detectado movimiento! Video disponible en el topic: /camera/motion_video "
    #  self.telegram_bot.sendMessage(chat_id='TU_CHAT_ID', text=mensaje)

def detectar_movimiento(mi_robot):
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
                continue
            mi_robot.notify_movement()
            return True  # Retorna True al detectar movimiento
        
#def detectar_movimiento(frame_anterior, frame_actual):
    # Convertir los frames a escala de grises para la comparación
    #frame_anterior_gris = cv2.cvtColor(frame_anterior, cv2.COLOR_BGR2GRAY)
    #frame_actual_gris = cv2.cvtColor(frame_actual, cv2.COLOR_BGR2GRAY)

    # Calcular la diferencia absoluta entre los frames
    #diff = cv2.absdiff(frame_anterior_gris, frame_actual_gris)

    # Aplicar un umbral para identificar los píxeles cambiantes
    #_, umbral = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

    # Encuentra contornos en la imagen umbralizada
    #contours, _ = cv2.findContours(umbral, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Si hay contornos (cambios en la imagen), retorna True
    #return len(contours) > 0

def programar_tarea(robot):
    horas_y_posiciones = [
        (time.struct_time((2023, 12, 4, 10, 0, 0, 1, 338, 0)), (1, 0, 0)),
        (time.struct_time((2023, 12, 4, 15, 30, 0, 1, 338, 0)), (0, 1, 0)),
        (time.struct_time((2023, 12, 4, 19, 45, 0, 1, 338, 0)), (0, 0, 1)),
    ]

    while True:
        for hora, posicion in horas_y_posiciones:
            while True:
                tiempo_actual = time.localtime()

                if tiempo_actual.tm_hour == hora.tm_hour and tiempo_actual.tm_min == hora.tm_min:
                    robot.set_position(*posicion)
                    robot.notify_movement()
                    # Parar y esperar 120 segundos
                    time.sleep(120)
                    # Verificar si hay movimiento durante la espera
                    if detectar_movimiento():
                        robot.aviso()
                    break
                time.sleep(60)  # Esperar un minuto antes de verificar nuevamente

if __name__ == 'main':
    try:
        rospy.init_node('pi_watch')

        # Crear un robot
        mi_robot = Robot()
        # Programar las tareas para asignar posiciones y detectar movimiento en bucle
        programar_tarea(mi_robot)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
