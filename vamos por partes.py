from dis import Positions
from operator import truediv
from signal import siginterrupt
import numpy as np
import cv2
import tensorflow as tf
from gpiozero import MotionSensor
import time
import requests
import picamera
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import telepot
from geometry_msgs.msg import Twist

CLASSES = [
    "nada","persona", "bicicleta", "coche", "motocicleta", "avión", "autobús", "tren", "camión", "barco", "semáforo",
    "boca de incendios", "señal de calle", "señal de stop", "parquímetro", "banco", "pájaro", "gato", "perro", "caballo",
    "oveja", "vaca", "elefante", "oso", "cebra", "jirafa", "sombrero", "mochila", "paraguas", "zapato", "gafas",
    "bolso", "corbata", "maleta", "frisbee", "esquís", "snowboard", "pelota deportiva", "cometa", "bate de béisbol",
    "guante de béisbol", "patineta", "tabla de surf", "raqueta de tenis", "botella", "plato", "copa de vino", "taza",
    "tenedor", "cuchillo", "cuchara", "tazón", "plátano", "manzana", "sándwich", "naranja", "brócoli", "zanahoria",
    "perro caliente", "pizza", "donut", "pastel", "silla", "sofá", "planta en maceta", "cama", "espejo", "mesa de comedor",
    "ventana", "escritorio", "inodoro", "puerta", "televisor", "computadora portátil", "ratón", "control remoto",
    "teclado", "teléfono celular", "microondas", "horno", "tostadora", "fregadero", "refrigerador", "licuadora", "libro",
    "reloj", "jarrón", "tijeras", "oso de peluche", "secador de pelo", "cepillo de dientes", "cepillo para el cabello"
]

with open('./token.txt', 'r') as file:
    TOKEN = file.read().strip()
motion_sensor = MotionSensor(4)
bot = telepot.Bot(TOKEN)

print("Cargando modelo...")
saved_model_path = './saved_model'
model = tf.saved_model.load(saved_model_path)
print("Modelo listo!!")
file_video = "incident.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
salida = cv2.VideoWriter(file_video, fourcc, 20.0, (640, 480))
incident_severity=-1

def preprocess_image(image):
    image = cv2.resize(image, (300, 300))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.astype(np.uint8) 
    image = np.expand_dims(image, axis=0)  
    return image

def capture_photo(camera, filename='incident.jpg'):
    camera.capture(filename)

def inActiveSchedule():
    url = "http://localhost:8000/activeSchedule"
    response = requests.get(url)

    if response.status_code == 200:
        data = response.json()
        schedule_start = data.get("scheduleStart")
        schedule_end = data.get("scheduleEnd")
        current_time = int(time.time())

        if schedule_start <= current_time <= schedule_end:
            return True
        else:
            return False
    else:
        print(f"Error en la solicitud. Código de estado: {response.status_code}")
        return False
        """ def in_modo_vigilancia():
    tiempo_actual = time.localtime()
    return hora_inicio_vigilancia <= tiempo_actual.tm_hour < hora_fin_vigilancia """

def rotate(angle):
    position = [(0.0, 0.0, 0.0)]
    start_time = rospy.Time.now()
    rotate_angle = 45  # Grados de rotación
    while (rospy.Time.now() - start_time).to_sec() < end_time:
        for position in position:
            twist_cmd = Twist()
            twist_cmd.angular.z = rotate_angle * 3.14159 / 180.0  # Convertir a radianes
            twist_publisher.publish(twist_cmd)
            rospy.sleep(120)

def move(total_time):
    positions = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)]  # Posiciones a, b, c
    start_time = rospy.Time.now()

    while (rospy.Time.now() - start_time).to_sec() < total_time:
        for position in positions:
            twist_cmd = Twist()
            twist_cmd.linear.x, twist_cmd.linear.y, twist_cmd.linear.z = position
            twist_publisher.publish(twist_cmd)
            rospy.sleep(300)  # Esperar 5 minutos antes de pasar a la siguiente posición
            if motion_detected():
                notify_movement()

def notify_movement():
    if inActiveSchedule() == True:
        rospy.loginfo("¡Se ha detectado movimiento! ¡El robot está alerta!")
        guardar_video()


def guardar_video():
        pub = rospy.Publisher('/camera/motion_video', Image, queue_size=10)
        rospy.init_node('pi_watch', anonymous=True)
        rate = rospy.Rate(10) # 10hz

while True:
    if inActiveSchedule():
        if motion_sensor.motion_detected():
            chat_id = requests.get("http://localhost:8000/activeScheduleUserToken")
            moment=time.time()
            image_path="Incident"+moment+"_"+chat_id+".jpg"
            with picamera.PiCamera() as camera:
                capture_photo(camera, image_path)
            captura = cv2.VideoCapture(0)
            start_time = time.time()

            while captura.isOpened() and (time.time() - start_time) < 5:
                ret, imagen = captura.read()

                #imagen_path = 'test_dataset/deteccion.jpg'
            
                #frame = cv2.imread(image_path)
                input_image = preprocess_image(imagen)


                detections = model(input_image)

                for i in range(len(detections['detection_boxes'])):
                    
                    (h, w) = imagen.shape[:2]
                    detection_boxes = detections['detection_boxes'].numpy()
                    num_detections = detection_boxes.shape[1]
                    detection_classes = detections['detection_classes'][0].numpy()
                    detection_scores = detections['detection_scores'][0].numpy()

                    for i in range(num_detections):
                        box = detection_boxes[0, i, :]
                        (startY, startX, endY, endX) = (box[0] * h, box[1] * w, box[2] * h, box[3] * w)
                        class_id = int(detection_classes[i])
                        confidence = detection_scores[i]
                        print(f"Clase: {CLASSES[class_id]}, Confianza: {confidence}")

                            # Filtrar por clase 1
                        if class_id == 1 and confidence >0.60:
                            label = "{}: {:.2f}%".format(CLASSES[class_id], confidence * 100)
                            cv2.rectangle(imagen, (int(startX), int(startY)), (int(endX), int(endY)), (0, 0, 255), 2)
                            y = int(startY) - 15 if int(startY) - 15 > 15 else int(startY) + 15
                            cv2.putText(imagen, label, (int(startX), y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                            incident_severity=2


                        if (class_id==17 or class_id ==18) and confidence>0.60:
                            label = "{}: {:.2f}%".format(CLASSES[class_id], confidence * 100)
                            cv2.rectangle(imagen, (int(startX), int(startY)), (int(endX), int(endY)), (0, 255, 0), 2)
                            y = int(startY) - 15 if int(startY) - 15 > 15 else int(startY) + 15
                            cv2.putText(imagen, label, (int(startX), y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            incident_severity=1

                    salida.write(imagen)

            captura.release()
            salida.release()

            if incident_severity==2:
                print("Intruso detectado")
                bot.sendMessage(chat_id, "¡ALERTA!")
                bot.sendMessage(chat_id, "¡SE HA DETECTADO UN INTRUSO!")
                bot.sendVideo(chat_id, open(file_video, 'rb'))
            elif incident_severity==1:
                print("Movimiento no humano detectado")
                bot.sendMessage(chat_id, "Existe movimiento no humano en la zona de patrullaje")
                bot.sendMessage(chat_id, "¿Se trata de tu mascota?")
            else:
                print("Ningun intruso detectado")
            
            # Mostrar el resultado
            #cv2.imshow("Resultado", imagen)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()

if __name__ == 'main':
        try:
            rospy.init_node('robot_control_py')

            # Programar las tareas para rotar y detectar movimiento en bucle
            rate = rospy.Rate(1)  # Frecuencia de 1 Hz (ajusta según tus necesidades)
            while not rospy.is_shutdown():
                mi_robot.rotate_robot()
                mi_robot.notify_movement()
                rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("")  