import numpy as np
import cv2
import tensorflow as tf
from gpiozero import MotionSensor
import time
import telepot
import requests
import picamera

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

with open('./token.txt', 'r') as file:
    TOKEN = file.read().strip()
motion_sensor = MotionSensor(4)
bot = telepot.Bot(TOKEN)


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