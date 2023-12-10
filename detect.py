from gpiozero import MotionSensor
from config import botToken
import time
import telepot
import cv2
import numpy as np  # Importa numpy con el alias correcto

motion_sensor = MotionSensor(4)
file_video = "incident.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
salida = cv2.VideoWriter(file_video, fourcc, 20.0, (640, 480))

bot = telepot.Bot(botToken)
chat_id = 2
enviar = False

CLASSES = ["fondo","aeroplano", "bicicleta", "pajaro", "barco", "botella", "bus", "coche", "gato"
           "silla", "vaca","comedor", "perro", "caballo", "moto", "persona", "planta_maceta", "oveja", "sofa",
           "tren", "tv"]
# Utiliza la ruta correcta a tu modelo SavedModel
saved_model_path = "ruta/a/tu/saved_model"

print("Cargando modelo...")
# Cargar el modelo SavedModel de TensorFlow
net = cv2.dnn_DetectionModel(saved_model_path)
print("Modelo cargado")

while True:
    if motion_sensor.motion_detected():
        captura = cv2.VideoCapture(0)
        start_time = time.time()

        while captura.isOpened() and (time.time() - start_time) < 5:
            ret, imagen = captura.read()

            if ret:
                frame = cv2.rotate(imagen, cv2.ROTATE_180)
                (h, w) = frame.shape[:2]

                # Preprocesamiento de la imagen
                blob = cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True)

                # Hacer inferencia
                net.setInput(blob)
                classes, scores, boxes = net.forward(['num_detections', 'detection_scores', 'detection_boxes'])

                for i in range(len(classes[0])):
                    confidence = scores[0, i]
                    if confidence > 0.7:
                        class_id = int(classes[0, i])
                        if class_id == 15:
                            enviar = True
                            box = boxes[0, i]
                            (startX, startY, endX, endY) = (box * np.array([w, h, w, h])).astype("int")
                            label = "{}: {:.2f}%".format(CLASSES[class_id], confidence * 100)
                            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
                            y = startY - 15 if startY - 15 > 15 else startY + 15
                            cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                salida.write(frame)

            captura.release()
            salida.release()

            if enviar:
                print("Intruso detectado")
                bot.sendVideo(chat_id, open(file_video, 'rb'))
            else:
                print("No hay intruso")