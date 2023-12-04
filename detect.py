from gpiozero import MotionSensor
from config import botToken
import time
import telepot
import cv2
import numpy

motion_sensor=MotionSensor(4)
file_video="incident.mp4"
fourcc =cv2.VideoWriter_fourcc(*'mp4v')
salida=cv2.VideoWriter(file_video,fourcc,20.0,(640,480))

bot =telepot.Bot(botToken)
chat_id = 2
enviar=False

CLASSES = ["fondo","aeroplano", "bicicleta", "pajaro", "barco", "botella", "bus", "coche", "gato"
           "silla", "vaca","comedor", "perro", "caballo", "moto", "persona", "planta_maceta", "oveja", "sofa",
           "tren", "tv"]
print("Cargando modelo...")
net=cv2.dnn.readNetFromCaffe("modelo.prototxt.txt", "modelo.caffemodel")
print("Modelo cargado")

while True:
    if motion_sensor.motion_detected():
        captura=cv2.VideoCapture(0)
        start_time=time.time()
        while captura.isOpened() and (time.time()-start_time)<5:
            ret, imagen=captura.read()
            if ret == True:
                frame =cv2.rotate(imagen, cv2.ROTATE_180)
                (h,w) =frame.shape[:2]
                blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300,300)), 0.007843,(300,300), 127.5)
                detections =net.forward()
                net.setInput(blob)
                detections=net.forward()
                for i in range(5):
                    confidence =detections[0,0,i,2]
                    if confidence> 0.7 :
                        idx = int(detections[0,0,i,1])
                        if idx == 15:
                            enviar=True;
                            box =detections[0,0,i,3:7]*np.array([w,h,w,h])
                            (startX, startY, endX, endY) = box.astype("int")
                            label = "{}: {:.2f}%".format(CLASSES[idx],confidence *100)
                            cv2.rectangle(frame,(startX, startY), (endX, endY), (0,0,255), 2)
                            y=startY-15 if startY-15>15 else startY + 15
                            cv2.putText(frame,label, (startX, y), cv2.FONT_HERSEHEY_SIMPLEX, 2.5, (0,0,255),2)
                salida.write(frame)
            captura.release()
            salida.release()
            if enviar==True:
                print("Intruso detectado")
                bot.sendVideo(chat_id, open(file_video,'rb'))
            else:
                print("No hay intruso")
    
