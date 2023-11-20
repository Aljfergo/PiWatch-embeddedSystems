import time
import telepot
import cv2

file_video= "output.mp4"
fourcc =cv2.VideoWriter_fourcc(*'mp4v')
salida=cv2.VideoWriter(file_video, fourcc, 20.0, (640,480))

bot= telepot.Bot (//getBotKeyFromData.ini)
chat_id = //getChatIdFromBBDD
print("iniciando grabación...")

captura = cv2.VideoCapture(0)
while captura.isOpened() and(time.time()-start_time)<5:
  ret, imagen=captura.read()
  if ret == True:
    frame = cv2.rotate(imagen, cv2.ROTATE_180)
    salida.write(frame)
captura.release()
salida.release()
bot.sendVideo(chat_id, open(file_video, 'rb'))
