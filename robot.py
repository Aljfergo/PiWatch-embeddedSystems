from tracemalloc import start
from asyncore import dispatcher
from cgitb import text
from logging import warn
import time 
import cv2
from telegram import Update 
from telegram.ext import Updater, CommandHandler, CallbackContext
import threading
import rospy

class Robot:
    
    def init (self,name, telegram_bot):
        self.name = name
        self.position = (0,0,0)
        self.telegram_bot = telegram_bot

    def set_position(self, x, y, z):
        self.position = (x, y, z)
        print("{self.name} esta ahora en la posicion {self.position}")

    def aviso(self):
        warn = "Se ha detectado un intruso"
        self.telegram_bot.send_warn(chat_id="", text=warn)



def start_bot(update: Update, contex = CallbackContext) -> None:
        update.warn.reply("Hey soy el Robotniik")

def mov(robot, hora, x, y, z):
    while True:
        tiempo_actual = time.localtime()
        if tiempo_actual.tm_hour == hora.tm_hour and tiempo_actual.tm_min == hora.tm_min:
            robot.set_position(x, y, z)
            break
        time.sleep(120)  # Esperar un minuto antes de verificar nuevamente
        if detectar():
            el_robot.aviso()

def detectar():
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
                return True
            else:
                return False

#Datos del bot de telegram
Token_bot = ".."
updater = Updater(token = Token_bot, use_context = True)
dispatcher = updater.dispatcher
dispatcher.add_handler(CommandHandler("start_bot", start))

#El robot
el_robot = Robot("robot", updater.bot)

# Definir las horas y posiciones
hora_1 = time.struct_time((2023, 12, 4, 10, 0, 0, 1, 338, 0))
hora_2 = time.struct_time((2023, 12, 4, 12, 30, 0, 1, 338, 0))
hora_3 = time.struct_time((2023, 12, 4, 15, 0, 0, 1, 338, 0))

posicion_1 = (0, 0, 0)
posicion_2 = (0, 0, 0)
posicion_3 = (0, 0, 0)

# Programar las tareas para asignar las posiciones a las horas especificadas
mov(el_robot, hora_1, *posicion_1)
mov(el_robot, hora_2, *posicion_2)
mov(el_robot, hora_3, *posicion_3)

# Iniciar la detección de movimiento en un hilo separado
motion_detection_thread = threading.Thread(target=detectar)
motion_detection_thread.start()

# Iniciar el bot de Telegram
updater.start_polling()
updater.idle()
