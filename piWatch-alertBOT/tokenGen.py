import telepot
from telepot.loop import MessageLoop
import uuid


TOKEN = '6785587695:AAH0tjLxIIlGxuaUaJIXewNlkQO2HI9MeT4'

def handle_start(msg):
    chat_id = msg['chat']['id']
    user_token = str(uuid.uuid4())
    bot.sendMessage(chat_id, f'Tu token único es: {user_token}')

bot = telepot.Bot(TOKEN)
MessageLoop(bot, {'chat': handle_start}).run_as_thread()
print('Escuchando...')


while True:
    pass