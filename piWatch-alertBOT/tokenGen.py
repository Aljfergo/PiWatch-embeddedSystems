import telepot
from telepot.loop import MessageLoop
from token import token


TOKEN = '6785587695:AAH0tjLxIIlGxuaUaJIXewNlkQO2HI9MeT4'

def handle_start(msg):
    chat_id = msg['chat']['id']
    bot.sendMessage(chat_id, f'Bienvenido a PiWatch! Por favor, inserte el siguiente token en el registro para poder vincular sus cuentas.')
    bot.sendMessage(chat_id, f'Tu token único es: {chat_id}')

bot = telepot.Bot(TOKEN)
MessageLoop(bot, {'chat': handle_start}).run_as_thread()
print('Escuchando...')


while True:
    pass