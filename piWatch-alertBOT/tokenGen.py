import telepot
from telepot.loop import MessageLoop

# Leer el token desde el archivo
with open('../token.txt', 'r') as file:
    TOKEN = file.read().strip()

def handle_start(msg):
    chat_id = msg['chat']['id']
    bot.sendMessage(chat_id, '¡Bienvenido a PiWatch!')
    bot.sendMessage(chat_id, 'Por favor, inserte el siguiente token en el registro para poder vincular sus cuentas.')
    bot.sendMessage(chat_id, f'Tu token único es: {chat_id}')

bot = telepot.Bot(TOKEN)
MessageLoop(bot, {'chat': handle_start}).run_as_thread()
print('Escuchando...')

while True:
    pass