FROM python:3.9

# Directorio de trabajo
WORKDIR /app
COPY . /app

# Dependencias
RUN pip install -r requirements.txt

EXPOSE 8000

# Ejecuto
CMD ["python", "main.py"]
# CMD ["rosrun", "empotrados piwatch_listener.py"]
# CMD ["rosrun", "empotrados piwatch_talker.py"]
