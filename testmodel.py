import numpy as np
import cv2
import tensorflow as tf

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

imagen_path = 'test_dataset/deteccion.jpg'
frame = cv2.imread(imagen_path)

def preprocess_image(image):
    image = cv2.resize(image, (300, 300))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.astype(np.uint8) 
    image = np.expand_dims(image, axis=0)  
    return image

input_image = preprocess_image(frame)

detections = model(input_image)

for i in range(len(detections['detection_boxes'])):
    
    (h, w) = frame.shape[:2]
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
            cv2.rectangle(frame, (int(startX), int(startY)), (int(endX), int(endY)), (0, 0, 255), 2)
            y = int(startY) - 15 if int(startY) - 15 > 15 else int(startY) + 15
            cv2.putText(frame, label, (int(startX), y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        if (class_id==17 or class_id ==18) and confidence>0.60:
            label = "{}: {:.2f}%".format(CLASSES[class_id], confidence * 100)
            cv2.rectangle(frame, (int(startX), int(startY)), (int(endX), int(endY)), (0, 255, 0), 2)
            y = int(startY) - 15 if int(startY) - 15 > 15 else int(startY) + 15
            cv2.putText(frame, label, (int(startX), y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)








# Mostrar el resultado
cv2.imshow("Resultado", frame)
cv2.waitKey(0)
cv2.destroyAllWindows()