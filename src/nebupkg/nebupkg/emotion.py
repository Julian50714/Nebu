import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array

# Cargar modelo entrenado
modelo = load_model("model.weights.h5")

# Emociones del RAF-DB (asegúrate que el orden sea el mismo que usaste en entrenamiento)
emociones = ['Sorpresa', 'Disgusto', 'Miedo', 'Felicidad', 'Tristeza', 'Enojo', 'Neutral']

# Inicializar la cámara
cap = cv2.VideoCapture(0)  # Usa 0 para la webcam predeterminada

# Clasificador Haar para detección facial
detector_cara = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

print("Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convertir a escala de grises para detección
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    caras = detector_cara.detectMultiScale(gris, scaleFactor=1.3, minNeighbors=5)

    for (x, y, w, h) in caras:
        rostro = frame[y:y+h, x:x+w]
        rostro_rgb = cv2.resize(rostro, (100, 100))
        rostro_array = img_to_array(rostro_rgb) / 255.0
        rostro_array = np.expand_dims(rostro_array, axis=0)

        pred = modelo.predict(rostro_array)[0]
        emocion_idx = np.argmax(pred)
        emocion = emociones[emocion_idx]
        confianza = np.max(pred)

        # Dibujar rectángulo y etiqueta
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        texto = f"{emocion} ({confianza*100:.1f}%)"
        cv2.putText(frame, texto, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    cv2.imshow("Reconocimiento de emociones - RAF-DB", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar cámara y cerrar ventanas
cap.release()
cv2.destroyAllWindows()

