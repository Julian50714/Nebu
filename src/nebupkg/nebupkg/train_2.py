import os
import numpy as np
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from tensorflow.keras.preprocessing.image import load_img, img_to_array
from sklearn.model_selection import train_test_split

def cargar_rafdb_completo(ruta_raiz):
    """
    Carga todas las imágenes disponibles del RAF-DB y devuelve X, y (sin separar).
    """
    ruta_etiquetas = os.path.join(ruta_raiz, "EmoLabel", "list_patition_label.txt")
    if not os.path.exists(ruta_etiquetas):
        raise FileNotFoundError(f"Archivo de etiquetas no encontrado: {ruta_etiquetas}")

    print("📄 Leyendo etiquetas...")

    with open(ruta_etiquetas, "r") as f:
        lineas = f.readlines()

    lista_imagenes = []
    lista_labels = []

    for linea in lineas:
        partes = linea.strip().split()
        if len(partes) != 2:
            continue
        nombre_img, etiqueta = partes
        lista_imagenes.append(nombre_img)
        lista_labels.append(int(etiqueta) - 1)  # Asegura que vaya de 0 a 6

    print(f"✅ Se encontraron {len(lista_imagenes)} imágenes.")

    X = []
    y = []

    for i, img_name in enumerate(lista_imagenes):
        nombre_alineado = img_name.replace(".jpg", "_aligned.jpg")
        ruta_imagen = os.path.join(ruta_raiz, "Image", "aligned", nombre_alineado)

        if not os.path.exists(ruta_imagen):
            print(f"⚠️ Imagen no encontrada: {ruta_imagen}, se omite")
            continue

        imagen = load_img(ruta_imagen, target_size=(100, 100))
        imagen = img_to_array(imagen) / 255.0
        X.append(imagen)
        y.append(lista_labels[i])

    X = np.array(X)
    y = to_categorical(np.array(y), num_classes=7)

    print(f"Shape X: {X.shape}")
    print(f"Shape y: {y.shape}")
    return X, y

def crear_modelo():
    model = Sequential([
        Conv2D(32, (3, 3), activation='relu', input_shape=(100, 100, 3)),
        MaxPooling2D((2, 2)),
        Conv2D(64, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)),
        Conv2D(128, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)),
        Flatten(),
        Dense(128, activation='relu'),
        Dropout(0.5),
        Dense(7, activation='softmax')
    ])
    model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    return model

if __name__ == "__main__":
    ruta_rafdb = "./archive/rafdb_basic"  # Cambia si tu ruta es diferente

    print("🔄 Cargando datos completos...")
    X, y = cargar_rafdb_completo(ruta_rafdb)

    print("🔀 Dividiendo en train/test...")
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    print("🛠️ Creando modelo...")
    model = crear_modelo()

    print("🚀 Entrenando modelo...")
    history = model.fit(
        X_train, y_train,
        epochs=100,
        batch_size=64,
        validation_data=(X_test, y_test)
    )

    print("🧪 Evaluando modelo en test...")
    loss, accuracy = model.evaluate(X_test, y_test)
    print(f"📊 Test loss: {loss:.4f}, Test accuracy: {accuracy:.4f}")

    print("💾 Guardando modelo...")
    model.save("rafdb_model.h5")

    print("✅ ¡Entrenamiento completado!")

