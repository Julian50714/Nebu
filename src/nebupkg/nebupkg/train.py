import os
import numpy as np
from tensorflow.keras.utils import to_categorical
from PIL import Image

def cargar_rafdb(ruta_raiz, tipo="train"):
    """
    Carga imágenes y etiquetas del RAF-DB (Real-world Affective Faces Database).

    Parámetros:
        ruta_raiz (str): Ruta a la carpeta base del RAF-DB descomprimido.
        tipo (str): "train" o "test" para indicar qué conjunto cargar.

    Retorna:
        X (np.array): Array con imágenes cargadas (N, 100, 100, 3).
        y (np.array): Array one-hot con etiquetas (N, 7).
    """

    ruta_imagenes = os.path.join(ruta_raiz, "Image", "aligned")
    ruta_etiquetas = os.path.join(ruta_raiz, "EmoLabel", "list_patition_label.txt")

    if not os.path.exists(ruta_etiquetas):
        raise FileNotFoundError(f"❌ No se encontró el archivo de etiquetas: {ruta_etiquetas}")

    # Leer las etiquetas y separar según train/test
    imagenes_lista = []
    etiquetas_lista = []

    print(f"📄 Leyendo etiquetas para {tipo}...")

    with open(ruta_etiquetas, "r") as f:
        for linea in f:
            nombre_imagen, etiqueta_str = linea.strip().split()
            etiqueta = int(etiqueta_str)

            # El archivo tiene nombres con 'train_' o 'test_'
            if tipo == "train" and nombre_imagen.startswith("train_"):
                imagenes_lista.append(nombre_imagen)
                etiquetas_lista.append(etiqueta)
            elif tipo == "test" and nombre_imagen.startswith("test_"):
                imagenes_lista.append(nombre_imagen)
                etiquetas_lista.append(etiqueta)

    print(f"✅ Se encontraron {len(imagenes_lista)} imágenes para {tipo}.")

    X = []
    y = []

    for idx, nombre_imagen in enumerate(imagenes_lista):
        # Los archivos alineados tienen sufijo "_aligned.jpg"
        nombre_archivo = nombre_imagen.replace(".jpg", "_aligned.jpg")
        ruta_imagen = os.path.join(ruta_imagenes, nombre_archivo)

        if not os.path.exists(ruta_imagen):
            print(f"⚠️ Imagen no encontrada: {ruta_imagen}")
            continue

        # Abrir imagen y convertir a numpy array
        imagen = Image.open(ruta_imagen).convert("RGB")
        imagen = imagen.resize((100, 100))  # Asegura tamaño correcto
        imagen_np = np.array(imagen) / 255.0  # Normalizar a [0,1]

        X.append(imagen_np)
        y.append(etiquetas_lista[idx])

    X = np.array(X)
    y = np.array(y)

    # Restar 1 a etiquetas para que empiecen en 0
    y = y - 1

    # One-hot encoding para 7 clases
    y = to_categorical(y, num_classes=7)

    print(f"Shape X: {X.shape}")
    print(f"Shape y: {y.shape}")

    return X, y


if __name__ == "__main__":
    ruta_rafdb = "/home/julian/nebufull/src/nebupkg/nebupkg/archive/rafdb_basic"

    # Cargar datos de entrenamiento
    X_train, y_train = cargar_rafdb(ruta_rafdb, tipo="train")

    # Cargar datos de prueba
    X_test, y_test = cargar_rafdb(ruta_rafdb, tipo="test")

    # Aquí puedes seguir con el entrenamiento de tu modelo...

