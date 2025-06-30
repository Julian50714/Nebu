import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import whisper
import time
import os
import threading
import sys

SAMPLE_RATE = 16000
THRESHOLD = 0.9
MAX_SEGMENT_DURATION = 1000

whisper_model = whisper.load_model("base")

def detectar_voz(audio, threshold=THRESHOLD):
    return np.any(np.abs(audio) > threshold)

class VoiceTranscriberNode(Node):
    def __init__(self):
        super().__init__('voice_transcriber_node')
        self.publisher_ = self.create_publisher(String, 'transcripcion_voz', 10)
        self.subscription = self.create_subscription(Bool, 'tts_complete', self.mic_control_callback, 10)
        self.get_logger().info('🎤 Nodo de transcripción de voz con Whisper iniciado')

        self.timer = self.create_timer(0.1, self.escuchar_continuamente)

        self.escuchando = False
        self.audio_buffer = []
        self.inicio_segmento = None
        self.tiempo_inicio_silencio = 0
        self.mic_enabled = True  # Cambiado a True por defecto

        # Variables para la visualización
        self.visualizando = False
        self.nivel_audio = 0

        # Iniciar thread de visualización
        self.visualization_thread = threading.Thread(target=self.mostrar_visualizacion)
        self.visualization_thread.daemon = True
        self.visualization_thread.start()

        self.get_logger().info("📺 Visualización en terminal activada")

    def mic_control_callback(self, msg):
        self.mic_enabled = msg.data
        estado = "🟢 activado" if self.mic_enabled else "🔴 desactivado"
        self.get_logger().info(f'🎙️ Micrófono {estado} por señal externa')

    def mostrar_visualizacion(self):
        """Función que corre en un hilo separado para mostrar la visualización"""
        while True:
            if self.visualizando:
                # Limpiar línea actual
                sys.stdout.write('\r')
                
                # Crear barra de nivel de audio
                nivel_normalizado = min(int(self.nivel_audio * 30), 30)
                barra = '█' * nivel_normalizado + '░' * (30 - nivel_normalizado)
                
                # Colores ANSI
                verde = '\033[92m'
                rojo = '\033[91m'
                amarillo = '\033[93m'
                azul = '\033[94m'
                reset = '\033[0m'
                
                if self.escuchando:
                    # Animación de micrófono activo
                    microfono = f"{rojo}🔴 GRABANDO{reset}"
                    pulso = "●" if int(time.time() * 2) % 2 else "○"
                    sys.stdout.write(f"{microfono} {pulso} [{verde}{barra}{reset}] {nivel_normalizado:2d}/30")
                else:
                    # Micrófono en espera
                    sys.stdout.write(f"{azul}🎤 ESCUCHANDO{reset} ◦ [{amarillo}{barra}{reset}] {nivel_normalizado:2d}/30")
                
                sys.stdout.flush()
                time.sleep(0.1)
            else:
                # Mostrar estado inactivo
                sys.stdout.write(f'\r🎙️ Micrófono desactivado{" " * 50}')
                sys.stdout.flush()
                time.sleep(0.5)

    def escuchar_continuamente(self):
        if not self.mic_enabled:
            self.visualizando = False
            return

        self.visualizando = True
        
        data = sd.rec(int(0.5 * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=1, dtype='float32')
        sd.wait()

        # Calcular nivel de audio para la visualización
        self.nivel_audio = np.sqrt(np.mean(data**2)) * 10

        if detectar_voz(data):
            if not self.escuchando:
                print(f"\n🔊 Voz detectada, grabando...")
                self.escuchando = True
                self.inicio_segmento = time.time()
                self.audio_buffer = []

            self.audio_buffer.append(data)
            self.tiempo_inicio_silencio = 0
        elif self.escuchando:
            if self.tiempo_inicio_silencio == 0:
                self.tiempo_inicio_silencio = time.time()
            elif time.time() - self.tiempo_inicio_silencio > 1.5:
                print(f"\n🛑 Silencio detectado. Procesando transcripción...")
                self.finalizar_grabacion()

        if self.escuchando and (time.time() - self.inicio_segmento > MAX_SEGMENT_DURATION):
            print(f"\n⏱️ Tiempo máximo alcanzado. Procesando transcripción...")
            self.finalizar_grabacion()

    def finalizar_grabacion(self):
        self.escuchando = False
        self.tiempo_inicio_silencio = 0

        if self.audio_buffer:
            audio_final = np.concatenate(self.audio_buffer, axis=0)
            wav.write("grabacion.wav", SAMPLE_RATE, audio_final)
            self.transcribir_con_whisper("grabacion.wav")
            os.remove("grabacion.wav")
            self.audio_buffer = []

    def transcribir_con_whisper(self, ruta_audio):
        try:
            print("\n🧠 Procesando con Whisper...")
            result = whisper_model.transcribe(ruta_audio, language="es")
            texto = result["text"].strip()
            if texto:
                print(f"\n📝 Transcripción: {texto}")
                print("=" * 60)
                self.publisher_.publish(String(data=texto))
            else:
                print("\n🤷 Whisper no detectó contenido claro.")
        except Exception as e:
            print(f"\n🚫 Error al transcribir con Whisper: {e}")

    def mostrar_estado_inicial(self):
        """Mostrar información inicial del sistema"""
        print("\n" + "=" * 60)
        print("🎤 SISTEMA DE TRANSCRIPCIÓN DE VOZ ACTIVADO")
        print("=" * 60)
        print("📊 Configuración:")
        print(f"   • Frecuencia de muestreo: {SAMPLE_RATE} Hz")
        print(f"   • Umbral de detección: {THRESHOLD}")
        print(f"   • Duración máxima: {MAX_SEGMENT_DURATION}ms")
        print("📋 Instrucciones:")
        print("   • Habla normalmente para activar la grabación")
        print("   • El sistema se activa automáticamente al detectar voz")
        print("   • Presiona Ctrl+C para salir")
        print("=" * 60)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceTranscriberNode()
    
    # Mostrar información inicial
    node.mostrar_estado_inicial()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n🛑 Cerrando sistema de transcripción...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("👋 ¡Hasta luego!")

if __name__ == '__main__':
    main()
