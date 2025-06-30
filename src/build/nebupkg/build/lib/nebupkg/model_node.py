import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import ollama
import time
from datetime import datetime
import json
import threading
import requests
import pygame
import io

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        self.API_KEY = "sk_ca96214557287b6c31ee53a745a3fd20fcd7e487abc12827"
        self.VOICE_ID = "8tm9IYjg8ybDoxe8w6Rk"
        self.TTS_URL = f"https://api.elevenlabs.io/v1/text-to-speech/{self.VOICE_ID}"

        self.conversation_count = 0
        self.detected_name = None
        self.current_emotion = None
        self.chat_history = []

        self.termination_keywords = [
            "adiós", "adios", "hasta luego", "chao", "bye", "salir", 
            "terminar", "finalizar", "despedida", "nos vemos"
        ]

        self.model = "llama3"
        self.base_system_prompt = """Eres Nebu, actua como un terapeuta. 
Características importantes:
- Eres moralmente neutral
- Responde de forma concisa y natural (máximo 2-3 oraciones)
- Mantén un tono cálido y profesional
- Responde en español
- Sé directo y útil en tus respuestas
- Sigues códigos éticos de tu profesión (terapeuta).
- Tienes una visión positiva del ser humano."""

        self.text_subscriber = self.create_subscription(String, 'transcripcion_voz', self.text_callback, 10)
        self.name_subscriber = self.create_subscription(String, 'person_name', self.name_callback, 10)
        self.emotion_subscriber = self.create_subscription(String, 'emocion_actual', self.emotion_callback, 10)

        self.response_publisher = self.create_publisher(String, 'llm_response', 10)
        self.tts_control_publisher = self.create_publisher(Bool, 'tts_complete', 10)

        pygame.mixer.init()

        self.get_logger().info('🤖 Nodo LLM iniciado con TTS y memoria conversacional')
        self.initialize_system()

    def name_callback(self, msg):
        self.detected_name = msg.data.strip()
        self.get_logger().info(f'📛 Nombre detectado recibido: {self.detected_name}')

    def emotion_callback(self, msg):
        self.current_emotion = msg.data.strip()
        self.get_logger().info(f'😊 Emoción actual recibida: {self.current_emotion}')
    
    def check_termination_keywords(self, text):
        return any(kw in text.lower() for kw in self.termination_keywords)

    def is_emotion_query(self, text):
        triggers = [
            "mi emoción", "mi estado emocional", "cómo me siento", 
            "cómo estoy", "mi sentimiento", "qué emoción tengo", "cómo me veo", "cuál es mi emoción"
        ]
        return any(trigger in text.lower() for trigger in triggers)

    def text_to_speech(self, text):
        try:
            headers = {
                "Accept": "audio/mpeg",
                "Content-Type": "application/json",
                "xi-api-key": self.API_KEY
            }
            data = {
                "text": text,
                "model_id": "eleven_monolingual_v1",
                "voice_settings": {
                    "stability": 0.5,
                    "similarity_boost": 0.75
                }
            }
            response = requests.post(self.TTS_URL, json=data, headers=headers)
            return response.content if response.status_code == 200 else None
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")
            return None

    def play_audio(self, audio_data):
        try:
            audio_io = io.BytesIO(audio_data)
            pygame.mixer.music.load(audio_io)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.wait(100)
        except Exception as e:
            self.get_logger().error(f'Error al reproducir audio: {e}')
    
    def signal_tts_complete(self):
        self.tts_control_publisher.publish(Bool(data=True))
        self.get_logger().info('📡 TTS completado → Activando micrófono')

    def signal_tts_start(self):
        self.tts_control_publisher.publish(Bool(data=False))
        self.get_logger().info('📡 TTS iniciado → Desactivando micrófono')
    
    def initialize_system(self):
        try:
            self.get_logger().info('Verificando conexión con Ollama...')
            response = ollama.chat(model=self.model, messages=[{"role": "user", "content": "Hola"}])
            if response and "message" in response:
                self.get_logger().info('✅ Conexión con Ollama exitosa')
                threading.Timer(2.0, self.send_welcome_message).start()
        except Exception as e:
            self.get_logger().error(f'Error al conectar con Ollama: {e}')

    def send_welcome_message(self):
        text = "¡Hola! Soy Nebu, tu asistente. Estoy listo para ayudarte."
        if self.current_emotion:
            text += f" Veo que te sientes {self.current_emotion}."
        self.publish_response(text)
        audio = self.text_to_speech(text)
        if audio:
            self.play_audio(audio)
        self.signal_tts_complete()

    def text_callback(self, msg):
        text = msg.data.strip()
        self.conversation_count += 1
        self.get_logger().info(f'📝 Entrada #{self.conversation_count}: "{text}"')

        self.signal_tts_start()

        if self.check_termination_keywords(text):
            self.handle_goodbye()
            return

        if self.is_emotion_query(text):
            self.handle_emotion_query()
        else:
            self.process_conversation(text)

    def handle_goodbye(self):
        text = f"{self.detected_name + ', ' if self.detected_name else ''}¡Hasta luego! Si necesitas algo más, aquí estaré."
        self.publish_response(text)
        audio = self.text_to_speech(text)
        if audio:
            self.play_audio(audio)
        self.signal_tts_complete()

    def handle_emotion_query(self):
        if self.current_emotion:
            response = f"Actualmente te sientes {self.current_emotion.lower()}."
        else:
            response = "Aún no tengo información sobre tu estado emocional."
        if self.detected_name:
            response = f"{self.detected_name}, {response}"
        self.publish_response(response)
        audio = self.text_to_speech(response)
        if audio:
            self.play_audio(audio)
        self.signal_tts_complete()

    def process_conversation(self, user_text):
        try:
            system_prompt = self.base_system_prompt
            if self.detected_name:
                system_prompt += f" Estás conversando con {self.detected_name}."
            if self.current_emotion:
                system_prompt += f" El usuario actualmente se siente {self.current_emotion}."

            messages = [{"role": "system", "content": system_prompt}] + self.chat_history
            messages.append({"role": "user", "content": user_text})
            
            start = time.time()
            response = ollama.chat(model=self.model, messages=messages)
            elapsed = time.time() - start
            
            if response and "message" in response and "content" in response["message"]:
                llm_response = response["message"]["content"].strip()
                if self.detected_name:
                    llm_response = f"{self.detected_name}, {llm_response}"

                self.chat_history.extend([
                    {"role": "user", "content": user_text},
                    {"role": "assistant", "content": llm_response}
                ])
                self.get_logger().info(f'Respuesta ({elapsed:.2f}s): {llm_response}')
                self.publish_response(llm_response)

                audio = self.text_to_speech(llm_response)
                if audio:
                    self.play_audio(audio)
                self.signal_tts_complete()
            else:
                self.handle_error_response()
        except Exception as e:
            self.get_logger().error(f'Error procesando conversación: {e}')
            self.handle_error_response()

    def handle_error_response(self):
        text = "Disculpa, tuve un problema procesando tu solicitud. ¿Puedes repetir?"
        self.publish_response(text)
        audio = self.text_to_speech(text)
        if audio:
            self.play_audio(audio)
        self.signal_tts_complete()

    def publish_response(self, text):
        self.response_publisher.publish(String(data=text))
        self.get_logger().info(f'📤 Respuesta enviada: "{text}"')

    def destroy_node(self):
        self.get_logger().info('🧹 Limpiando recursos...')
        pygame.mixer.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = LLMNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n🛑 Interrupción por teclado')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print('👋 Nodo LLM cerrado')

if __name__ == '__main__':
    main()
    
    
    
