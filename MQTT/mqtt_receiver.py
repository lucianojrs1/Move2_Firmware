import random
import json
import logging #oq eh isso
from datetime import datetime, timezone
from paho.mqtt import client as mqtt_client
from config import MQTT_BROKER, MQTT_PORT, MQTT_TOPIC, MQTT_USERNAME, MQTT_PASSWORD

logger = logging.getLogger(__name__)

class MQTTHandler:
    def __init__(self, db_handler):
        self.db_handler = db_handler
        self.client_id = f'subscribe-{random.randint(0, 1000)}'
        self.client = self._create_client()

    def _create_client(self) -> mqtt_client:
        """Configura o cliente MQTT com callbacks e autenticação."""
        client = mqtt_client.Client(
            client_id=self.client_id,
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2,
            userdata={"db_handler": self.db_handler}
        )
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        client.tls_set()  # Obrigatório para porta 8883
        
        client.on_connect = self._on_connect
        client.on_message = self._on_message
        return client

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        """Callback executado ao conectar/reconectar ao broker."""
        if reason_code == 0:
            logger.info("✅ Conectado ao Broker MQTT com sucesso!")
            client.subscribe(MQTT_TOPIC)
            logger.info(f"📡 Inscrito no tópico: {MQTT_TOPIC}")
        else:
            logger.error(f"❌ Falha ao conectar ao MQTT, código: {reason_code}")

    def _on_message(self, client, userdata, msg):
        """Callback executado ao receber uma mensagem."""
        db_handler = userdata["db_handler"]
        
        # 1. Decodifica o payload
        try:
            payload_str = msg.payload.decode('utf-8')
        except UnicodeDecodeError:
            payload_str = msg.payload.hex()  # Fallback para dados binários

        # 2. Tenta parsear como JSON
        try:
            data = json.loads(payload_str)
        except json.JSONDecodeError:
            data = {"raw_payload": payload_str}

        # 3. Adiciona metadados de rastreamento
        data["mqtt_topic"] = msg.topic
        data["received_at"] = datetime.now(timezone.utc)  # Padrão correto no Python 3.12

        # 4. Salva no banco de dados
        db_handler.insert_message(data)

    def connect(self):
        """Inicia a conexão com o broker."""
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT)
        except Exception as e:
            logger.error(f"❌ Erro ao conectar ao broker MQTT: {e}")
            raise

    def loop_forever(self):
        """Mantém o cliente rodando e processando eventos de rede."""
        self.client.loop_forever()