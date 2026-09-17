import os
from dotenv import load_dotenv

# Carrega as variáveis do arquivo .env
load_dotenv()

# Configurações MQTT
MQTT_BROKER = os.getenv("MQTT_BROKER", "eaa7d5aa.ala.eu-central-1.emqxsl.com")
MQTT_PORT = int(os.getenv("MQTT_PORT", 8883))
MQTT_TOPIC = os.getenv("MQTT_TOPIC", "esp32/can_leitura")
MQTT_USERNAME = os.getenv("MQTT_USERNAME", "modcs")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", "12345678")

# Configurações MongoDB
MONGO_URI = os.getenv("MONGO_URI", "mongodb+srv://lucianojosejr15876_db_user:pEG2relvdQj6Xg1a@modcsimongodb.qsndk8f.mongodb.net/?appName=MoDCSIMongoDB")
MONGO_DB_NAME = os.getenv("MONGO_DB_NAME", "MoDCSIMongoDB")
MONGO_COLLECTION_NAME = os.getenv("MONGO_COLLECTION_NAME", "mensagens_recebidas")