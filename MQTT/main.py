import logging
from mongo_db import MongoDBHandler
from mqtt_receiver import MQTTHandler

# Configuração global de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def main():
    logging.info("🚀 Iniciando o serviço de ingestão MQTT -> MongoDB...")
    
    try:
        # 1. Inicializa a camada de banco de dados
        db_handler = MongoDBHandler()
        
        # 2. Inicializa a camada de MQTT, injetando a dependência do banco
        mqtt_handler = MQTTHandler(db_handler)
        mqtt_handler.connect()
        
        # 3. Inicia o loop infinito de escuta
        mqtt_handler.loop_forever()
        
    except KeyboardInterrupt:
        logging.info("🛑 Interrompido pelo usuário. Encerrando aplicação...")
    except Exception as e:
        logging.critical(f"💥 Erro fatal na aplicação: {e}")

if __name__ == '__main__':
    main()