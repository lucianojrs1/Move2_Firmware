import logging
from pymongo import MongoClient
from pymongo.errors import PyMongoError
from config import MONGO_URI, MONGO_DB_NAME, MONGO_COLLECTION_NAME

logger = logging.getLogger(__name__)

class MongoDBHandler:
    def __init__(self):
        self.client = None
        self.collection = None
        self.connect()

    def connect(self):
        """Estabelece conexão com o MongoDB e valida com um ping."""
        try:
            self.client = MongoClient(MONGO_URI)
            self.client.admin.command('ping')
            db = self.client[MONGO_DB_NAME]
            self.collection = db[MONGO_COLLECTION_NAME]
            logger.info("✅ Conectado ao MongoDB com sucesso!")
        except PyMongoError as e:
            logger.error(f"❌ Falha ao conectar ao MongoDB: {e}")
            raise

    def insert_message(self, data: dict):
        """Insere um dicionário de dados na coleção do MongoDB."""
        try:
            result = self.collection.insert_one(data)
            logger.info(f"💾 Dado salvo no MongoDB. ID: {result.inserted_id}")
            return result.inserted_id
        except PyMongoError as e:
            logger.error(f"⚠️ Erro ao salvar no MongoDB: {e}")
            raise