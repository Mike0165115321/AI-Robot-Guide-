import os
import redis
import json
from dotenv import load_dotenv

load_dotenv()

# Configuration
REDIS_HOST = os.getenv("REDIS_HOST", "localhost")
REDIS_PORT = int(os.getenv("REDIS_PORT", 6379))
REDIS_DB = int(os.getenv("REDIS_DB", 0))
QUEUE_NAME = "line_msg_queue"

class RedisClient:
    def __init__(self):
        self.client = redis.Redis(
            host=REDIS_HOST,
            port=REDIS_PORT,
            db=REDIS_DB,
            decode_responses=True # Return strings instead of bytes
        )

    def push_message(self, message_data: dict):
        """Push a message to the queue (Producer)"""
        try:
            self.client.lpush(QUEUE_NAME, json.dumps(message_data))
            print(f"[Redis] Pushed message to {QUEUE_NAME}")
        except Exception as e:
            print(f"[Redis] Error pushing message: {e}")
            raise e

    def pop_message(self):
        """Blocking pop a message from the queue (Consumer)"""
        # brpop returns a tuple (queue_name, data)
        # timeout=0 means block indefinitely
        try:
            result = self.client.brpop(QUEUE_NAME, timeout=0)
            if result:
                _, data = result
                return json.loads(data)
        except Exception as e:
            print(f"[Redis] Error popping message: {e}")
            return None
        return None

# Singleton instance
redis_client = RedisClient()
