from datetime import datetime
from typing import Optional, Dict, Any
from core.database.mongodb_manager import MongoDBManager

# Simple logger instance
db_manager = MongoDBManager()

def log_admin_action(user_username: str, action: str, target_slug: str, details: Optional[Dict[str, Any]] = None):
    """
    Logs an admin action to the 'admin_logs' collection.
    
    Args:
        user_username: Who performed the action (e.g. 'admin')
        action: What happened (e.g. 'UPDATE_LOCATION', 'DELETE_LOCATION', 'LOGIN')
        target_slug: The identifier of the object affected (e.g. location slug)
        details: Dictionary containing more info (e.g. changed fields, old values)
    """
    try:
        log_entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "user": user_username,
            "action": action,
            "target": target_slug,
            "details": details or {}
        }
        
        # We access the internal db object directly or use a method if available.
        # Assuming get_collection returns the collection object.
        collection = db_manager.get_collection("admin_logs")
        if collection is not None:
             collection.insert_one(log_entry)
        else:
            print("Error: Could not get admin_logs collection")

    except Exception as e:
        print(f"Failed to write admin log: {e}")
