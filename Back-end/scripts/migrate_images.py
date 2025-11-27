import os
import sys
from pathlib import Path
import logging

# Add backend to sys.path
current_script_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.abspath(os.path.join(current_script_dir, '..'))
sys.path.insert(0, backend_dir)

from core.database.mongodb_manager import MongoDBManager

logging.basicConfig(level=logging.INFO)

def migrate_images():
    print("🚀 Starting Image Migration...")
    
    mongo_manager = MongoDBManager()
    collection = mongo_manager.get_collection("image_metadata")
    
    if collection is None:
        print("❌ Failed to get collection.")
        return

    # Clear existing data
    collection.delete_many({})
    print("🗑️  Cleared existing image metadata.")

    static_dir = Path(backend_dir) / "static" / "images"
    if not static_dir.is_dir():
        print(f"❌ Static directory not found: {static_dir}")
        return

    image_extensions = {".jpg", ".jpeg", ".png", ".webp"}
    count = 0

    for f in static_dir.iterdir():
        if f.is_file() and f.suffix.lower() in image_extensions:
            file_path_str = f"/static/images/{f.name}"
            prefix = ""
            
            if "-" in f.name:
                prefix = f.name.rsplit("-", 1)[0] + "-"
            elif "_" in f.name:
                prefix = f.name.split("_")[0] + "_"
            
            doc = {
                "filename": f.name,
                "url": file_path_str,
                "prefix": prefix,
                "path": str(f)
            }
            
            collection.insert_one(doc)
            count += 1
    
    print(f"✅ Migrated {count} images to MongoDB.")

if __name__ == "__main__":
    migrate_images()
