import sys
import os
import json
import shutil
import time
import asyncio
import aiofiles
import logging

current_script_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.abspath(os.path.join(current_script_dir, '..'))
sys.path.insert(0, backend_dir)

from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from utils.helper_functions import create_synthetic_document
from core.config import settings

# (ฟังก์ชัน create_embedding_text ไม่ได้ถูกเรียกใช้ แต่เก็บไว้ได้)
def create_embedding_text(data_item: dict) -> str:
    pass

class VectorBuilder:
    def __init__(self):
        print("⚙️  Vector Builder initializing... (V5.2 Async Version)")
        self.mongo_manager = MongoDBManager()
        self.qdrant_manager = QdrantManager()
        print("✅ Builder is ready.")

    async def initialize_services(self):
        await self.qdrant_manager.initialize()
        print("✅ QdrantManager (Async) initialized.")
        collection = self.mongo_manager.get_collection("nan_locations")
        if collection is not None:
            try:
                collection.create_index("slug", unique=True)
                print("✅ Ensured 'slug' field in MongoDB has a unique index.")
            except Exception as e:
                print(f"⚠️ Could not create unique index for 'slug': {e}")

    async def close_services(self):
        await self.qdrant_manager.close()
        print("✅ QdrantManager (Async) closed.")

    async def process_and_move_files(self, data_folder: str, processed_folder: str):
        print(f"\n--- 📚 Scanning for new data in '{data_folder}' ---")
        
        files_to_process = [f for f in os.listdir(data_folder) if f.endswith(".jsonl")]
        
        if not files_to_process:
            print(" - 🟡 No new data files found.")
            return

        print(f"📦 Found {len(files_to_process)} file(s) to process.")
        
        for filename in files_to_process:
            print(f"\n--- 🏭 Processing file: '{filename}' ---")
            file_path = os.path.join(data_folder, filename)
            
            line_num = 0  
            async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
                
                async for line in f:
                    line_num += 1  
                    try:
                        data_item = json.loads(line)

                        if not isinstance(data_item, dict):
                            print(f" - ⚠️ Skipping line {line_num}: Data is not a JSON object.")
                            continue

                        item_slug = data_item.get("slug")
                        if not item_slug:
                            print(f" - ⚠️ Skipping line {line_num}: missing 'slug'. (Run add_image_links.py first!)")
                            continue
                        
                        existing_item = await asyncio.to_thread(
                            self.mongo_manager.get_location_by_slug, item_slug
                        )
                        
                        if existing_item:
                            print(f" - Skipping: Slug '{item_slug}' already exists in the database.")
                            continue

                        embedding_text = await asyncio.to_thread(
                            create_synthetic_document, data_item
                        )
                        
                        mongo_id = await asyncio.to_thread(
                            self.mongo_manager.add_location, data_item
                        )
                        
                        if mongo_id:
                            # 🆕 Extract metadata for Qdrant payload
                            qdrant_metadata = {
                                "title": data_item.get("title"),
                                "slug": data_item.get("slug"),
                                "category": data_item.get("category"),
                                "district": (data_item.get("location_data") or {}).get("district"),
                                "sub_district": (data_item.get("location_data") or {}).get("sub_district")
                            }
                            await self.qdrant_manager.upsert_location(mongo_id, embedding_text, metadata=qdrant_metadata)
                            print(f" - Successfully processed and inserted '{item_slug}'.")
                        
                    except json.JSONDecodeError:
                        print(f" - ⚠️ Skipping malformed JSON on line {line_num}.")
                    except Exception as e:
                        print(f" - ❌ An error occurred on line {line_num}: {e}", exc_info=True)
                        raise e

            dest_path = os.path.join(processed_folder, filename)
            await asyncio.to_thread(shutil.move, file_path, dest_path)
            print(f" - ✅ Finished processing. Moved '{filename}' to processed folder.")

async def main():
    DATA_SOURCE_FOLDER = os.path.join(backend_dir, 'core', 'database', 'data')
    PROCESSED_DATA_FOLDER = os.path.join(DATA_SOURCE_FOLDER, '_processed')
    os.makedirs(PROCESSED_DATA_FOLDER, exist_ok=True)
    
    print("\n" + "="*60)
    print("--- 🛠️  Starting Offline Data & Vector Construction (V5.2 Async) 🛠️ ---")
    print("="*60)
    
    builder = VectorBuilder()
    success = False
    try:
        await builder.initialize_services()
        await builder.process_and_move_files(
            data_folder=DATA_SOURCE_FOLDER,
            processed_folder=PROCESSED_DATA_FOLDER
        )
        success = True  
    except Exception as e:
        print(f"\n❌ A critical error occurred during the build process: {e}")
        logging.error("Build process failed critically", exc_info=True)
    finally:
        await builder.close_services()
    
    print("\n" + "="*60)
    if success:
        print("✅ Build process finished successfully!")
    else:
        print("❌ Build process FAILED due to errors above.")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(main())