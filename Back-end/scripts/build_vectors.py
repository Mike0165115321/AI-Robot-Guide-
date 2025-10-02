# /Back-end/scripts/build_vectors.py

import sys
import os
import json
import shutil
import time

current_script_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.abspath(os.path.join(current_script_dir, '..'))
sys.path.insert(0, backend_dir)

from core.database.mongodb_manager import MongoDBManager
from core.database.qdrant_manager import QdrantManager
from utils.helper_functions import create_synthetic_document

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

def create_embedding_text(data_item: dict) -> str:
    """
    สร้างเอกสารสังเคราะห์จากโครงสร้างข้อมูลใหม่เพื่อนำไปสร้าง Vector
    """
    title = data_item.get("title", "")
    topic = data_item.get("topic", "")
    summary = data_item.get("summary", "")
    
    details_list = []
    for detail_part in data_item.get("details", []):
        heading = detail_part.get("heading", "")
        content = detail_part.get("content", "")
        details_list.append(f"{heading}: {content}")
    details_text = "\n".join(details_list)

    keywords_text = ", ".join(data_item.get("keywords", []))

    # ประกอบร่างเป็นเอกสารสุดท้ายที่สมบูรณ์
    full_text = f"หัวข้อ: {title} ({topic})\nสรุป: {summary}\nรายละเอียด:\n{details_text}\nคำสำคัญ: {keywords_text}"
    return full_text.strip()

class VectorBuilder:
    def __init__(self):
        print("⚙️  Vector Builder initializing...")
        self.mongo_manager = MongoDBManager()
        self.qdrant_manager = QdrantManager()
        print("✅ Builder is ready.")

    def process_and_move_files(self, data_folder: str, processed_folder: str):
        print(f"\n--- 📚 Scanning for new data in '{data_folder}' ---")
        files_to_process = [f for f in os.listdir(data_folder) if f.endswith(".jsonl")]
        
        if not files_to_process:
            print("  - 🟡 No new data files found.")
            return

        print(f"📦 Found {len(files_to_process)} file(s) to process.")
        
        for filename in files_to_process:
            print(f"\n--- 🏭 Processing file: '{filename}' ---")
            file_path = os.path.join(data_folder, filename)
            
            with open(file_path, 'r', encoding='utf-8') as f:
                for line_num, line in enumerate(f, 1):
                    try:
                        data_item = json.loads(line)

                        if not isinstance(data_item, dict):
                            print(f"  - ⚠️ Skipping line {line_num}: Data is not a JSON object.")
                            continue

                        item_title = data_item.get("title")
                        if not item_title:
                            print(f"  - ⚠️ Skipping line {line_num}: missing 'title'.")
                            continue
                        
                        existing_item = self.mongo_manager.get_location_by_title(item_title)
                        if existing_item:
                            print(f"  - Skipping: '{item_title}' already exists in the database.")
                            continue
                        embedding_text = create_synthetic_document(data_item)
                        mongo_id = self.mongo_manager.add_location(data_item)
                        
                        # 2. ส่ง Embedding Text ไปสร้าง Vector
                        if mongo_id:
                            self.qdrant_manager.upsert_location(mongo_id, embedding_text)
                            print(f"  - Successfully processed and inserted '{item_title}'.")
                        
                    except json.JSONDecodeError:
                        print(f"  - ⚠️ Skipping malformed JSON on line {line_num}.")
                    except Exception as e:
                        print(f"  - ❌ An error occurred on line {line_num}: {e}")

            dest_path = os.path.join(processed_folder, filename)
            shutil.move(file_path, dest_path)
            print(f"  - ✅ Finished processing. Moved '{filename}' to processed folder.")

if __name__ == "__main__":
    DATA_SOURCE_FOLDER = os.path.join(project_root, 'core', 'database', 'data')
    PROCESSED_DATA_FOLDER = os.path.join(DATA_SOURCE_FOLDER, '_processed')
    os.makedirs(PROCESSED_DATA_FOLDER, exist_ok=True)
    
    print("\n" + "="*60)
    print("--- 🛠️  Starting Offline Data & Vector Construction  🛠️ ---")
    print("="*60)
    
    print("Waiting for database services to be ready...")
    time.sleep(5)

    builder = VectorBuilder()
    builder.process_and_move_files(
        data_folder=DATA_SOURCE_FOLDER,
        processed_folder=PROCESSED_DATA_FOLDER
    )
    
    print("\n" + "="*60)
    print("✅ Build process finished successfully!")
    print("="*60)