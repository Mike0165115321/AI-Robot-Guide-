import os
import shutil
from sentence_transformers import SentenceTransformer

# =================================================================
# 1. กำหนดค่าต่างๆ
# =================================================================
# โมเดลทั้งหมดที่คุณต้องการดาวน์โหลด (ในรูปแบบ [ชื่อใน Hugging Face Hub, ชื่อโฟลเดอร์สำหรับบันทึก])
MODELS_TO_DOWNLOAD = [
    # embedding model (E5-large)
    {"hub_name": "intfloat/multilingual-e5-large", "folder_name": "intfloat-multilingual-e5-large"},
    # BAAI model 1 (BGE-M3)
    {"hub_name": "BAAI/bge-m3", "folder_name": "BAAI-bge-m3"},
    # BAAI reranker model (BGE-reranker-base)
    {"hub_name": "BAAI/bge-reranker-base", "folder_name": "BAAI-bge-reranker-base"},
]

# **ตำแหน่งหลักในการจัดเก็บโมเดล (ใช้ Linux Path)**
BASE_DIR_LINUX = "/home/mikedev/MyModels/Model RAG" 

# =================================================================
# 2. ฟังก์ชันดาวน์โหลดและบันทึก
# =================================================================
def download_and_save_model(hub_name: str, folder_name: str):
    """
    ดาวน์โหลดโมเดลและบันทึกไปยังโฟลเดอร์ที่กำหนด
    """
    # กำหนด Path ปลายทาง
    save_path = os.path.join(BASE_DIR_LINUX, folder_name)
    
    print(f"\n--- 🚀 Processing Model: {hub_name} ---")

    # 1. ตรวจสอบและลบของเก่า (ตามที่คุณต้องการ)
    if os.path.exists(save_path):
        print(f"🗑️ Deleting existing folder: {save_path}")
        try:
            shutil.rmtree(save_path)
            print("    Successfully deleted old version.")
        except Exception as e:
            print(f"    ❌ Error deleting old version: {e}. Skipping download.")
            return

    try:
        # 2. โหลดโมเดลจาก Hugging Face Hub (ต้องใช้อินเทอร์เน็ต)
        print(f"⬇️ Downloading model from Hub...")
        model = SentenceTransformer(hub_name)
        
        # 3. บันทึกโมเดลลงใน Path ใหม่
        print(f"💾 Saving model to: {save_path}")
        model.save(save_path)
        
        print(f"✅ Download and save successful for {folder_name}")

    except Exception as e:
        print(f"❌ Critical Error occurred while processing {hub_name}: {e}")

# =================================================================
# 3. รันกระบวนการหลัก
# =================================================================
if __name__ == "__main__":
    # ตรวจสอบและสร้างโฟลเดอร์หลัก (ถ้ายังไม่มี)
    if not os.path.exists(BASE_DIR_LINUX):
        os.makedirs(BASE_DIR_LINUX)
        print(f"Created base directory: {BASE_DIR_LINUX}")
    
    # รันดาวน์โหลดสำหรับทุกโมเดล
    for model_info in MODELS_TO_DOWNLOAD:
        download_and_save_model(model_info["hub_name"], model_info["folder_name"])
        
    print("\n\n--- ✨ All model downloads completed. ---")
    print(f"All models are now stored in the directory: {BASE_DIR_LINUX}")