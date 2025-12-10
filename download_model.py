import os
import shutil
from sentence_transformers import SentenceTransformer
from transformers import AutoModelForCausalLM, AutoTokenizer

# ================================================================
# 1. Config
# ================================================================
MODELS_TO_DOWNLOAD = [
    {"hub_name": "Xtra-Computing/XtraGPT-7B", "folder_name": "Xtra-Computing-XtraGPT-7B", "type": "llm"},
]

BASE_DIR_LINUX = "/home/mikedev/MyModels/Model-RAG"


# ================================================================
# 2. Download functions
# ================================================================
def download_embedding_model(hub_name, save_path):
    print(f"⬇️ Downloading embedding model: {hub_name}")
    model = SentenceTransformer(hub_name)
    model.save(save_path)
    print("✅ Saved embedding model")


def download_llm_model(hub_name, save_path):
    print(f"⬇️ Downloading LLM model: {hub_name}")

    tokenizer = AutoTokenizer.from_pretrained(hub_name, trust_remote_code=True)
    model = AutoModelForCausalLM.from_pretrained(
        hub_name,
        trust_remote_code=True
    )

    tokenizer.save_pretrained(save_path)
    model.save_pretrained(save_path)

    print("✅ Saved LLM model")


def download_and_save_model(info):
    hub_name = info["hub_name"]
    folder_name = info["folder_name"]
    model_type = info["type"]

    save_path = os.path.join(BASE_DIR_LINUX, folder_name)

    print(f"\n--- 🚀 Processing: {hub_name} ---")

    if os.path.exists(save_path):
        print(f"🗑️ Removing old folder: {save_path}")
        shutil.rmtree(save_path)

    os.makedirs(save_path, exist_ok=True)

    try:
        if model_type == "embed":
            download_embedding_model(hub_name, save_path)
        elif model_type == "llm":
            download_llm_model(hub_name, save_path)

        print(f"🎉 Completed: {hub_name}")

    except Exception as e:
        print(f"❌ Error while processing {hub_name}: {e}")


# ================================================================
# 3. Main
# ================================================================
if __name__ == "__main__":
    if not os.path.exists(BASE_DIR_LINUX):
        os.makedirs(BASE_DIR_LINUX)

    for info in MODELS_TO_DOWNLOAD:
        download_and_save_model(info)

    print("\n✨ All downloads completed.")

#   {"hub_name": "intfloat/multilingual-e5-large", "folder_name": "intfloat-multilingual-e5-large", "type": "embed"},
#   {"hub_name": "BAAI/bge-m3", "folder_name": "BAAI-bge-m3", "type": "embed"},
#   {"hub_name": "BAAI/bge-reranker-base", "folder_name": "BAAI-bge-reranker-base", "type": "embed"}, 