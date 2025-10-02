# /Back-end/api/main.py (Final Clean Version for Pure JS VAD)

import uvicorn
import os
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from api.routers import admin_api, chat_api
from pathlib import Path 
from utils.file_cleaner import start_background_cleanup

app = FastAPI(
    title="AI Robot Guide จังหวัดน่าน API",
    description="API สำหรับจัดการข้อมูลและให้บริการสนทนาสำหรับ AI Guide",
    version="1.0.0"
)

app.mount("/static", StaticFiles(directory="../static"), name="static")

# --- Middleware (CORS) ---
origins = ["*"]
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(admin_api.router, prefix="/api/admin/locations") 
app.include_router(chat_api.router, prefix="/api/chat")   

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
FRONTEND_DIR = PROJECT_ROOT / "frontend"

print(f"✅ Serving frontend from directory: {FRONTEND_DIR}")
if not FRONTEND_DIR.is_dir():
    print(f"❌ CRITICAL ERROR: Frontend directory not found at the calculated path! Check Path Configuration.")

app.mount("/assets", StaticFiles(directory=FRONTEND_DIR / "assets"), name="assets")

templates = Jinja2Templates(directory=str(FRONTEND_DIR))

@app.get("/{full_path:path}", include_in_schema=False)
async def serve_frontend(request: Request, full_path: str):
    path_map = {
        "": "index.html",
        "chat": "chat.html",
        "admin": "admin.html",
        "robot_avatar": "robot_avatar.html"
    }
    file_to_serve = path_map.get(full_path, full_path)
    
    template_path = FRONTEND_DIR / file_to_serve
    if not template_path.is_file():
        return templates.TemplateResponse("index.html", {"request": request})

    return templates.TemplateResponse(file_to_serve, {"request": request})

start_background_cleanup()

if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=9090, reload=True)