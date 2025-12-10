
import asyncio
import os
from pathlib import Path
from motor.motor_asyncio import AsyncIOMotorClient
from core.config import settings

# Setup logging
import logging
logging.basicConfig(level=logging.INFO)

async def check_images():
    print("Connecting to Mongo:", settings.MONGO_URI)
    client = AsyncIOMotorClient(settings.MONGO_URI)
    db = client[settings.MONGO_DATABASE_NAME]
    
    locations_col = db["nan_locations"]
    images_col = db["image_metadata"]
    
    # 1. Get some locations
    locations = await locations_col.find({"doc_type": "Location"}).to_list(10)
    print(f"Found {len(locations)} locations to check.")
    
    for loc in locations:
        title = loc.get("title", "Unknown")
        prefix = loc.get("metadata", {}).get("image_prefix")
        print(f"\nLocation: {title}")
        print(f"  Prefix: {prefix}")
        
        if not prefix:
            print("  [WARN] No image_prefix")
            continue
            
        # 2. Check images for prefix
        imgs = await images_col.find({"prefix": prefix}).to_list(None)
        if not imgs:
            print(f"  [WARN] No images found for prefix '{prefix}'")
        
        for img in imgs:
            url = img.get("url")
            print(f"    Image URL: {url}")
            
            # 3. Check if local file exists
            if url and not url.startswith("http"):
                # Remove leading slash if present
                clean_path = url.lstrip('/')
                
                # Check in static
                static_path = Path("static") / clean_path.replace("static/", "")
                frontend_assets_path = Path("../frontend/assets") / clean_path.replace("assets/", "")
                
                # Try to guess where it is
                # If url is "/static/images/foo.jpg", likely in static/images/foo.jpg
                # If url is "images/foo.jpg", might be assets/images/foo.jpg
                
                possible_paths = []
                if "static" in url:
                     possible_paths.append(Path("static") / url.split("static/")[-1])
                if "assets" in url:
                     possible_paths.append(Path("../frontend/assets") / url.split("assets/")[-1])
                
                # Add check for direct path relative to Back-end
                possible_paths.append(Path(clean_path))
                
                found = False
                for p in possible_paths:
                    if p.exists():
                        print(f"      [OK] File exists at: {p}")
                        found = True
                        break
                
                if not found:
                    print(f"      [MISSING] File NOT found. Checked: {[str(p) for p in possible_paths]}")

    client.close()

if __name__ == "__main__":
    asyncio.run(check_images())
