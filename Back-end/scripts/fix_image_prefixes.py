#!/usr/bin/env python3
"""
🖼️ Fix Image Prefixes Script
============================
สคริปต์นี้ใช้เพื่อจับคู่ไฟล์ภาพที่มีอยู่ใน /static/images/ กับ records ใน MongoDB
และอัพเดท metadata.image_prefix ให้อัตโนมัติ

วิธีใช้:
    python fix_image_prefixes.py --dry-run   # ดูตัวอย่างก่อนไม่อัพเดทจริง
    python fix_image_prefixes.py             # อัพเดทจริง
"""

import os
import sys
import argparse
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from core.database.mongodb_manager import MongoDBManager


def get_image_prefixes_from_folder(image_dir: Path) -> set:
    """สแกนโฟลเดอร์ภาพและดึง prefixes ทั้งหมดออกมา"""
    prefixes = set()
    if not image_dir.exists():
        print(f"❌ โฟลเดอร์ {image_dir} ไม่มีอยู่")
        return prefixes
    
    for f in image_dir.iterdir():
        if not f.is_file():
            continue
        if f.suffix.lower() not in ('.jpg', '.jpeg', '.png', '.webp'):
            continue
        
        # Extract prefix: "wat-kong-01.jpg" -> "wat-kong"
        base = f.stem  # "wat-kong-01"
        parts = base.rsplit('-', 1)
        if len(parts) == 2 and parts[1].isdigit():
            prefix = parts[0]
        else:
            prefix = base
        prefixes.add(prefix)
    
    return prefixes


def find_best_matching_prefix(slug: str, all_prefixes: set, debug: bool = False) -> str | None:
    """
    ค้นหา prefix ที่ตรงกับ slug ที่สุด
    
    ลำดับความสำคัญ:
    1. ตรงเป๊ะ (slug == prefix)
    2. Slug เริ่มต้นด้วย prefix หรือ prefix เริ่มต้นด้วย slug
    3. Prefix + suffix pattern (e.g., "wat-kong-nan" -> "wat-kong")
    4. Fuzzy substring matching
    """
    slug_clean = slug.lower().strip()
    
    if debug:
        print(f"    [DEBUG] Checking slug: {slug_clean}")
    
    # 1. Exact match
    if slug_clean in all_prefixes:
        if debug:
            print(f"    [DEBUG] Exact match found: {slug_clean}")
        return slug_clean
    
    # 2. Slug starts with prefix or prefix starts with slug
    for prefix in all_prefixes:
        if slug_clean.startswith(prefix) or prefix.startswith(slug_clean):
            if debug:
                print(f"    [DEBUG] Starts-with match: {prefix}")
            return prefix
    
    # 3. Prefix + suffix pattern (e.g., "wat-kong-nan" -> "wat-kong")
    common_suffixes = ['-nan', '-province', '-prov', '-area', '-district', '-nan-province', '-nan-city']
    for suffix in common_suffixes:
        if slug_clean.endswith(suffix):
            base_slug = slug_clean[:-len(suffix)]
            if base_slug in all_prefixes:
                if debug:
                    print(f"    [DEBUG] Suffix removal match: {base_slug} (removed {suffix})")
                return base_slug
    
    # 4. Find prefixes that are substrings of slug (longest match wins)
    matches = []
    for prefix in all_prefixes:
        # Check both directions
        if prefix in slug_clean or slug_clean in prefix:
            matches.append(prefix)
        # Also check without hyphens
        elif prefix.replace('-', '') in slug_clean.replace('-', ''):
            matches.append(prefix)
        elif slug_clean.replace('-', '') in prefix.replace('-', ''):
            matches.append(prefix)
    
    if matches:
        best = max(matches, key=len)
        if debug:
            print(f"    [DEBUG] Substring match: {best} (from {matches})")
        return best
    
    if debug:
        print(f"    [DEBUG] No match found")
    
    return None


def main():
    parser = argparse.ArgumentParser(description='Fix image prefixes in MongoDB')
    parser.add_argument('--dry-run', action='store_true', help='Show what would be updated without making changes')
    parser.add_argument('--debug', action='store_true', help='Show debug output for matching')
    parser.add_argument('--fix-incorrect', action='store_true', help='Also fix incorrect prefixes that point to non-existent images')
    args = parser.parse_args()
    
    print("🖼️  Fix Image Prefixes Script")
    print("=" * 50)
    
    # Initialize
    image_dir = Path(__file__).resolve().parent.parent / "static" / "images"
    print(f"📁 Image Directory: {image_dir}")
    
    # Get all image prefixes
    all_prefixes = get_image_prefixes_from_folder(image_dir)
    print(f"🔍 พบ {len(all_prefixes)} prefixes จากไฟล์ภาพ")
    
    # Connect to MongoDB
    db = MongoDBManager()
    collection = db.get_collection("nan_locations")
    
    if collection is None:
        print("❌ ไม่สามารถเชื่อมต่อ MongoDB collection 'nan_locations'")
        return
    
    # Get all locations
    all_locations = list(collection.find({}))
    print(f"📍 พบ {len(all_locations)} locations ใน MongoDB")
    
    updates = []
    
    for loc in all_locations:
        slug = loc.get('slug', '')
        metadata = loc.get('metadata', {}) or {}
        current_prefix = metadata.get('image_prefix')
        
        # Case 1: No prefix at all
        if not current_prefix:
            matched_prefix = find_best_matching_prefix(slug, all_prefixes, debug=args.debug)
            if matched_prefix:
                updates.append({
                    'slug': slug,
                    'title': loc.get('title', 'N/A')[:50],
                    '_id': loc['_id'],
                    'new_prefix': matched_prefix,
                    'reason': 'missing prefix'
                })
        
        # Case 2: Has prefix but it's incorrect (doesn't match any image file)
        elif args.fix_incorrect:
            # Check if current prefix has matching images
            # Clean the prefix - remove trailing dash and common suffixes
            clean_prefix = current_prefix.rstrip('-')
            
            # Also try removing -nan suffix
            base_prefix = clean_prefix
            if clean_prefix.endswith('-nan'):
                base_prefix = clean_prefix[:-4]  # Remove "-nan"
            
            # Check for EXACT match only - prefix must equal an image file prefix
            has_images = (clean_prefix in all_prefixes) or (base_prefix in all_prefixes)
            
            if not has_images:
                # Try to find a correct prefix
                matched_prefix = find_best_matching_prefix(slug, all_prefixes, debug=args.debug)
                if matched_prefix:
                    updates.append({
                        'slug': slug,
                        'title': loc.get('title', 'N/A')[:50],
                        '_id': loc['_id'],
                        'old_prefix': current_prefix,
                        'new_prefix': matched_prefix,
                        'reason': 'incorrect prefix'
                    })
    
    # Separate by reason for better reporting
    missing = [u for u in updates if u.get('reason') == 'missing prefix']
    incorrect = [u for u in updates if u.get('reason') == 'incorrect prefix']
    
    print(f"\n📊 ผลการวิเคราะห์:")
    print(f"   - ไม่มี prefix เลย และจับคู่ได้: {len(missing)}")
    print(f"   - มี prefix ผิด และจับคู่ใหม่ได้: {len(incorrect)}")
    print(f"   - รวมที่ต้องอัพเดท: {len(updates)}")
    
    if not updates:
        print("\n✨ ไม่มี locations ที่ต้องอัพเดท!")
        return
    
    # Show updates
    if missing:
        print("\n🆕 รายการที่ไม่มี prefix (จะเพิ่มใหม่):")
        print("-" * 80)
        for u in missing:
            print(f"  • {u['slug']:45} -> {u['new_prefix']}")
    
    if incorrect:
        print("\n🔧 รายการที่มี prefix ผิด (จะแก้ไข):")
        print("-" * 80)
        for u in incorrect:
            print(f"  • {u['slug']:35} [{u['old_prefix'][:15]:15}] -> {u['new_prefix']}")
    
    print("-" * 80)
    
    if args.dry_run:
        print("\n🔍 [DRY RUN] ไม่มีการเปลี่ยนแปลงจริง")
        return
    
    # Apply updates
    print(f"\n🔄 กำลังอัพเดท {len(updates)} records...")
    
    success_count = 0
    for u in updates:
        try:
            # Use trailing dash for consistency with existing data
            new_prefix_with_dash = u['new_prefix'] + '-' if not u['new_prefix'].endswith('-') else u['new_prefix']
            
            result = collection.update_one(
                {'_id': u['_id']},
                {'$set': {'metadata.image_prefix': new_prefix_with_dash}}
            )
            if result.modified_count > 0:
                success_count += 1
                print(f"  ✅ Updated: {u['slug']}")
            else:
                print(f"  ⚠️ No change: {u['slug']}")
        except Exception as e:
            print(f"  ❌ Failed {u['slug']}: {e}")
    
    print(f"\n🎉 สำเร็จ! อัพเดท {success_count}/{len(updates)} records")
    print("💡 กรุณา refresh หน้าเว็บเพื่อดูภาพที่แสดง")


if __name__ == "__main__":
    main()
