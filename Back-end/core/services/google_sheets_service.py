"""
Google Sheets Sync Service
รองรับการ sync ข้อมูลจาก Google Sheets เข้าสู่ MongoDB
"""

import os
import json
import logging
from typing import List, Dict, Any, Optional, Tuple
from datetime import datetime
from pathlib import Path

import gspread
from gspread import Spreadsheet, Worksheet

# Path to credentials file
CREDENTIALS_PATH = Path(__file__).parent.parent.parent / "credentials" / "still-toolbox-479616-e4-8901cbba2bcf.json"


class SyncResult:
    """ผลลัพธ์การ sync"""
    def __init__(self):
        self.created = 0
        self.updated = 0
        self.deleted = 0
        self.errors: List[str] = []
        self.timestamp = datetime.now().isoformat()
    
    def to_dict(self):
        return {
            "created": self.created,
            "updated": self.updated,
            "deleted": self.deleted,
            "errors": self.errors,
            "timestamp": self.timestamp
        }


class GoogleSheetsService:
    """
    Service สำหรับ sync ข้อมูลจาก Google Sheets
    รองรับทั้ง Polling และ Webhook mode
    """
    
    def __init__(self, mongo_manager=None):
        self.client: Optional[gspread.Client] = None
        self.spreadsheet: Optional[Spreadsheet] = None
        self.worksheet: Optional[Worksheet] = None
        self.mongo = mongo_manager
        self.sheet_id: Optional[str] = None
        self.last_sync: Optional[str] = None
        
        # Required columns mapping (Sheet column → DB field)
        self.column_mapping = {
            "slug": "slug",
            "title": "title", 
            "category": "category",
            "topic": "topic",
            "summary": "summary",
            "keywords": "keywords",  # comma-separated in sheet
        }
    
    def connect(self, sheet_id: str = None, sheet_url: str = None) -> bool:
        """
        เชื่อมต่อ Google Sheet
        
        Args:
            sheet_id: ID ของ sheet (ส่วนยาวๆ ใน URL)
            sheet_url: URL เต็มของ sheet
        
        Returns:
            True ถ้าเชื่อมต่อสำเร็จ
        """
        try:
            # Initialize client
            if not self.client:
                if not CREDENTIALS_PATH.exists():
                    logging.error(f"❌ Credentials file not found: {CREDENTIALS_PATH}")
                    return False
                
                self.client = gspread.service_account(filename=str(CREDENTIALS_PATH))
                logging.info("✅ Google Sheets client initialized")
            
            # Extract sheet_id from URL if needed
            if sheet_url and not sheet_id:
                # URL format: https://docs.google.com/spreadsheets/d/SHEET_ID/edit
                parts = sheet_url.split("/d/")
                if len(parts) > 1:
                    sheet_id = parts[1].split("/")[0]
            
            if not sheet_id:
                logging.error("❌ No sheet_id or sheet_url provided")
                return False
            
            # Open spreadsheet
            self.spreadsheet = self.client.open_by_key(sheet_id)
            self.worksheet = self.spreadsheet.sheet1  # Use first sheet
            self.sheet_id = sheet_id
            
            logging.info(f"✅ Connected to sheet: {self.spreadsheet.title}")
            return True
            
        except gspread.exceptions.SpreadsheetNotFound:
            logging.error(f"❌ Sheet not found or not shared with service account")
            return False
        except Exception as e:
            logging.error(f"❌ Failed to connect to Google Sheet: {e}")
            return False
    
    def fetch_all_rows(self) -> List[Dict[str, Any]]:
        """
        ดึงข้อมูลทั้งหมดจาก Sheet
        
        Returns:
            List of dict (แต่ละ row เป็น dict)
        """
        if not self.worksheet:
            logging.error("❌ Not connected to any sheet")
            return []
        
        try:
            # Get all records (assumes first row is header)
            records = self.worksheet.get_all_records()
            logging.info(f"📊 Fetched {len(records)} rows from sheet")
            return records
        except Exception as e:
            logging.error(f"❌ Failed to fetch rows: {e}")
            return []
    
    def _normalize_row(self, row: Dict[str, Any]) -> Dict[str, Any]:
        """แปลง row จาก Sheet ให้ตรงกับ DB schema"""
        normalized = {}
        
        for sheet_col, db_field in self.column_mapping.items():
            value = row.get(sheet_col, "")
            
            # Handle keywords (comma-separated → list)
            if db_field == "keywords" and isinstance(value, str):
                normalized[db_field] = [k.strip() for k in value.split(",") if k.strip()]
            else:
                normalized[db_field] = value if value else None
        
        # Add default metadata
        normalized["metadata"] = {
            "synced_from": "google_sheets",
            "sheet_id": self.sheet_id,
            "sync_time": datetime.now().isoformat()
        }
        
        return normalized
    
    def detect_changes(self, sheet_data: List[Dict], db_data: List[Dict]) -> Dict[str, List]:
        """
        เปรียบเทียบข้อมูลจาก Sheet กับ DB เพื่อหา changes
        
        ⚠️ สำคัญ: จะลบเฉพาะข้อมูลที่ sync มาจาก Google Sheets นี้เท่านั้น
        ข้อมูลจากแหล่งอื่น (manual entry, bulk import) จะไม่ถูกลบ
        
        Returns:
            Dict with keys: to_create, to_update, to_delete
        """
        # Build lookup by slug
        db_by_slug = {doc.get("slug"): doc for doc in db_data if doc.get("slug")}
        sheet_by_slug = {}
        
        for row in sheet_data:
            slug = row.get("slug")
            if slug:
                sheet_by_slug[slug] = self._normalize_row(row)
        
        changes = {
            "to_create": [],
            "to_update": [],
            "to_delete": []
        }
        
        # Find new and updated
        for slug, sheet_row in sheet_by_slug.items():
            if slug not in db_by_slug:
                # New row
                changes["to_create"].append(sheet_row)
            else:
                # Check if updated (compare key fields)
                db_row = db_by_slug[slug]
                if self._has_changes(db_row, sheet_row):
                    sheet_row["_id"] = db_row.get("_id")
                    changes["to_update"].append(sheet_row)
        
        # Find deleted - ONLY for records that were synced from THIS Google Sheet
        # ⚠️ ไม่ลบข้อมูลจากแหล่งอื่น (manual entry, bulk import, etc.)
        for slug, db_row in db_by_slug.items():
            if slug not in sheet_by_slug:
                # Check if this record came from Google Sheets sync
                metadata = db_row.get("metadata", {})
                synced_from = metadata.get("synced_from", "")
                synced_sheet_id = metadata.get("sheet_id", "")
                
                # Only delete if it was synced from THIS specific sheet
                if synced_from == "google_sheets" and synced_sheet_id == self.sheet_id:
                    changes["to_delete"].append(db_row)
                    logging.info(f"🗑️ Will delete (synced from this sheet): {slug}")
                else:
                    logging.debug(f"⏭️ Skipping delete (not from this sheet): {slug}")
        
        logging.info(f"📊 Changes detected - Create: {len(changes['to_create'])}, Update: {len(changes['to_update'])}, Delete: {len(changes['to_delete'])}")
        return changes
    
    def _has_changes(self, db_row: Dict, sheet_row: Dict) -> bool:
        """ตรวจสอบว่ามีการเปลี่ยนแปลงหรือไม่"""
        compare_fields = ["title", "category", "topic", "summary"]
        for field in compare_fields:
            db_val = db_row.get(field) or ""
            sheet_val = sheet_row.get(field) or ""
            if str(db_val).strip() != str(sheet_val).strip():
                return True
        return False
    
    def sync_to_mongodb(self, changes: Dict[str, List]) -> SyncResult:
        """
        Apply changes ลง MongoDB
        
        Args:
            changes: Dict from detect_changes()
        
        Returns:
            SyncResult object
        """
        result = SyncResult()
        
        if not self.mongo:
            result.errors.append("MongoDB manager not configured")
            return result
        
        # Create new locations
        for row in changes.get("to_create", []):
            try:
                self.mongo.add_location(row)
                result.created += 1
            except Exception as e:
                result.errors.append(f"Create failed for {row.get('slug')}: {e}")
        
        # Update existing
        for row in changes.get("to_update", []):
            try:
                slug = row.get("slug")
                if slug:
                    self.mongo.update_location(slug, row)
                    result.updated += 1
            except Exception as e:
                result.errors.append(f"Update failed for {row.get('slug')}: {e}")
        
        # Delete removed
        for row in changes.get("to_delete", []):
            try:
                slug = row.get("slug")
                if slug:
                    self.mongo.delete_location_by_slug(slug)
                    result.deleted += 1
            except Exception as e:
                result.errors.append(f"Delete failed for {row.get('slug')}: {e}")
        
        self.last_sync = result.timestamp
        logging.info(f"✅ Sync complete: {result.to_dict()}")
        return result
    
    def full_sync(self) -> SyncResult:
        """
        ทำ full sync (fetch → detect → apply)
        """
        if not self.worksheet:
            result = SyncResult()
            result.errors.append("Not connected to any sheet")
            return result
        
        # Fetch from sheet
        sheet_data = self.fetch_all_rows()
        
        # Fetch from DB
        db_data = self.mongo.get_all_locations() if self.mongo else []
        
        # Detect changes
        changes = self.detect_changes(sheet_data, db_data)
        
        # Apply changes
        return self.sync_to_mongodb(changes)
    
    def get_status(self) -> Dict[str, Any]:
        """ดึงสถานะการเชื่อมต่อ"""
        return {
            "connected": self.spreadsheet is not None,
            "sheet_id": self.sheet_id,
            "sheet_title": self.spreadsheet.title if self.spreadsheet else None,
            "last_sync": self.last_sync
        }


# Singleton instance
_sheets_service: Optional[GoogleSheetsService] = None

def get_sheets_service(mongo_manager=None) -> GoogleSheetsService:
    """Get or create singleton instance"""
    global _sheets_service
    if _sheets_service is None:
        _sheets_service = GoogleSheetsService(mongo_manager)
    elif mongo_manager and not _sheets_service.mongo:
        _sheets_service.mongo = mongo_manager
    return _sheets_service
