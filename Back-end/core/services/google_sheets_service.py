"""
Google Sheets Sync Service
รองรับการ sync ข้อมูลจาก Google Sheets เข้าสู่ MongoDB
รองรับ 3 โหมด: Public CSV, Service Account, OAuth2
"""

import os
import io
import csv
import json
import logging
import requests
import threading
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
    รองรับ 3 โหมด:
    - public: ดึงข้อมูลผ่าน CSV export (ไม่ต้อง credentials)
    - service_account: ใช้ Service Account credentials
    - oauth2: ใช้ User OAuth2 token
    """
    
    def __init__(self, mongo_manager=None):
        self.client: Optional[gspread.Client] = None
        self.spreadsheet: Optional[Spreadsheet] = None
        self.worksheet: Optional[Worksheet] = None
        self.mongo = mongo_manager
        self.sheet_id: Optional[str] = None
        self.sheet_title: Optional[str] = None  # For public mode where we can't get title
        self.last_sync: Optional[str] = None
        self.connection_mode: Optional[str] = None  # "public", "service_account", "oauth2"
        
        # [PRODUCTION] Sync lock to prevent concurrent syncs
        self._sync_lock = threading.Lock()
        self._is_syncing = False
        
        # Required columns mapping (Sheet column → DB field)
        self.column_mapping = {
            "slug": "slug",
            "title": "title", 
            "category": "category",
            "topic": "topic",
            "summary": "summary",
            "keywords": "keywords",  # comma-separated in sheet
        }
    
    def _extract_sheet_id(self, sheet_url: str) -> Optional[str]:
        """ดึง sheet_id จาก URL"""
        if not sheet_url:
            return None
        # URL format: https://docs.google.com/spreadsheets/d/SHEET_ID/edit
        parts = sheet_url.split("/d/")
        if len(parts) > 1:
            return parts[1].split("/")[0]
        return None

    def _extract_gid(self, sheet_url: str) -> str:
        """ดึง gid (sheet tab id) จาก URL, default เป็น 0"""
        if not sheet_url or "gid=" not in sheet_url:
            return "0"
        try:
            return sheet_url.split("gid=")[1].split("&")[0].split("#")[0]
        except:
            return "0"

    def connect_public(self, sheet_url: str) -> bool:
        """
        เชื่อมต่อ Google Sheet แบบ Public (ไม่ต้อง credentials)
        
        Sheet ต้องถูก share เป็น "Anyone with the link" ก่อน
        
        Args:
            sheet_url: URL เต็มของ Google Sheet
        
        Returns:
            True ถ้าเชื่อมต่อสำเร็จ (sheet เป็น public และอ่านได้)
        """
        try:
            sheet_id = self._extract_sheet_id(sheet_url)
            if not sheet_id:
                logging.error("❌ Google Sheets URL ไม่ถูกต้อง")
                return False
            
            gid = self._extract_gid(sheet_url)
            
            # Try fetching CSV to verify the sheet is public
            csv_url = f"https://docs.google.com/spreadsheets/d/{sheet_id}/export?format=csv&gid={gid}"
            
            response = requests.get(csv_url, timeout=30.0, allow_redirects=True)
            
            if response.status_code == 200:
                # Verify it's actually CSV data (not an HTML error page)
                content_type = response.headers.get("content-type", "")
                if "text/html" in content_type:
                    logging.error("❌ Sheet ไม่เป็นสาธารณะหรือไม่มีอยู่จริง")
                    return False
                
                self.sheet_id = sheet_id
                self.sheet_title = f"Public Sheet ({sheet_id[:8]}...)"
                self.connection_mode = "public"
                self._public_gid = gid
                
                logging.info(f"✅ เชื่อมต่อกับ public sheet สำเร็จ: {sheet_id}")
                return True
            else:
                logging.error(f"❌ ไม่สามารถเข้าถึง sheet: HTTP {response.status_code}")
                return False
                    
        except Exception as e:
            logging.error(f"❌ การเชื่อมต่อกับ public sheet ล้มเหลว: {e}")
            return False

    def fetch_public_csv(self) -> List[Dict[str, Any]]:
        """
        ดึงข้อมูลจาก Public Google Sheet ผ่าน CSV export
        
        Returns:
            List of dict (แต่ละ row เป็น dict)
        """
        if self.connection_mode != "public" or not self.sheet_id:
            logging.error("❌ ไม่ได้เชื่อมต่อในโหมด public")
            return []
        
        try:
            gid = getattr(self, '_public_gid', '0')
            csv_url = f"https://docs.google.com/spreadsheets/d/{self.sheet_id}/export?format=csv&gid={gid}"
            
            response = requests.get(csv_url, timeout=30.0, allow_redirects=True)
            
            if response.status_code != 200:
                logging.error(f"❌ ดึง CSV ล้มเหลว: HTTP {response.status_code}")
                return []
            
            # Parse CSV with explicit UTF-8 encoding
            # response.content is bytes, decode with UTF-8
            csv_content = response.content.decode('utf-8')
            reader = csv.DictReader(io.StringIO(csv_content))
            records = list(reader)
            
            logging.info(f"📊 ดึงข้อมูลได้ {len(records)} แถวจาก public sheet")
            return records
                
        except Exception as e:
            logging.error(f"❌ ดึง public CSV ล้มเหลว: {e}")
            return []

    def connect(self, sheet_id: str = None, sheet_url: str = None) -> bool:
        """
        เชื่อมต่อ Google Sheet (Service Account mode)
        
        ถ้าไม่มี credentials จะลองใช้ public mode แทน
        
        Args:
            sheet_id: ID ของ sheet (ส่วนยาวๆ ใน URL)
            sheet_url: URL เต็มของ sheet
        
        Returns:
            True ถ้าเชื่อมต่อสำเร็จ
        """
        try:
            # Check if credentials exist
            if not CREDENTIALS_PATH.exists():
                logging.warning(f"⚠️ ไม่พบ Credentials กำลังลองใช้โหมด public...")
                if sheet_url:
                    return self.connect_public(sheet_url)
                else:
                    logging.error("❌ ไม่มี credentials และไม่มี URL สำหรับโหมด public")
                    return False
            
            # Initialize client with Service Account
            if not self.client:
                self.client = gspread.service_account(filename=str(CREDENTIALS_PATH))
                logging.info("✅ Google Sheets client เริ่มต้นแล้ว (Service Account)")
            
            # Extract sheet_id from URL if needed
            if sheet_url and not sheet_id:
                sheet_id = self._extract_sheet_id(sheet_url)
            
            if not sheet_id:
                logging.error("❌ ไม่ได้ระบุ sheet_id หรือ sheet_url")
                return False
            
            # Open spreadsheet
            self.spreadsheet = self.client.open_by_key(sheet_id)
            self.worksheet = self.spreadsheet.sheet1  # Use first sheet
            self.sheet_id = sheet_id
            self.sheet_title = self.spreadsheet.title
            self.connection_mode = "service_account"
            
            logging.info(f"✅ เชื่อมต่อกับ sheet: {self.spreadsheet.title}")
            return True
            
        except gspread.exceptions.SpreadsheetNotFound:
            logging.error(f"❌ ไม่พบ Sheet หรือไม่ได้แชร์กับ service account")
            # Try public mode as fallback
            if sheet_url:
                logging.info("🔄 กำลังลองโหมด public เพื่อสำรองข้อมูล...")
                return self.connect_public(sheet_url)
            return False
        except Exception as e:
            logging.error(f"❌ เชื่อมต่อกับ Google Sheet ล้มเหลว: {e}")
            return False
    
    def fetch_all_rows(self) -> List[Dict[str, Any]]:
        """
        ดึงข้อมูลทั้งหมดจาก Sheet (รองรับทั้ง public และ service_account mode)
        
        Returns:
            List of dict (แต่ละ row เป็น dict)
        """
        # Use public CSV fetch if in public mode
        if self.connection_mode == "public":
            return self.fetch_public_csv()
        
        # Service Account mode
        if not self.worksheet:
            logging.error("❌ ไม่ได้เชื่อมต่อกับ sheet ใดๆ")
            return []
        
        try:
            # Get all records (assumes first row is header)
            records = self.worksheet.get_all_records()
            logging.info(f"📊 ดึงข้อมูลได้ {len(records)} แถวจาก sheet")
            return records
        except Exception as e:
            logging.error(f"❌ ดึงข้อมูลแถวไม่สำเร็จ: {e}")
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
        
        [PRODUCTION POLICY] ลบข้อมูลไม่ทำใน Sync - ถ้าต้องการลบต้องลบจาก Admin Panel เท่านั้น
        เพื่อป้องกันการสูญหายของข้อมูลจากการ sync ที่ผิดพลาด
        
        Returns:
            Dict with keys: to_create, to_update (no to_delete)
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
            # [REMOVED] to_delete - Sync จะไม่ลบข้อมูลอีกต่อไป
        }
        
        # Find new and updated
        for slug, sheet_row in sheet_by_slug.items():
            if slug not in db_by_slug:
                # New row - add metadata for tracking
                sheet_row["metadata"] = {
                    "synced_from": "google_sheets",
                    "sheet_id": self.sheet_id,
                    "synced_at": datetime.now().isoformat()
                }
                changes["to_create"].append(sheet_row)
            else:
                # Check if updated (compare key fields)
                db_row = db_by_slug[slug]
                if self._has_changes(db_row, sheet_row):
                    sheet_row["_id"] = db_row.get("_id")
                    # Update sync metadata
                    sheet_row["metadata"] = db_row.get("metadata", {})
                    sheet_row["metadata"]["last_synced_at"] = datetime.now().isoformat()
                    changes["to_update"].append(sheet_row)
        
        # [REMOVED] Delete logic - Sync will NEVER delete data
        # If user wants to delete, they must do it manually from Admin Panel
        
        logging.info(f"📊 ตรวจพบการเปลี่ยนแปลง - สร้างใหม่: {len(changes['to_create'])}, อัปเดต: {len(changes['to_update'])} (ไม่มีการลบ)")
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
                    # [FIX] Use update_location_by_slug instead of update_location
                    # update_location expects ObjectId, but we have slug from the sheet
                    self.mongo.update_location_by_slug(slug, row)
                    result.updated += 1
            except Exception as e:
                result.errors.append(f"Update failed for {row.get('slug')}: {e}")
        
        # [PRODUCTION] Delete logic REMOVED
        # Sync will never delete data - users must delete manually from Admin Panel
        
        self.last_sync = result.timestamp
        logging.info(f"✅ การซิงค์เสร็จสมบูรณ์: สร้าง {result.created}, อัปเดต {result.updated} (ไม่มีการลบ)")
        return result
    
    def full_sync(self) -> SyncResult:
        """
        ทำ full sync (fetch → detect → apply)
        รองรับทั้ง public และ service_account mode
        
        [PRODUCTION] มี sync lock ป้องกันการ sync ซ้อนกัน
        """
        # [PRODUCTION] Check if sync already in progress
        if self._is_syncing:
            result = SyncResult()
            result.errors.append("กำลังซิงค์อยู่แล้ว กรุณารอสักครู่")
            return result
        
        # Acquire lock
        with self._sync_lock:
            self._is_syncing = True
            try:
                # Check if connected (either mode)
                if not self.sheet_id:
                    result = SyncResult()
                    result.errors.append("ยังไม่ได้เชื่อมต่อ Sheet")
                    return result
                
                # For service_account mode, also check worksheet
                if self.connection_mode == "service_account" and not self.worksheet:
                    result = SyncResult()
                    result.errors.append("ยังไม่ได้เชื่อมต่อ Sheet")
                    return result
                
                logging.info("🔄 เริ่มการซิงค์ข้อมูล...")
                
                # Fetch from sheet
                sheet_data = self.fetch_all_rows()
                
                # Fetch from DB
                db_data = self.mongo.get_all_locations() if self.mongo else []
                
                # Detect changes
                changes = self.detect_changes(sheet_data, db_data)
                
                # Apply changes
                return self.sync_to_mongodb(changes)
            finally:
                self._is_syncing = False
    
    def get_status(self) -> Dict[str, Any]:
        """ดึงสถานะการเชื่อมต่อ"""
        is_connected = self.sheet_id is not None
        return {
            "connected": is_connected,
            "sheet_id": self.sheet_id,
            "sheet_title": self.sheet_title,
            "last_sync": self.last_sync,
            "mode": self.connection_mode
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
