# /core/services/import_service.py
"""
ImportService: อ่านไฟล์ Excel/CSV และแปลงเป็น List of Dictionary
สำหรับ AI-Powered Smart ETL System
"""

import io
import pandas as pd
from typing import List, Dict, Any, Optional


class ImportService:
    """
    Service สำหรับ parse ไฟล์ Excel/CSV เป็น structured data
    """
    
    SUPPORTED_EXCEL_EXTENSIONS = ['.xlsx', '.xls']
    SUPPORTED_CSV_EXTENSIONS = ['.csv']
    
    def parse_excel(self, file_content: bytes, sheet_name: Optional[str] = None) -> List[Dict[str, Any]]:
        """
        อ่านไฟล์ Excel จาก Memory โดยไม่ต้อง Save ลง Disk
        
        Args:
            file_content: Binary content ของไฟล์ Excel
            sheet_name: ชื่อ Sheet ที่ต้องการอ่าน (default: sheet แรก)
            
        Returns:
            List of Dictionary โดยแต่ละ dict คือ 1 row
        """
        try:
            # อ่าน Excel จาก BytesIO
            df = pd.read_excel(
                io.BytesIO(file_content),
                sheet_name=sheet_name if sheet_name else 0,  # 0 = sheet แรก
                engine='openpyxl'
            )
            
            # Clean data: แปลง NaN เป็น empty string
            df = df.fillna("")
            
            # แปลง column names เป็น string ทั้งหมด (กันกรณี column เป็นตัวเลข)
            df.columns = df.columns.astype(str)
            
            # แปลงเป็น List of Dict
            records = df.to_dict(orient='records')
            
            print(f"✅ [ImportService] อ่าน Excel สำเร็จ: {len(records)} แถว, {len(df.columns)} คอลัมน์")
            return records
            
        except Exception as e:
            print(f"❌ [ImportService] เกิดข้อผิดพลาดในการอ่าน Excel: {e}")
            raise ValueError(f"ไม่สามารถอ่านไฟล์ Excel ได้: {str(e)}")
    
    def parse_csv(self, file_content: bytes, encoding: str = 'utf-8') -> List[Dict[str, Any]]:
        """
        อ่านไฟล์ CSV จาก Memory
        
        Args:
            file_content: Binary content ของไฟล์ CSV
            encoding: Encoding ของไฟล์ (default: utf-8)
            
        Returns:
            List of Dictionary โดยแต่ละ dict คือ 1 row
        """
        try:
            # Try UTF-8 first, fallback to TIS-620 (Thai encoding)
            try:
                df = pd.read_csv(io.BytesIO(file_content), encoding=encoding)
            except UnicodeDecodeError:
                print("⚠️ [ImportService] UTF-8 ล้มเหลว, กำลังลอง TIS-620...")
                df = pd.read_csv(io.BytesIO(file_content), encoding='tis-620')
            
            # Clean data
            df = df.fillna("")
            df.columns = df.columns.astype(str)
            
            records = df.to_dict(orient='records')
            
            print(f"✅ [ImportService] อ่าน CSV สำเร็จ: {len(records)} แถว, {len(df.columns)} คอลัมน์")
            return records
            
        except Exception as e:
            print(f"❌ [ImportService] เกิดข้อผิดพลาดในการอ่าน CSV: {e}")
            raise ValueError(f"ไม่สามารถอ่านไฟล์ CSV ได้: {str(e)}")
    
    def parse_file(self, file_content: bytes, filename: str) -> List[Dict[str, Any]]:
        """
        Auto-detect file type และ parse ตาม extension
        
        Args:
            file_content: Binary content ของไฟล์
            filename: ชื่อไฟล์ (ใช้ดู extension)
            
        Returns:
            List of Dictionary
        """
        filename_lower = filename.lower()
        
        if any(filename_lower.endswith(ext) for ext in self.SUPPORTED_EXCEL_EXTENSIONS):
            return self.parse_excel(file_content)
        elif any(filename_lower.endswith(ext) for ext in self.SUPPORTED_CSV_EXTENSIONS):
            return self.parse_csv(file_content)
        else:
            raise ValueError(f"ไม่รองรับไฟล์ประเภทนี้: {filename}. รองรับเฉพาะ Excel (.xlsx, .xls) และ CSV (.csv)")
    
    def detect_columns(self, data: List[Dict[str, Any]]) -> List[str]:
        """
        ดึงรายชื่อ columns ทั้งหมดจากข้อมูล
        
        Args:
            data: List of Dictionary จากการ parse
            
        Returns:
            List of column names
        """
        if not data:
            return []
        
        # รวม keys จากทุก row (กรณี row แรกอาจไม่มี key ครบ)
        all_columns = set()
        for row in data:
            all_columns.update(row.keys())
        
        return sorted(list(all_columns))
    
    def get_preview(self, data: List[Dict[str, Any]], max_rows: int = 10) -> Dict[str, Any]:
        """
        สร้าง preview summary สำหรับแสดงผลใน UI
        
        Args:
            data: List of Dictionary
            max_rows: จำนวน rows สูงสุดที่จะแสดง
            
        Returns:
            Dict containing columns, preview_rows, total_rows
        """
        columns = self.detect_columns(data)
        preview_rows = data[:max_rows]
        
        return {
            "columns": columns,
            "preview_rows": preview_rows,
            "total_rows": len(data),
            "showing_rows": len(preview_rows)
        }


# Singleton instance
import_service = ImportService()
