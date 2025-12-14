# /core/services/pdf_reader_service.py
"""
PDF Reader Service: อ่านข้อความจากไฟล์ PDF
ใช้ PyMuPDF (fitz) สำหรับ extract text
"""

import fitz  # PyMuPDF
import logging
from typing import Optional


class PDFReaderService:
    """
    Service สำหรับอ่าน PDF และ extract text
    รองรับ PDF หลายหน้า
    """
    
    def extract_text(self, pdf_bytes: bytes) -> str:
        """
        Extract text จาก PDF bytes
        
        Args:
            pdf_bytes: Content ของไฟล์ PDF เป็น bytes
            
        Returns:
            Text ทั้งหมดจากทุกหน้าของ PDF
        """
        try:
            # เปิด PDF จาก bytes
            doc = fitz.open(stream=pdf_bytes, filetype="pdf")
            
            text_parts = []
            for page_num in range(len(doc)):
                page = doc.load_page(page_num)
                page_text = page.get_text("text")
                if page_text.strip():
                    text_parts.append(f"--- หน้า {page_num + 1} ---\n{page_text}")
            
            doc.close()
            
            full_text = "\n\n".join(text_parts)
            logging.info(f"📄 [PDFReader] ดึงข้อความได้ {len(full_text)} ตัวอักษร จาก {len(text_parts)} หน้า")
            
            return full_text
            
        except Exception as e:
            logging.error(f"❌ [PDFReader] เกิดข้อผิดพลาดในการดึงข้อความ: {e}")
            raise ValueError(f"ไม่สามารถอ่าน PDF ได้: {str(e)}")
    
    def get_page_count(self, pdf_bytes: bytes) -> int:
        """นับจำนวนหน้าใน PDF"""
        try:
            doc = fitz.open(stream=pdf_bytes, filetype="pdf")
            count = len(doc)
            doc.close()
            return count
        except Exception:
            return 0


# Singleton instance
pdf_reader_service = PDFReaderService()
