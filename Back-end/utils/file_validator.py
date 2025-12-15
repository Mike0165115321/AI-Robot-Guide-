import logging
from fastapi import UploadFile

# Magic Numbers (Signatures)
SIGNATURES = {
    "pdf": b"%PDF",
    "xlsx": b"PK\x03\x04", # ZIP signature (XLSX is a zipped XML)
    "xls": b"\xD0\xCF\x11\xE0\xA1\xB1\x1A\xE1", # Legacy Excel
}

async def verify_file_signature(file: UploadFile) -> bool:
    """
    Verifies the file signature (magic number) matches its extension.
    Also handles cursor management (seek 0) to prevent data loss.
    """
    try:
        # 1. Remember original position (usually 0)
        await file.seek(0)
        
        # 2. Read Header (2048 bytes is enough for most signatures)
        header = await file.read(2048)
        
        # 3. Reset Cursor immediately (Critical!)
        await file.seek(0)
        
        filename = file.filename.lower() if file.filename else ""
        
        # Check by extension
        if filename.endswith(".xlsx"):
            return header.startswith(SIGNATURES["xlsx"])
            
        elif filename.endswith(".pdf"):
            return header.startswith(SIGNATURES["pdf"])
        
        elif filename.endswith(".xls"):
            return header.startswith(SIGNATURES["xls"])
            
        elif filename.endswith(".csv") or filename.endswith(".txt"):
            # CSV has no magic number: use heuristic check
            return is_safe_text_file(header)
            
        # Unknown extension or not in our allowlist
        return False
        
    except Exception as e:
        logging.error(f"Error validating file signature: {e}")
        return False

def is_safe_text_file(header: bytes) -> bool:
    """
    Heuristic check for safe text files (CSV/TXT).
    Rejects binary files masking as text.
    """
    # Check 1: Must not contain Null Byte (Binary files usually have \x00)
    if b"\x00" in header:
        return False
        
    # Check 2: Must be valid UTF-8
    try:
        header.decode("utf-8")
        return True
    except UnicodeDecodeError:
        # If not UTF-8, it might be other encoding, but for our system we prefer UTF-8.
        # However, some legacy Thai CSVs might be TIS-620. 
        # For security, we stick to UTF-8 or ensure no binary control chars.
        # Let's try TIS-620 just in case, but usually binary files fail both.
        try:
            header.decode("cp874") # Thai encoding
            return True
        except:
            return False
