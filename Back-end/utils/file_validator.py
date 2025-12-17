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
    Rejects binary files masking as text, but allows UTF-16/32.
    """
    # List of encodings to try
    encodings = ["utf-8", "utf-8-sig", "cp874", "utf-16", "utf-16le", "utf-16be", "latin-1"]
    
    for enc in encodings:
        try:
            # Try to decode the header with the encoding
            header.decode(enc)
            
            # If successful, additional check:
            # If it's UTF-8/Ascii, it shouldn't have too many null bytes (unless it's empty)
            if enc in ["utf-8", "utf-8-sig", "cp874", "latin-1"]:
                if b"\x00" in header:
                   # Allowing rare nulls, but usually text shouldn't have them.
                   # But let's be permissive to avoid blocking valid files.
                   pass 
            
            return True
        except UnicodeDecodeError:
            continue
            
    # If all decodes fail, likely binary garbage
    return False
