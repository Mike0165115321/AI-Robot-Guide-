# Qdrant Manager - คำอธิบายการทำงาน

ไฟล์ `qdrant_manager.py` เป็น **Manager Class สำหรับจัดการ Vector Database (Qdrant)** ใช้ในระบบ RAG (Retrieval-Augmented Generation)

## หน้าที่หลัก

### 1. การเริ่มต้นระบบ (`__init__` & `initialize`)
- เชื่อมต่อกับ Qdrant Server ผ่าน `AsyncQdrantClient`
- โหลด **Embedding Model** (`intfloat/multilingual-e5-large`) สำหรับแปลงข้อความเป็น Vector
- ตรวจสอบ/สร้าง Collection ใน Qdrant ถ้ายังไม่มี

### 2. การสร้าง Vector (`_create_vector`)
- แปลงข้อความเป็น Embedding Vector โดยใช้ `SentenceTransformer`
- ทำงานแบบ Async โดยใช้ `asyncio.to_thread`

### 3. การเพิ่มข้อมูล (`upsert_location`)
- รับ `mongo_id` และ `description` จาก MongoDB
- เติม prefix `"passage: "` ตาม format ของ E5 model
- สร้าง Vector และบันทึกลง Qdrant พร้อม payload (`mongo_id`, `text_content`)

### 4. การค้นหา (`search_similar`)
- รับ query text แล้วเติม prefix `"query: "`
- ค้นหา Vector ที่ใกล้เคียงที่สุดโดยใช้ **Cosine Similarity**
- คืนค่าผลลัพธ์พร้อม score และ payload

### 5. การลบข้อมูล (`delete_vector`)
- ลบ Vector ออกจาก Qdrant โดยใช้ `mongo_id` เป็น key

## สรุป
เป็น Layer กลางระหว่าง Application กับ Qdrant สำหรับจัดการ Semantic Search ใช้คู่กับ MongoDB (เก็บข้อมูลหลัก) และ LLM (ใช้ผลค้นหาเป็น context)
