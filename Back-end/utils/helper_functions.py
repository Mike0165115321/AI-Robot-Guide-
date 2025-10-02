# /Back-end/utils/helper_functions.py

def create_synthetic_document(data_item: dict) -> str:
    """
    สร้างเอกสารสังเคราะห์จากโครงสร้างข้อมูลองค์ความรู้ (Knowledge Model)
    เพื่อใช้ทั้งในการ Re-ranking และการสร้าง Context สำหรับ LLM
    """
    if not isinstance(data_item, dict):
        return ""

    title = data_item.get("title", "")
    topic = data_item.get("topic", "")
    summary = data_item.get("summary", "")
    
    details_list = []
    for detail_part in data_item.get("details", []):
        heading = detail_part.get("heading", "")
        content = detail_part.get("content", "")
        details_list.append(f"{heading}: {content}")
    details_text = "\n".join(details_list)

    keywords_text = ", ".join(data_item.get("keywords", []))

    full_text = (
        f"หัวข้อ: {title} (หมวดหมู่ย่อย: {topic})\n"
        f"สรุป: {summary}\n\n"
        f"รายละเอียด:\n{details_text}\n\n"
        f"คำสำคัญ: {keywords_text}"
    )
    
    return full_text.strip()