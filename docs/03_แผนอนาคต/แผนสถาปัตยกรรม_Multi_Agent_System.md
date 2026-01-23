# แผนการออกแบบสถาปัตยกรรม Multi-Agent ที่ขยายตัวได้ (Scalable Multi-Agent Architecture Design)

เอกสารนี้รร่างแบบแผนจำลองสำหรับการปรับโครงสร้างระบบเพื่อรองรับ **Multi-Agent System** ในอนาคต โดยเน้นความ **Modular** (เป็นส่วนย่อย), **Scalable** (ขยายได้ง่าย), และ **Maintainable** (ดูแลรักษาง่าย)

เป้าหมายคือการเตรียมระบบให้พร้อมสำหรับ **Complex Reasoning** (เช่น การวางแผนเที่ยวแบบ Multi-step) โดยไม่ทำให้ Core Logic ปัจจุบัน (RAG Orchestrator) ยุ่งเหยิง

---

## 1. แนวคิดหลัก (Core Concept)

แทนที่จะยัด Logic การคิดซับซ้อนเข้าไปใน `RAGOrchestrator` โดยตรง เราจะเปลี่ยนให้ Orchestrator ทำหน้าที่เป็นเพียง **Conductor** (วาทยากร) ที่ส่งงานยากๆ ไปให้ **Specialized Agents** (ผู้เชี่ยวชาญเฉพาะทาง)

*   **RAG Orchestrator:** จัดการ Fast response, Routing, และงานทั่วไป (Short-term interactions)
*   **Specialized Agents:** รับงานที่ต้องใช้ Thinking Steps เยอะๆ (Planning, Deep Research, Coding, Booking)

---

## 2. โครงสร้างไฟล์ใหม่ (Proposed Directory Structure)

เราจะสร้าง "Hub" สำหรับ Agents โดยเฉพาะ แยกออกจาก `services` ปกติ เพื่อให้ชัดเจนว่านี่คือ "หน่วยคิด" (Thinkers) ไม่ใช่แค่ "หน่วยทำงาน" (Workers)

```text
Back-end/
├── core/
│   ├── agents/                 # 📂 [NEW] Agent Hub
│   │   ├── __init__.py         # Agent Registry logic
│   │   ├── base_agent.py       # 🧩 Abstract Base Class (แม่แบบมาตรฐาน)
│   │   │
│   │   ├── planner/            # 🤖 Agent 1: Travel Planner
│   │   │   ├── __init__.py
│   │   │   ├── planner_agent.py
│   │   │   ├── prompts/        # 🗣️ Prompts แยกตามภาษา
│   │   │   │   ├── system_th.txt
│   │   │   │   └── system_en.txt
│   │   │   └── tools.py        # 🛠️ Tools เฉพาะของ Planner (เช่น CalendarCalc)
│   │   │
│   │   ├── researcher/         # 🤖 Agent 2: Deep Researcher (Example Future)
│   │   │   ├── __init__.py
│   │   │   └── researcher_agent.py
│   │   │
│   │   └── booking/            # 🤖 Agent 3: Booking Helper (Example Future)
│   │       ├── __init__.py
│   │       └── booking_agent.py
│   │
│   ├── orchestrator/
│   │   └── rag_orchestrator.py # 🧠 Updated to delegate tasks to AgentRegistry
│   └── ...
```

---

## 3. การออกแบบ Base Agent (Standardized Interface)

ทุก Agent ต้องสวม "เครื่องแบบ" เดียวกัน เพื่อให้ Orchestrator เรียกใช้ได้ง่าย (Polymorphism) ไม่ต้องเขียน if-else เยอะแยะในอนาคต

**File:** `Back-end/core/agents/base_agent.py`

```python
from abc import ABC, abstractmethod
from typing import Dict, Any, List

class BaseAgent(ABC):
    def __init__(self, model_service, language: str = "th"):
        self.model_service = model_service
        self.language = language

    @abstractmethod
    async def process(self, query: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Main entry point for the agent.
        Input: Query string + Context (History, User Prefs)
        Output: Standardized Response Dict
        """
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Agent's unique identifier (e.g., 'TRAVEL_PLANNER')"""
        pass
        
    def set_language(self, language: str):
        """Update language for dynamic context switching"""
        self.language = language
        # Logic to reload prompts if needed
```

---

## 4. การจัดการภาษา (Language Handing Strategy)

ปัญหา Multi-Agent คือแต่ละตัวอาจจะ "ลืม" ว่าคุยภาษาอะไรกันอยู่ เราจะแก้ด้วย:

1.  **Global Context Propagation:** `RAGOrchestrator` ต้องส่ง `language_code` ไปให้ Agents ตลอดเวลาที่เริ่มกระบวนการ `process()`
2.  **Multilingual System Prompts:** ในแต่ละ Agent Folder จะมีไฟล์ prompt แยกภาษาชัดเจน เพื่อไม่ให้ Agent สับสน
    *   `agents/planner/prompts/system_th.txt`
    *   `agents/planner/prompts/system_en.txt`
3.  **Output Standardization:** บังคับให้ Agent ตอบกลับเป็นโครงสร้างมาตรฐาน (JSON) ที่มี key `{"text": "...", "meta": {...}}` หรือ format ที่ตกลงกัน เพื่อให้ Frontend นำไปแสดงผลถูกภาษา

---

## 5. การทำงานร่วมกับ Orchestrator (Integration Flow)

`RAGOrchestrator` จะทำหน้าที่ Routing ตามความซับซ้อนของเจตนา (Intent):

1.  **User Input** -> `RAGOrchestrator`
2.  **Intent Check:**
    *   ถ้าเป็น `INFORMATIONAL` (ถามที่เที่ยวทั่วไป) -> ทำ RAG ปกติ (Fast path, Llama/Gemini response)
    *   ถ้าเป็น `COMPLEX_TASK` (เช่น "วางแผนเที่ยว 3 วัน") -> **Delegate** ไปที่ `AgentRegistry`
3.  **Agent Execution:**
    *   `agent = AgentRegistry.get("TRAVEL_PLANNER")`
    *   `agent.set_language(current_lang)`
    *   `response = await agent.process(query, context)`
    *   Agent คิด 2-3 step (อาจใช้ Tool เรียก RAG อีกทีเพื่อหาข้อมูลสถานที่จริงมาประกอบแผน)
4.  **Response:** Agent ส่งแผนกลับมา -> Orchestrator ส่งต่อให้ User

---

## 6. สรุปประโยชน์ของโครงสร้างนี้

1.  **Scalability:** เพิ่ม Agent ได้ไม่จำกัด (Agent 4, 5, 6...) โดยไม่ต้องแก้ code หลักของ Orchestrator
2.  **Isolation:** บั๊กใน Planner ไม่ทำให้ระบบ Chat หลักพัง
3.  **Specialization:** แต่ละ Agent ใช้โมเดลที่ต่างกันได้ (เช่น Planner ใช้ Gemini Pro, Chat ใช้ Llama 70B Fast)
4.  **Clean Code:** โฟลเดอร์เป็นระเบียบ ง่ายต่อการให้ Junior Dev เข้ามาช่วยเขียน Agent ย่อย

---
*บันทึกเมื่อ: 2026-01-23*
