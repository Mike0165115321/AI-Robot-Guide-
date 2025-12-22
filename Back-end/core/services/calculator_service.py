"""
Safe Calculator Service - Hybrid Mode

A smart calculator that uses:
1. Direct Python calculation for pure math expressions (fast)
2. AI-assisted calculation for natural language math questions (understanding + context)

This ensures accuracy (Python) and natural language understanding (AI 70B).
"""

import re
import math
import logging
from typing import Optional, Tuple

from core.ai_models.groq_handler import get_groq_response
from core.config import settings


class CalculatorService:
    """
    Hybrid calculator service:
    - Pure math → Direct Python (fast, accurate)
    - Text + math → AI 70B analyzes, calls Python, formats response
    """

    # Allowed math functions and constants for safe_eval
    SAFE_NAMES = {
        'abs': abs,
        'round': round,
        'min': min,
        'max': max,
        'sum': sum,
        'pow': pow,
        'sqrt': math.sqrt,
        'sin': math.sin,
        'cos': math.cos,
        'tan': math.tan,
        'log': math.log,
        'log10': math.log10,
        'pi': math.pi,
        'e': math.e,
    }

    # Pattern: Pure math (numbers and operators only)
    PURE_MATH_PATTERN = r'^[\d\s\+\-\*\/\.\(\)\%\,]+$'
    
    # Pattern: Thai math with simple operators
    SIMPLE_THAI_MATH = r'^\s*\d+\s*(บวก|ลบ|คูณ|หาร|%\s*ของ)\s*\d+\s*$'

    # Patterns to detect ANY calculator query (broad)
    CALC_PATTERNS = [
        r'คำนวณ',
        r'คิดเลข',
        r'เท่าไหร่',
        r'เท่าไร',
        r'เป็นเท่าไหร่',
        r'หาร',
        r'คูณ',
        r'บวก',
        r'ลบ',
        r'เปอร์เซ็นต์',
        r'%\s*ของ',
        r'\d+\s*[\+\-\*\/\%]\s*\d+',
        r'\d+\s*\*\*\s*\d+',
        r'vat',
        r'ภาษี',
        r'ดอกเบี้ย',
        r'ส่วนลด',
        r'กี่บาท',
        r'กี่เปอร์เซ็นต์',
    ]

    @classmethod
    def is_calculator_query(cls, text: str) -> bool:
        """Check if the text is a calculator query."""
        text = text.lower().strip()
        for pattern in cls.CALC_PATTERNS:
            if re.search(pattern, text, re.IGNORECASE):
                return True
        return False

    @classmethod
    def is_pure_math(cls, text: str) -> bool:
        """
        Check if the text is PURE math (no Thai text, just numbers and operators).
        Examples: "15+20", "100*0.07", "1000/4"
        """
        text = text.strip()
        # Remove common prefixes
        text = re.sub(r'^(คำนวณ|คิดเลข|หา)\s*', '', text)
        
        # Check if it's pure numbers and operators
        if re.match(cls.PURE_MATH_PATTERN, text.replace('**', '')):
            return True
        
        # Check for simple Thai math like "100 บวก 50"
        if re.match(cls.SIMPLE_THAI_MATH, text, re.IGNORECASE):
            return True
            
        return False

    @classmethod
    def parse_thai_math(cls, text: str) -> str:
        """Convert Thai math expressions to Python math expressions."""
        expr = text.lower().strip()
        
        # Remove common prefixes
        expr = re.sub(r'^(คำนวณ|คิดเลข|หา)\s*', '', expr)
        
        # Handle percentage: "15% ของ 850" or "15 เปอร์เซ็นต์ ของ 850"
        expr = re.sub(r'(\d+(?:\.\d+)?)\s*(%|เปอร์เซ็นต์)\s*ของ\s*(\d+(?:\.\d+)?)', 
                      r'(\1/100)*\3', expr)
        
        # Handle percentage simple: "15% of 850" style
        expr = re.sub(r'(\d+(?:\.\d+)?)\s*%\s*\*?\s*(\d+(?:\.\d+)?)', 
                      r'(\1/100)*\2', expr)
        
        # Thai operators to symbols
        replacements = [
            (r'\s*บวก\s*', '+'),
            (r'\s*ลบ\s*', '-'),
            (r'\s*คูณ\s*', '*'),
            (r'\s*หาร\s*', '/'),
            (r'\s*ยกกำลัง\s*', '**'),
            (r'\s*กำลังสอง', '**2'),
            (r'\s*รากที่สอง\s*ของ\s*', 'sqrt('),
            (r'เท่าไหร่|เท่าไร|=|\?', ''),
        ]
        
        for pattern, replacement in replacements:
            expr = re.sub(pattern, replacement, expr)
        
        # Clean up extra spaces
        expr = re.sub(r'\s+', '', expr)
        
        # Add closing paren for sqrt if needed
        if 'sqrt(' in expr and ')' not in expr:
            expr += ')'
        
        return expr

    @classmethod
    def safe_eval(cls, expression: str) -> Tuple[Optional[float], Optional[str]]:
        """Safely evaluate a math expression."""
        try:
            # Only allow safe characters
            expr_clean = expression
            for name in cls.SAFE_NAMES.keys():
                expr_clean = expr_clean.replace(name, '')
            
            if not re.match(r'^[\d\s\+\-\*\/\.\(\)\%\,]+$', expr_clean.replace('**', '')):
                return None, "พบอักขระที่ไม่อนุญาต"
            
            # Evaluate with restricted globals
            result = eval(expression, {"__builtins__": {}}, cls.SAFE_NAMES)
            
            # Format result
            if isinstance(result, float):
                if result == int(result):
                    result = int(result)
                else:
                    result = round(result, 6)
            
            return result, None
            
        except ZeroDivisionError:
            return None, "ไม่สามารถหารด้วย 0 ได้"
        except SyntaxError:
            return None, "รูปแบบคณิตศาสตร์ไม่ถูกต้อง"
        except Exception as e:
            return None, f"เกิดข้อผิดพลาด: {str(e)}"

    @classmethod
    def calculate_direct(cls, query: str) -> dict:
        """Direct Python calculation - for pure math expressions."""
        expression = cls.parse_thai_math(query)
        
        if not expression or expression.strip() == '':
            return cls._help_message()
        
        result, error = cls.safe_eval(expression)
        
        if error:
            return {
                "answer": f"❌ ไม่สามารถคำนวณได้: {error}",
                "action": None,
                "sources": [],
                "image_url": None,
                "image_gallery": []
            }
        
        # Format nicely
        if isinstance(result, (int, float)):
            formatted = f"{result:,}" if isinstance(result, int) else f"{result:,.6f}".rstrip('0').rstrip('.')
        else:
            formatted = str(result)
        
        return {
            "answer": f"🧮 **คำตอบ**\n\n`{query.strip()}` = **{formatted}**",
            "action": None,
            "sources": [],
            "image_url": None,
            "image_gallery": []
        }

    @classmethod
    async def calculate_with_ai(cls, query: str) -> dict:
        """
        AI-assisted calculation - for natural language math questions.
        AI 70B analyzes the question, extracts the math, runs Python, and formats response.
        """
        logging.info(f"🧮 [Calculator] AI-assisted mode for: '{query}'")
        
        # System prompt for math extraction
        system_prompt = """คุณเป็นผู้ช่วยคำนวณ ทำตามขั้นตอนนี้:

1. วิเคราะห์คำถามและหานิพจน์คณิตศาสตร์
2. ตอบในรูปแบบ JSON ดังนี้:

หากเป็นโจทย์คณิตศาสตร์:
{"is_math": true, "expression": "1000*0.07", "explanation": "VAT 7% ของ 1,000 บาท"}

หากไม่ใช่โจทย์คณิตศาสตร์:
{"is_math": false, "reason": "ไม่พบโจทย์คณิตศาสตร์"}

ตอบเป็น JSON เท่านั้น ไม่ต้องอธิบายเพิ่ม"""

        try:
            # Call AI 70B to extract math
            response = await get_groq_response(
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": query}
                ],
                model_name=settings.GROQ_LLAMA_MODEL  # 70B
            )
            
            # Parse JSON response
            import json
            try:
                # Clean up response if needed
                response_clean = response.strip()
                if response_clean.startswith("```"):
                    response_clean = re.sub(r'^```json?\n?', '', response_clean)
                    response_clean = re.sub(r'\n?```$', '', response_clean)
                
                parsed = json.loads(response_clean)
            except json.JSONDecodeError:
                logging.warning(f"🧮 [Calculator] AI response not JSON: {response}")
                # Fallback to direct calculation
                return cls.calculate_direct(query)
            
            if not parsed.get("is_math", False):
                return {
                    "answer": f"🤔 ไม่พบโจทย์คณิตศาสตร์ในคำถามนี้ค่ะ\n\nลองพิมพ์เช่น:\n- `100 บวก 50`\n- `15% ของ 850`\n- `VAT 7% ของ 1000 บาท เท่าไหร่`",
                    "action": None,
                    "sources": [],
                    "image_url": None,
                    "image_gallery": []
                }
            
            # Extract expression and calculate
            expression = parsed.get("expression", "")
            explanation = parsed.get("explanation", "")
            
            if not expression:
                return cls.calculate_direct(query)
            
            # Calculate with Python
            result, error = cls.safe_eval(expression)
            
            if error:
                return {
                    "answer": f"❌ ไม่สามารถคำนวณได้: {error}",
                    "action": None,
                    "sources": [],
                    "image_url": None,
                    "image_gallery": []
                }
            
            # Format result
            if isinstance(result, (int, float)):
                formatted = f"{result:,}" if isinstance(result, int) else f"{result:,.6f}".rstrip('0').rstrip('.')
            else:
                formatted = str(result)
            
            # Build rich answer
            answer = f"🧮 **{explanation}**\n\n"
            answer += f"**คำตอบ:** {formatted}"
            
            # Add context for common scenarios
            if "vat" in query.lower() or "ภาษี" in query.lower():
                if "7" in expression:
                    # Try to find the base amount
                    match = re.search(r'(\d+(?:\.\d+)?)\s*\*', expression)
                    if match:
                        base = float(match.group(1))
                        answer += f"\n\n💰 **รวมทั้งหมด:** {base + result:,.2f} บาท"
            
            return {
                "answer": answer,
                "action": None,
                "sources": [],
                "image_url": None,
                "image_gallery": []
            }
            
        except Exception as e:
            logging.error(f"🧮 [Calculator] AI error: {e}")
            # Fallback to direct calculation
            return cls.calculate_direct(query)

    @classmethod
    async def calculate(cls, query: str) -> dict:
        """
        Main entry point - Hybrid Mode:
        - Pure math → Direct Python (fast)
        - Text + math → AI 70B assisted (understanding)
        """
        if cls.is_pure_math(query):
            logging.info(f"🧮 [Calculator] Pure math detected, using direct Python")
            return cls.calculate_direct(query)
        else:
            logging.info(f"🧮 [Calculator] Natural language math, using AI 70B")
            return await cls.calculate_with_ai(query)

    @classmethod
    def _help_message(cls) -> dict:
        return {
            "answer": "🧮 **เครื่องคิดเลข**\n\nพิมพ์โจทย์คณิตศาสตร์ได้เลยค่ะ เช่น:\n- `15% ของ 850`\n- `100 บวก 50`\n- `VAT 7% ของ 1000 บาท`\n- `5 คูณ 3`",
            "action": None,
            "sources": [],
            "image_url": None,
            "image_gallery": []
        }


# Singleton instance
calculator_service = CalculatorService()
