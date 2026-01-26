"""
🧪 Integration Flow Test: Entity Extraction & Navigation Routing
ทดสอบการไหลของข้อมูลจาก QueryInterpreter -> RAGOrchestrator -> NavigationService
เน้นการตรวจสอบว่า skip_cleaning flag ถูกส่งต่อถูกต้องหรือไม่

วิธีรัน:
    python Back-end/tests/test_entity_extraction_flow.py
"""

import unittest
from unittest.mock import MagicMock, AsyncMock, patch
import sys
import os
import asyncio

# Setup paths to import core modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 🩹 MOCK External Dependencies BEFORE importing core modules
# This allows running tests even if some packages (like pymongo/bson) are missing in the test env
# 🩹 MOCK External Dependencies BEFORE importing core modules
# This allows running tests even if some packages (like pymongo/bson) are missing in the test env
sys.modules['bson'] = MagicMock()
sys.modules['bson.objectid'] = MagicMock()
sys.modules['bson.errors'] = MagicMock() # Explicitly mock errors
sys.modules['pymongo'] = MagicMock()
sys.modules['pymongo.errors'] = MagicMock() # Proactive mock

sys.modules['pydantic'] = MagicMock() # Proactive mock
sys.modules['qdrant_client'] = MagicMock()
sys.modules['sentence_transformers'] = MagicMock()

# Mock Google Cloud & GenAI
sys.modules['google'] = MagicMock()
sys.modules['google.auth'] = MagicMock()
sys.modules['google.oauth2'] = MagicMock()
# Explicitly mock the submodule that caused the error
sys.modules['google.oauth2.credentials'] = MagicMock() 

sys.modules['google.auth.transport'] = MagicMock()
sys.modules['google.auth.transport.requests'] = MagicMock()
sys.modules['google.auth.transport.grpc'] = MagicMock()
sys.modules['google.generativeai'] = MagicMock()
sys.modules['google.assistant'] = MagicMock()
sys.modules['google.assistant.embedded'] = MagicMock()
sys.modules['google.assistant.embedded.v1alpha2'] = MagicMock()

sys.modules['googleapiclient'] = MagicMock()
sys.modules['googleapiclient.discovery'] = MagicMock()
sys.modules['googleapiclient.errors'] = MagicMock() # Explicitly mock errors

# Other libs
sys.modules['grpc'] = MagicMock()
sys.modules['groq'] = MagicMock()
sys.modules['numpy'] = MagicMock()
sys.modules['pandas'] = MagicMock()
sys.modules['dotenv'] = MagicMock()
sys.modules['torch'] = MagicMock()  # Mock Torch!
sys.modules['transformers'] = MagicMock() # Mock Transformers!
sys.modules['PIL'] = MagicMock() # Mock Pillow!
sys.modules['requests'] = MagicMock() # Mock Requests!
sys.modules['yt_dlp'] = MagicMock()  # Mock Youtube DL
sys.modules['isodate'] = MagicMock() # Mock IsoDate

from core.ai_models.rag_orchestrator import RAGOrchestrator
from core.ai_models.services.navigation_service import NavigationService

class TestEntityExtractionFlow(unittest.TestCase):
    
    def setUp(self):
        # Mock Dependencies for RAGOrchestrator
        self.mock_mongo = MagicMock()
        self.mock_qdrant = MagicMock()
        self.mock_interpreter = AsyncMock()
        self.mock_nav_service = AsyncMock()  # Mock NavigationService inside Orchestrator
        self.mock_analytics = MagicMock()
        self.mock_session = AsyncMock()
        self.mock_image = AsyncMock()
        self.mock_tts = AsyncMock()
        self.mock_prompt = MagicMock()
        self.mock_key_manager = MagicMock()

        # Instantiate RAGOrchestrator with Mocks (Injecting mocks where possible)
        # Note: Since RAGOrchestrator initializes its own services in __init__, 
        # we might need to patch them AFTER instantiation or use patch context.
        # Here we will try to instantiate and then swap attributes.
        
        with patch('core.ai_models.rag_orchestrator.MongoDBManager', return_value=self.mock_mongo), \
             patch('core.ai_models.rag_orchestrator.QdrantClient', return_value=self.mock_qdrant), \
             patch('core.ai_models.rag_orchestrator.QueryInterpreter', return_value=self.mock_interpreter), \
             patch('core.ai_models.rag_orchestrator.NavigationService', return_value=self.mock_nav_service), \
             patch('core.ai_models.rag_orchestrator.AnalyticsHandler', return_value=self.mock_analytics), \
             patch('core.ai_models.rag_orchestrator.AlertManager'), \
             patch('core.ai_models.rag_orchestrator.SessionManager', return_value=self.mock_session), \
             patch('core.ai_models.rag_orchestrator.ImageService', return_value=self.mock_image), \
             patch('core.ai_models.rag_orchestrator.PromptEngine', return_value=self.mock_prompt), \
             patch('core.ai_models.rag_orchestrator.CrossEncoder'), \
             patch('core.ai_models.rag_orchestrator.SentinelModel'), \
             patch('core.ai_models.rag_orchestrator.KnowledgeGapService'):
             
             self.orchestrator = RAGOrchestrator()
             
             # Force swap dependencies to be sure
             self.orchestrator.query_interpreter = self.mock_interpreter
             self.orchestrator.nav_service = self.mock_nav_service
             self.orchestrator.session_manager = self.mock_session

    def test_navigation_flow_with_llm_entity(self):
        """
        ✅ Test Case 1: LLM ระบุ NAVIGATE_TO + Entity ชัดเจน
        Expectation: RAGOrchestrator ต้องเรียก handle_get_directions พร้อม skip_cleaning=True
        """
        user_query = "อยากไปวัดพระธาตุเขาน้อย รู้จักมั้ยครับ"
        
        # 1. Setup LLM Response (Mock)
        llm_interpolation = {
            "intent": "NAVIGATE_TO",
            "entity": "วัดพระธาตุเขาน้อย",
            "corrected_query": "วัดพระธาตุเขาน้อย",
            "sub_queries": ["วัดพระธาตุเขาน้อย"],
            "is_complex": False
        }
        self.mock_interpreter.interpret_and_route.return_value = llm_interpolation
        
        # 2. Run Orbit
        asyncio.run(self.orchestrator.answer_query(query=user_query, session_id="test_sess"))
        
        # 3. Verify
        # Check if interpret_and_route was called
        self.mock_interpreter.interpret_and_route.assert_called_once_with(user_query)
        
        # Check if handle_get_directions (on NavService) was called with skip_cleaning=True
        self.mock_nav_service.handle_get_directions.assert_called_with(
            entity_slug="วัดพระธาตุเขาน้อย",
            user_lat=0.0,
            user_lon=0.0,
            skip_cleaning=True  # <--- MUST BE TRUE
        )
        print("✅ Test 1 Passed: LLM Navigation triggers skip_cleaning=True")

    def test_navigation_flow_fallback(self):
        """
        ✅ Test Case 2: Intent อื่นๆ แต่มี Keyword นำทาง (Legacy/Fallback Logic)
        Expectation: RAGOrchestrator (Fallback Logic) เรียก handle_get_directions 
        (โดย default signature ปกติจะไม่มี skip_cleaning หรือเป็น False ถ้าไม่ได้แก้ตรงนั้น)
        *หมายเหตุ: ใน RAGOrchestrator เก่า Fallback logic ไม่ได้ส่ง skip_cleaning=True*
        """
        user_query = "นำทางไป วัดภูมินทร์"
        
        # 1. Setup LLM Response (Mock -> INFORMATIONAL or Unknown)
        llm_interpolation = {
            "intent": "INFORMATIONAL", # LLM Missed it
            "entity": None,
            "corrected_query": "นำทางไป วัดภูมินทร์"
        }
        self.mock_interpreter.interpret_and_route.return_value = llm_interpolation
        
        # 2. Run Orbit
        asyncio.run(self.orchestrator.answer_query(query=user_query, session_id="test_sess"))
        
        # 3. Verify
        # Should hit the fallback keyword check logic
        # It slices "นำทางไป " out manually -> "วัดภูมินทร์"
        
        # Verify call arguments
        # Note: The fallback logic calls self.handle_get_directions(entity_slug=X) without skip_cleaning kwarg
        # So skip_cleaning should default to False (or not be present in kwargs depending on implementation)
        
        args, kwargs = self.mock_nav_service.handle_get_directions.call_args
        
        # Arg 0 should be "วัดภูมินทร์"
        self.assertEqual(kwargs.get('entity_slug'), "วัดภูมินทร์")
        
        # skip_cleaning should essentially be False/None/Default
        # If we passed kwargs explicitly in fallback, check it. If not, it defaults.
        val = kwargs.get('skip_cleaning', False)
        self.assertFalse(val, "Fallback logic should NOT skip cleaning (unless explicitly updated)")
        
        print("✅ Test 2 Passed: Fallback logic relies on internal cleaning (skip_cleaning=False)")

    def test_navigation_service_clean_logic(self):
        """
        ✅ Test Case 3: ทดสอบ Logic ภายใน NavigationService โดยตรง (Unit Test)
        """
        # Create real instance with mocks
        real_nav_service = NavigationService(self.mock_mongo, self.mock_prompt)
        
        # 3.1 Test with skip_cleaning=True
        asyncio.run(real_nav_service.handle_get_directions(
            entity_slug="อยากไปวัดพระธาตุเขาน้อย รู้จักมั้ยครับ", 
            skip_cleaning=True
        ))
        
        # It should call mongo_manager.get_location_by_slug with ORIGINAL string (stripped)
        self.mock_mongo.get_location_by_slug.assert_any_call("อยากไปวัดพระธาตุเขาน้อย รู้จักมั้ยครับ")
        print("✅ Test 3.1 Passed: NavigationService respects skip_cleaning=True")
        
        # 3.2 Test with skip_cleaning=False (Default)
        self.mock_mongo.reset_mock()
        asyncio.run(real_nav_service.handle_get_directions(
            entity_slug="อยากไปวัดพระธาตุเขาน้อย รู้จักมั้ยครับ", 
            skip_cleaning=False
        ))
        
        # It should call _clean_navigation_entity -> "วัดพระธาตุเขาน้อย"
        # and then call get_location_by_slug with CLEAN string
        self.mock_mongo.get_location_by_slug.assert_any_call("วัดพระธาตุเขาน้อย")
        print("✅ Test 3.2 Passed: NavigationService cleans entity when skip_cleaning=False")

if __name__ == '__main__':
    unittest.main()
