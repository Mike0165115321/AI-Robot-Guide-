from pydantic import BaseModel, Field, ConfigDict, BeforeValidator
from typing import List, Optional, Any
from bson import ObjectId
from typing_extensions import Annotated

PyObjectId = Annotated[str, BeforeValidator(str)]

class Detail(BaseModel):
    heading: str
    content: str

class RelatedInfoItem(BaseModel):
    name: str
    value: str

class SourceItem(BaseModel):
    type: str
    reference: str

class LocationBase(BaseModel):
    summary: Optional[str] = None
    id: str
    category: str
    topic: str
    title: str
    summary: str
    image_url: Optional[str] = None
    
    details: Optional[List[Detail]] = []
    keywords: Optional[List[str]] = []
    related_info: Optional[List[RelatedInfoItem]] = []
    sources: Optional[List[SourceItem]] = []

class LocationCreate(LocationBase):
    pass

class LocationInDB(LocationBase):
    mongo_id: PyObjectId = Field(..., alias="_id")

    model_config = ConfigDict(
        from_attributes=True,
        populate_by_name=True,
        arbitrary_types_allowed=True
    )

class LocationAdminSummary(BaseModel):
    mongo_id: PyObjectId = Field(..., alias="_id")
    id: str
    title: str
    category: str
    topic: str
    model_config = ConfigDict(
        from_attributes=True,
        populate_by_name=True,
        arbitrary_types_allowed=True
    )

class ChatQuery(BaseModel):
    query: str = Field(..., example="วัดที่มีภาพวาดสวยๆ คือที่ไหนครับ")

class ChatSource(BaseModel):
    title: str
    summary: Optional[str] = None
    image_url: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    image_url: Optional[str] = None 
    sources: List[ChatSource]