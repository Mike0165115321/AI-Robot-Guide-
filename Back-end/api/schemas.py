from pydantic import BaseModel, Field, ConfigDict, BeforeValidator
from typing import List, Optional, Any
from bson import ObjectId
from typing_extensions import Annotated

PyObjectId = Annotated[str, BeforeValidator(str)]
class Detail(BaseModel): heading: str; content: str
class RelatedInfoItem(BaseModel): name: str; value: str
class SourceItem(BaseModel): type: str; reference: str
class AdminLocationMetadata(BaseModel): image_prefix: Optional[str] = None
class LocationBase(BaseModel):
    slug: str = Field(..., min_length=3, pattern=r"^[a-z0-9_-]+$", description="Human-readable key")
    category: str; topic: str; title: str
    summary: Optional[str] = None
    image_url: Optional[str] = Field(None, description="Fallback image")
    details: List[Detail] = Field(default_factory=list)
    keywords: List[str] = Field(default_factory=list)
    related_info: List[RelatedInfoItem] = Field(default_factory=list)
    sources: List[SourceItem] = Field(default_factory=list)
    metadata: Optional[AdminLocationMetadata] = None
class LocationCreate(LocationBase): pass
class LocationInDB(LocationBase):
    mongo_id: PyObjectId = Field(..., alias="_id")
    model_config = ConfigDict(from_attributes=True, populate_by_name=True, arbitrary_types_allowed=True)
class LocationAdminSummary(BaseModel):
    mongo_id: PyObjectId = Field(..., alias="_id"); slug: str; title: str; category: str; topic: str
    metadata: Optional[AdminLocationMetadata] = None
    model_config = ConfigDict(from_attributes=True, populate_by_name=True, arbitrary_types_allowed=True)
class LocationAdminSummaryWithImage(LocationAdminSummary):
    preview_image_url: Optional[str] = None
class ChatQuery(BaseModel):
    query: str = Field(..., example="วัดสวยๆ ในน่าน")
class ActionPayloadPrompt(BaseModel): placeholder: str
class ActionPayloadMusicChoice(BaseModel): video_id: str; title: str; thumbnail: str; channel_title: Optional[str] = None
class SourceInfo(BaseModel):
    title: str
    summary: Optional[str] = None
    image_urls: List[str] = Field(default_factory=list)
class ChatResponse(BaseModel):
    answer: str = Field(..., description="The main text response from the AI.")
    action: Optional[str] = Field(None, description="An action for the client to perform, e.g., 'PROMPT_FOR_SONG_INPUT'.")
    action_payload: Optional[Any] = Field(None, description="Data associated with the action.")
    image_url: Optional[str] = Field(None, description="A single, primary image URL to display.")
    image_gallery: List[str] = Field(default_factory=list, description="A list of image URLs for a gallery view.")
    sources: List[SourceInfo] = Field(default_factory=list, description="A list of source documents with their own images.")