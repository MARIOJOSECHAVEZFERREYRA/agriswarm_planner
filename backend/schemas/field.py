from pydantic import BaseModel, Field


class FieldSummary(BaseModel):
    name: str
    category: str


class FieldDocument(BaseModel):
    name: str
    description: str | None = None
    boundary: list[list[float]] = Field(..., min_length=3)
    obstacles: list[list[list[float]]] = Field(default_factory=list)
    base_point: list[float] = Field(..., min_length=2, max_length=2)
    ugv_polyline: list[list[float]] | None = None
    ugv_speed: float | None = None
    ugv_t_service: float | None = None
