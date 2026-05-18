from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from backend.core.settings import get_settings
from backend.database import Base, engine
from backend.models import drone_model, mission_model  # noqa: F401
from backend.routers import drones, fields, mission, simulation


settings = get_settings()

app = FastAPI(title=settings.app_name, version=settings.app_version)

app.add_middleware(
    CORSMiddleware,
    allow_origins=list(settings.cors_origins),
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(fields.router)
app.include_router(mission.router)
app.include_router(simulation.router)
app.include_router(drones.router)


@app.on_event("startup")
def create_tables() -> None:
    if settings.auto_create_schema:
        Base.metadata.create_all(bind=engine)


@app.get("/health")
def health():
    return {"status": "ok"}
