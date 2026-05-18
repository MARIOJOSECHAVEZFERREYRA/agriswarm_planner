from __future__ import annotations

import os
from functools import lru_cache

from pydantic import BaseModel


class Settings(BaseModel):
    app_name: str = "AgriSwarm Planner API"
    app_version: str = "0.1.0"
    database_url: str = "sqlite:///./agriswarm.db"
    cors_origins: tuple[str, ...] = ("http://localhost:5173",)
    auto_create_schema: bool = True
    model_config = {"frozen": True}

    @classmethod
    def from_env(cls) -> "Settings":
        raw_origins = os.getenv("AGRISWARM_CORS_ORIGINS")
        cors_origins = tuple(
            origin.strip()
            for origin in (raw_origins or "http://localhost:5173").split(",")
            if origin.strip()
        )
        return cls(
            app_name=os.getenv("AGRISWARM_APP_NAME", cls.model_fields["app_name"].default),
            app_version=os.getenv("AGRISWARM_APP_VERSION", cls.model_fields["app_version"].default),
            database_url=os.getenv("AGRISWARM_DATABASE_URL", cls.model_fields["database_url"].default),
            cors_origins=cors_origins or cls.model_fields["cors_origins"].default,
            auto_create_schema=_env_bool(
                "AGRISWARM_AUTO_CREATE_SCHEMA",
                cls.model_fields["auto_create_schema"].default,
            ),
        )


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    return Settings.from_env()
