from pathlib import Path

from fastapi import APIRouter, HTTPException
from pydantic import ValidationError

from backend.schemas.field import FieldDocument, FieldSummary

router = APIRouter(prefix="/fields", tags=["fields"])

FIELDS_DIR = Path(__file__).resolve().parents[2] / "tests" / "test_fields"


def _iter_field_files() -> list[Path]:
    return sorted(FIELDS_DIR.rglob("*.json"))


def _load_field_document(path: Path) -> FieldDocument:
    try:
        return FieldDocument.model_validate_json(path.read_text(encoding="utf-8"))
    except ValidationError as exc:  # pragma: no cover - invalid fixtures should fail loudly
        raise HTTPException(status_code=500, detail=f"Invalid field document: {path.name}") from exc


@router.get("/", response_model=list[FieldSummary])
def list_fields():
    return [
        FieldSummary(name=path.stem, category=path.parent.name)
        for path in _iter_field_files()
    ]


@router.get("/{name}", response_model=FieldDocument)
def get_field(name: str):
    for path in _iter_field_files():
        if path.stem == name:
            return _load_field_document(path)
    raise HTTPException(status_code=404, detail=f"Field '{name}' not found")
