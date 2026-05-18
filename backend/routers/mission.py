from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException
from sqlalchemy.orm import Session

from backend.database import get_db
from backend.schemas.mission import MissionCreate, MissionOut
from backend.services import mission_service

router = APIRouter(prefix="/mission", tags=["mission"])


@router.post("/compute", response_model=MissionOut, status_code=202)
def queue_mission_compute(
    payload: MissionCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    mission_service.validate_mission_request(payload)
    mission = mission_service.create_mission(db, payload)
    background_tasks.add_task(mission_service.compute_mission_by_id, mission.id)
    return mission


@router.get("/{mission_id}", response_model=MissionOut)
def get_mission_detail(mission_id: int, db: Session = Depends(get_db)):
    mission = mission_service.get_mission(db, mission_id)
    if not mission:
        raise HTTPException(status_code=404, detail="Mission not found")
    return mission


@router.get("/", response_model=list[MissionOut])
def list_mission_runs(skip: int = 0, limit: int = 50, db: Session = Depends(get_db)):
    return mission_service.list_missions(db, skip, limit)
