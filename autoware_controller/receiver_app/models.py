from pydantic import BaseModel
from typing import List, Optional


class PoseModel(BaseModel):
    position: List[float]
    orientation: List[float]


class LocalizationInitRequest(BaseModel):
    pose: PoseModel


class HeaderModel(BaseModel):
    stamp: List[int]
    frame_id: str


class SetRoutePointsRequest(BaseModel):
    header: HeaderModel
    goal: PoseModel
    waypoints: List[PoseModel]


class StartLoggingRequest(BaseModel):
    filename: str


class AutowareLaunchRequest(BaseModel):
    map_path: Optional[str] = None
    vehicle_model: Optional[str] = None
    sensor_model: Optional[str] = None


class ViolationsCalculateRequest(BaseModel):
    route_lanelet_ids: Optional[List[int]] = None


class PedestrianPublishRequest(BaseModel):
    position: List[float]
    orientation: Optional[List[float]] = None
    speed: Optional[float] = 0.0
    pedestrian_id: Optional[str] = None


class PedestrianBatchItem(BaseModel):
    pedestrian_id: str
    position: List[float]
    orientation: Optional[List[float]] = None
    speed: Optional[float] = 0.0


class PedestrianBatchRequest(BaseModel):
    pedestrians: List[PedestrianBatchItem]


class TrafficSignalRequest(BaseModel):
    map_primitive_id: int
    color: Optional[int] = None
    shape: Optional[int] = None
    status: Optional[int] = None
    confidence: Optional[float] = None


class TrafficSignalItemRequest(BaseModel):
    map_primitive_id: int
    color: Optional[int] = None
    shape: Optional[int] = None
    status: Optional[int] = None
    confidence: Optional[float] = None


class TrafficSignalsRequest(BaseModel):
    signals: List[TrafficSignalItemRequest]


class SenderStartRequest(BaseModel):
    receiver_urls: Optional[List[str]] = None
