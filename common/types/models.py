"""Lightweight ROS-independent data models for module contracts.

These models intentionally contain no geometry, ranking, graph, or navigation
behavior. Algorithms will be added by the owning modules.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Segment2D:
    start: Point2D
    end: Point2D


@dataclass(frozen=True)
class Pose2D:
    position: Point2D
    yaw: float


@dataclass(frozen=True)
class NeighborSight:
    center: Point2D
    radius: float
    visible_boundaries: list[Segment2D] = field(default_factory=list)


@dataclass(frozen=True)
class ClosedSight:
    angular_start: float
    angular_end: float
    boundary: Optional[Segment2D] = None


@dataclass(frozen=True)
class OpenSight:
    angular_start: float
    angular_end: float


@dataclass(frozen=True)
class OpenPoint:
    point: Point2D
    sight_index: Optional[int] = None


@dataclass(frozen=True)
class PerceptionResult:
    neighbor_sight: NeighborSight
    closed_sights: list[ClosedSight] = field(default_factory=list)
    open_sights: list[OpenSight] = field(default_factory=list)
    open_points: list[OpenPoint] = field(default_factory=list)


@dataclass(frozen=True)
class RankedOpenPoint:
    open_point: OpenPoint
    rank: Optional[float] = None


@dataclass(frozen=True)
class VisibilityNode:
    node_id: str
    pose: Pose2D


@dataclass(frozen=True)
class VisibilityEdge:
    start_node_id: str
    end_node_id: str
    cost: Optional[float] = None


@dataclass
class VisibilityGraph:
    nodes: list[VisibilityNode] = field(default_factory=list)
    edges: list[VisibilityEdge] = field(default_factory=list)


@dataclass(frozen=True)
class Gate:
    left: Point2D
    right: Point2D


@dataclass(frozen=True)
class Bundle:
    segments: list[Segment2D] = field(default_factory=list)
    gate: Optional[Gate] = None


@dataclass(frozen=True)
class BundleSequence:
    bundles: list[Bundle] = field(default_factory=list)
    gates: list[Gate] = field(default_factory=list)


class NavigationState(str, Enum):
    EXPLORE = "EXPLORE"
    ESCAPE = "ESCAPE"
    GOAL_REACHED = "GOAL_REACHED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class PlannedPath:
    points: list[Point2D] = field(default_factory=list)
    length: Optional[float] = None


@dataclass(frozen=True)
class NavigationDecision:
    state: NavigationState
    selected_target: Optional[OpenPoint] = None
    planned_path: Optional[PlannedPath] = None
