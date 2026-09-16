"""Interface placeholders for the planned module pipeline."""

from typing import Protocol, Sequence

from common.types import (
    BundleSequence,
    Gate,
    NavigationDecision,
    OpenPoint,
    PerceptionResult,
    PlannedPath,
    Pose2D,
    RankedOpenPoint,
    VisibilityGraph,
)


class PerceptionProvider(Protocol):
    """Provide ROS-independent local perception results."""

    def compute(self, center: Pose2D) -> PerceptionResult:
        raise NotImplementedError


class GraphBundleProvider(Protocol):
    """Prepare ranking, graph, and bundle outputs."""

    def update(self, perception: PerceptionResult) -> None:
        raise NotImplementedError

    def ranked_open_points(self) -> Sequence[RankedOpenPoint]:
        raise NotImplementedError

    def visibility_graph(self) -> VisibilityGraph:
        raise NotImplementedError

    def bundle_sequence(self) -> BundleSequence:
        raise NotImplementedError

    def gates(self) -> Sequence[Gate]:
        raise NotImplementedError


class NavigationProvider(Protocol):
    """Decide navigation state and prepare a path."""

    def decide(self) -> NavigationDecision:
        raise NotImplementedError

    def plan(self, target: OpenPoint) -> PlannedPath:
        raise NotImplementedError