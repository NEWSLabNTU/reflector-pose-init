"""What the node does with one detection result.

One pure function, ``judge``, turns a ``DetectResult`` into a ``Verdict``:
whether to publish, what /diagnostics should say, and at what level. The node
calls it once per attempt and does exactly what it says; nothing here touches
ROS beyond the diagnostic level constants, so the three outcomes can be pinned
by tests with real results from the simulator scenes.

None of the outcomes is terminal. That is a change of mind, and the reasoning
is worth keeping. The one-shot initializer this node grew out of treated a
second surviving candidate as proof that the one-board assumption was broken,
and stopped for good rather than risk a confident wrong pose. Refusing to
publish that frame is still right. Refusing to look at any later frame is
not: the vehicle is now driven around a basement with hundreds of
retroreflective clusters in it while it hunts for the board, so a transient
second candidate is expected, and a detector that latches off the first time
two reflectors share a frame never recovers and looks like a hang.
``NO_CANDIDATE`` always retried; ``AMBIGUOUS`` now retries the same way.

The confidence gate is the second half of the same thought. Every geometric
survivor used to be published, with the covariance left to say how good it
was, which spread one judgement across two packages. Now the detection
carries a scalar in [0, 1] (see ``reflective_pose_core.detector.confidence_terms``)
and a survivor below ``min_confidence`` is suppressed here, with the terms
that dragged it down on the diagnostic.
"""

from dataclasses import dataclass, field
from typing import List, Tuple

from diagnostic_msgs.msg import DiagnosticStatus

from reflective_pose_core.detector import BoardDetection, DetectResult, Status


@dataclass
class Verdict:
    """The decision for one attempt, and what to tell whoever is watching."""

    publish: bool
    level: bytes
    message: str
    values: List[Tuple[str, str]] = field(default_factory=list)
    #: Never set today. Kept so the node's state machine has one place to
    #: read it from if a genuinely unrecoverable outcome is ever added.
    terminal: bool = False


def _candidate_summary(candidate: BoardDetection) -> str:
    return (
        f"{candidate.range_m:.1f} m, {candidate.n_points} pts, "
        f"{candidate.extents[0]:.2f}x{candidate.extents[1]:.2f} m, "
        f"conf {candidate.confidence:.2f}"
    )


def _rejection_summary(result: DetectResult) -> str:
    return ", ".join(
        f"{rejection.reason} {rejection.detail}".strip() for rejection in result.rejections
    )


def _confidence_values(detection: BoardDetection) -> List[Tuple[str, str]]:
    values = [("confidence", f"{detection.confidence:.2f}")]
    values += [
        (f"confidence.{name}", f"{value:.2f}")
        for name, value in detection.confidence_terms.items()
    ]
    return values


def _detection_values(detection: BoardDetection) -> List[Tuple[str, str]]:
    horizontal_ok, vertical_ok = detection.centre_constrained
    return [
        ("range_m", f"{detection.range_m:.1f}"),
        ("points", str(detection.n_points)),
        ("extents_m", f"{detection.extents[0]:.2f}x{detection.extents[1]:.2f}"),
        ("plane_residual_m", f"{detection.plane_residual:.3f}"),
        ("centre_constrained", f"h={horizontal_ok} v={vertical_ok}"),
        (
            "observed_edges",
            " ".join(name for name, seen in detection.observed_edges.items() if seen) or "none",
        ),
    ]


def judge(result: DetectResult, min_confidence: float) -> Verdict:
    """Decide one attempt. Publish only a lone survivor at or above the gate."""
    if result.status is Status.AMBIGUOUS:
        candidates = result.candidates
        details = "; ".join(_candidate_summary(c) for c in candidates)
        rejected = _rejection_summary(result)
        values = [
            ("candidates", str(len(candidates))),
            ("clusters", str(result.n_clusters)),
            ("retro_points", str(result.n_after_gates)),
            ("confidence", "0.00"),
        ]
        values += [
            (f"candidate_{index}", _candidate_summary(candidate))
            for index, candidate in enumerate(candidates, start=1)
        ]
        return Verdict(
            publish=False,
            level=DiagnosticStatus.WARN,
            message=(
                f"ambiguous: {len(candidates)} board candidates survived gating "
                f"[{details}] out of {result.n_clusters} clusters "
                f"(rejected: {rejected or 'none'}); nothing published, next batch continues"
            ),
            values=values,
        )

    if result.status is not Status.OK:
        reasons = _rejection_summary(result)
        return Verdict(
            publish=False,
            level=DiagnosticStatus.WARN,
            message=(
                f"no candidate (retro points {result.n_after_gates}, "
                f"clusters {result.n_clusters}): {reasons or 'nothing clustered'}"
            ),
            values=[
                ("retro_points", str(result.n_after_gates)),
                ("clusters", str(result.n_clusters)),
                ("confidence", "0.00"),
            ],
        )

    detection = result.detection
    values = _confidence_values(detection) + _detection_values(detection)

    if detection.confidence < min_confidence:
        weakest = sorted(detection.confidence_terms.items(), key=lambda item: item[1])[:2]
        why = ", ".join(f"{name} {value:.2f}" for name, value in weakest)
        return Verdict(
            publish=False,
            level=DiagnosticStatus.WARN,
            message=(
                f"low confidence {detection.confidence:.2f} < {min_confidence:.2f} "
                f"(weakest: {why}) at {detection.range_m:.1f} m, "
                f"{detection.n_points} pts; nothing published"
            ),
            values=values + [("min_confidence", f"{min_confidence:.2f}")],
        )

    return Verdict(
        publish=True,
        level=DiagnosticStatus.OK,
        message=(
            f"board detected at {detection.range_m:.1f} m, {detection.n_points} pts, "
            f"confidence {detection.confidence:.2f}"
        ),
        values=values + [("min_confidence", f"{min_confidence:.2f}")],
    )
