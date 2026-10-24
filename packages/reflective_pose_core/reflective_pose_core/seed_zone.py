"""Where a detection may seed localization from: range, bearing and aspect.

A detection that passes every gate in ``detector`` is a planar retroreflector
of the right size at the right height. That is not the same as *the* board,
and a pose composed from the wrong reflector is a confident wrong
initialization. Golf-cart Phase 8 measured the remedy (roadmap L10, L6): the
real board is detected every time from a known zone, the false one at
Ming-Da (15.23, 21.65) is seen from well outside it (58-62 deg of aspect), so a
pose is acted on only when its own detection lies inside the zone.

Three numbers per detection, defined as the golf-cart repository's
``scripts/localization/board_range_report.py`` defines them, so an L10 table
and a log line here mean the same thing:

- **range**: sensor origin to the board centre, 3-D (``BoardDetection.range_m``)
- **bearing**: the board centre's direction from ``base_link``, deg, 0 dead
  ahead, positive left
- **aspect**: the sensor's direction seen from the board, deg, 0 on the
  board's normal; horizontal

The report computes them from a surveyed board and an NDT track; here they
come from the detection itself (its centre and fitted normal), which is what
"gate each detection on its own range, bearing and aspect" asks. For the real
board the two agree to the detection error.

The zone is open by default: a detector file that does not set it publishes
exactly what it did before.
"""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from .detector import BoardDetection


@dataclass(frozen=True)
class ViewGeometry:
    """How one detection was seen. Degrees, metres."""

    range_m: float
    bearing_deg: float
    aspect_deg: float


def _wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def view_geometry(detection: BoardDetection, transform_base_sensor: np.ndarray) -> ViewGeometry:
    """Range, bearing and aspect of a detection.

    ``transform_base_sensor`` is the ``base_link <- sensor`` TF. Aspect is
    measured in the base frame's horizontal plane, where yaw is the only
    rotation that matters; a difference of two angles, it does not move with
    the sensor's mounting yaw.
    """
    transform = np.asarray(transform_base_sensor, dtype=np.float64)
    rotation, translation = transform[:3, :3], transform[:3, 3]
    centre_base = rotation @ np.asarray(detection.centre, dtype=np.float64) + translation
    normal_base = rotation @ np.asarray(detection.normal, dtype=np.float64)
    to_sensor = translation - centre_base

    bearing = math.degrees(math.atan2(centre_base[1], centre_base[0]))
    aspect = _wrap_deg(
        math.degrees(
            math.atan2(to_sensor[1], to_sensor[0]) - math.atan2(normal_base[1], normal_base[0])
        )
    )
    return ViewGeometry(
        range_m=float(detection.range_m), bearing_deg=bearing, aspect_deg=aspect
    )


@dataclass(frozen=True)
class SeedZone:
    """The zone a detection must lie in to be published as a seed pose.

    Bounds are inclusive; angles are absolute values, symmetric about zero.
    The defaults are open: range 0 to infinity, bearing within 180 deg, aspect
    within 90 deg (the fitted normal points back at the sensor, so every
    detection's aspect is already inside 90).
    """

    range_min: float = 0.0
    range_max: float = float("inf")
    bearing_max_deg: float = 180.0
    aspect_max_deg: float = 90.0

    def __post_init__(self):
        values = (self.range_min, self.range_max, self.bearing_max_deg, self.aspect_max_deg)
        if any(math.isnan(float(value)) for value in values):
            raise ValueError(f"seed_zone bounds must be numbers, got {values}")
        if self.range_min < 0.0 or self.range_max <= self.range_min:
            raise ValueError(
                f"seed_zone needs 0 <= range_min < range_max, got "
                f"{self.range_min} .. {self.range_max}"
            )
        if not 0.0 < self.bearing_max_deg <= 180.0:
            raise ValueError(
                f"seed_zone.bearing_max_deg must be in (0, 180], got {self.bearing_max_deg}"
            )
        if not 0.0 < self.aspect_max_deg <= 90.0:
            raise ValueError(
                f"seed_zone.aspect_max_deg must be in (0, 90], got {self.aspect_max_deg}"
            )

    @property
    def enabled(self) -> bool:
        """Whether any bound is tighter than the open default."""
        return self != SeedZone()

    def rejection(self, view: ViewGeometry) -> Optional[str]:
        """None inside the zone, else every bound the view breaks, in words."""
        reasons = []
        if not self.range_min <= view.range_m <= self.range_max:
            reasons.append(
                f"range {view.range_m:.2f} m outside {self.range_min:g}-{self.range_max:g} m"
            )
        if abs(view.bearing_deg) > self.bearing_max_deg:
            reasons.append(
                f"bearing {view.bearing_deg:+.1f} deg beyond {self.bearing_max_deg:g} deg"
            )
        if abs(view.aspect_deg) > self.aspect_max_deg:
            reasons.append(
                f"aspect {view.aspect_deg:+.1f} deg beyond {self.aspect_max_deg:g} deg"
            )
        return "; ".join(reasons) or None
