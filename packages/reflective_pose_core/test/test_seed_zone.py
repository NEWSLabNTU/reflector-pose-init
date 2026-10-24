"""The seed zone: where a detection is allowed to seed localization from.

Golf-cart Phase 8, L6 and L10. The zone is a range band plus two angles, each
detection judged on its own geometry: bearing (the board's direction from
base_link, 0 dead ahead, + left) and aspect (the sensor's direction seen from
the board, 0 on its normal). The definitions are those of the parent repo's
scripts/localization/board_range_report.py, so a number read off an L10 table
means the same thing here.
"""

import math

import numpy as np
import pytest

from reflective_pose_core.detector import BoardDetection
from reflective_pose_core.geometry import make_transform, matrix_from_euler_rpy
from reflective_pose_core.seed_zone import SeedZone, ViewGeometry, view_geometry

#: The measured Ming-Da zone (L10): ahead within 60 deg, aspect within 35 deg,
#: 4-17 m.
MINGDA = SeedZone(range_min=4.0, range_max=17.0, bearing_max_deg=60.0, aspect_max_deg=35.0)


def detection_at(range_m, bearing_deg=0.0, aspect_deg=0.0, height=0.0):
    """A board ``range_m`` from the sensor along ``bearing_deg``, turned so the
    sensor sees it ``aspect_deg`` off its normal. Sensor frame, level."""
    bearing = math.radians(bearing_deg)
    horizontal = math.sqrt(max(range_m**2 - height**2, 0.0))
    centre = np.array([horizontal * math.cos(bearing), horizontal * math.sin(bearing), height])
    # The normal points back at the sensor, then is turned so that the
    # sensor's direction from the board sits aspect_deg from it: aspect is
    # angle(sensor - centre) - angle(normal), so the normal's angle is the
    # line of sight's minus aspect.
    back = math.atan2(-centre[1], -centre[0]) - math.radians(aspect_deg)
    normal = np.array([math.cos(back), math.sin(back), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, normal)
    return BoardDetection(
        centre=centre,
        rotation=np.column_stack((normal, right, up)),
        normal=normal,
        up=up,
        right=right,
        extents=(0.6, 0.6),
        points=np.zeros((0, 3)),
        n_points=100,
        plane_residual=0.01,
        range_m=float(np.linalg.norm(centre)),
        observed_edges={},
    )


IDENTITY = np.eye(4)


# -- the geometry -------------------------------------------------------------


def test_view_geometry_reads_range_bearing_and_aspect_back():
    view = view_geometry(detection_at(10.0, bearing_deg=20.0, aspect_deg=-15.0), IDENTITY)

    assert view.range_m == pytest.approx(10.0)
    assert view.bearing_deg == pytest.approx(20.0)
    assert view.aspect_deg == pytest.approx(-15.0)


def test_bearing_is_from_base_link_and_range_from_the_sensor():
    # A sensor 0.46 m ahead of base_link and 2 m up, as on the cart: the
    # board 10 m to the sensor's left is a little less than 90 deg from
    # base_link, and the range stays the sensor's.
    base_sensor = make_transform(np.eye(3), [0.46, 0.0, 1.966])
    view = view_geometry(detection_at(10.0, bearing_deg=90.0), base_sensor)

    assert view.range_m == pytest.approx(10.0)
    assert view.bearing_deg == pytest.approx(math.degrees(math.atan2(10.0, 0.46)))


def test_aspect_does_not_depend_on_the_sensor_yaw():
    # Aspect is a difference of two horizontal angles, so a yawed mount moves
    # the bearing and leaves the aspect alone.
    base_sensor = make_transform(matrix_from_euler_rpy([0.0, 0.0, math.radians(30.0)]), [0, 0, 0])
    view = view_geometry(detection_at(8.0, bearing_deg=0.0, aspect_deg=25.0), base_sensor)

    assert view.bearing_deg == pytest.approx(30.0)
    assert view.aspect_deg == pytest.approx(25.0)


# -- the gate -----------------------------------------------------------------


def test_inside_the_zone_is_accepted():
    view = ViewGeometry(range_m=16.3, bearing_deg=-17.0, aspect_deg=-17.5)

    assert MINGDA.rejection(view) is None


@pytest.mark.parametrize("aspect", [58.0, 60.0, -62.0])
def test_the_false_board_aspect_is_rejected(aspect):
    # The Ming-Da false board at (15.23, 21.65) is seen at 58-62 deg aspect.
    reason = MINGDA.rejection(ViewGeometry(range_m=4.5, bearing_deg=10.0, aspect_deg=aspect))

    assert reason is not None
    assert "aspect" in reason
    assert "35" in reason


def test_beyond_17_m_is_rejected():
    reason = MINGDA.rejection(ViewGeometry(range_m=17.84, bearing_deg=0.0, aspect_deg=0.0))

    assert reason is not None and "range" in reason and "17.8" in reason


def test_below_4_m_is_rejected():
    reason = MINGDA.rejection(ViewGeometry(range_m=3.5, bearing_deg=0.0, aspect_deg=0.0))

    assert reason is not None and "range" in reason and "3.5" in reason


@pytest.mark.parametrize("bearing", [61.0, -75.0, 120.0])
def test_bearing_beyond_60_deg_is_rejected(bearing):
    reason = MINGDA.rejection(ViewGeometry(range_m=8.0, bearing_deg=bearing, aspect_deg=0.0))

    assert reason is not None and "bearing" in reason


def test_the_bounds_are_inclusive():
    for view in (
        ViewGeometry(4.0, 0.0, 0.0),
        ViewGeometry(17.0, 0.0, 0.0),
        ViewGeometry(10.0, 60.0, 35.0),
        ViewGeometry(10.0, -60.0, -35.0),
    ):
        assert MINGDA.rejection(view) is None


def test_every_failed_gate_is_named():
    reason = MINGDA.rejection(ViewGeometry(range_m=20.0, bearing_deg=90.0, aspect_deg=60.0))

    assert "range" in reason and "bearing" in reason and "aspect" in reason


def test_the_default_zone_is_open():
    zone = SeedZone()

    assert not zone.enabled
    for view in (
        ViewGeometry(0.5, 0.0, 0.0),
        ViewGeometry(100.0, 179.0, 89.0),
        ViewGeometry(10.0, -180.0, -90.0),
    ):
        assert zone.rejection(view) is None


def test_a_set_zone_is_enabled():
    assert MINGDA.enabled
    assert SeedZone(aspect_max_deg=35.0).enabled


@pytest.mark.parametrize(
    "kwargs",
    [
        dict(range_min=-1.0),
        dict(range_min=10.0, range_max=5.0),
        dict(bearing_max_deg=0.0),
        dict(bearing_max_deg=200.0),
        dict(aspect_max_deg=-5.0),
        dict(aspect_max_deg=95.0),
        dict(range_max=float("nan")),
    ],
)
def test_nonsense_bounds_are_refused(kwargs):
    with pytest.raises(ValueError):
        SeedZone(**kwargs)
