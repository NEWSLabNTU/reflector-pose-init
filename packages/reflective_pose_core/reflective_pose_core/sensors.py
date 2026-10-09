"""LiDAR beam models, by name.

The detector needs one fact about the sensor that the point cloud does not
carry: at which elevations it samples. That decides how many returns a board of
known size *should* produce at a given range and height, which is what the
density gate and the density confidence term compare against, and the mean
row spacing decides how far short of an edge a scan may fall and still count as
having seen it. The simulator needs the same table to cast rays.

The table used to be the VLP-32C's, hard-coded. Each model here is selected by
``detector.sensor`` in the detector file, and every caller that needs beam
geometry asks this module rather than ``vlp32``.

Every model is expressed in the sensor's *intrinsic* frame -- x along the
centre of the horizontal field of view, z along the sensor's own up axis -- and
carries ``cloud_rotation``, the rotation from that frame into the frame the
driver publishes the cloud in. For a Velodyne they are the same frame. The
Seyond driver as AutoSDV runs it (``coordinate_mode: 0``) publishes the
sensor's native axes, x up, y right, z forward, and the vehicle TF rotates them
into ROS; the detector has to know that, because "elevation" is measured about
the sensor's up axis, not about the cloud frame's z.

Two models are exact and two are not, and the difference matters to whoever
tunes against them:

- ``vlp32c``, ``vlp16`` and ``vlp16_hires`` are the Nebula calibration tables,
  the same values the driver decodes with.
- ``robin_w`` is an approximation: 192 scan lines spread evenly over the 70 deg
  vertical field, 0.1 deg columns over 120 deg. The real sensor's lines are not
  constant-elevation rings and its density is not uniform. Treat its expected
  return count as an order of magnitude until a board bag says otherwise; the
  AutoSDV Robin-W profile therefore leaves the density *gate* off.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .vlp32 import load_beam_table

#: 0.2 deg: a Velodyne at 600 rpm / 10 Hz.
VELODYNE_10HZ_AZIMUTH_STEP = np.radians(0.2)


@dataclass(frozen=True)
class SensorModel:
    """One LiDAR's sampling pattern, in its intrinsic frame."""

    name: str
    #: Per-laser elevation, radians, in firing (laser id) order.
    elevation_rad: np.ndarray
    #: Per-laser azimuth offset, radians, same order. Zero for most sensors.
    azimuth_offset_rad: np.ndarray
    #: Column spacing, radians.
    azimuth_step_rad: float
    #: Horizontal field of view as (min, max) azimuth, radians; None is 360 deg.
    azimuth_fov_rad: Optional[Tuple[float, float]] = None
    #: ``cloud <- intrinsic``. Identity when the driver publishes in the
    #: sensor's own x-forward, z-up frame.
    cloud_rotation: np.ndarray = None
    #: Whether the table is the manufacturer's calibration or a model.
    exact: bool = True
    description: str = ""

    def __post_init__(self):
        rotation = np.eye(3) if self.cloud_rotation is None else self.cloud_rotation
        object.__setattr__(self, "cloud_rotation", np.asarray(rotation, dtype=np.float64))

    @property
    def elevation_table_rad(self) -> np.ndarray:
        """Sorted elevations, radians: what the density model counts rows from."""
        return np.sort(np.asarray(self.elevation_rad, dtype=np.float64))

    @property
    def mean_elevation_step_rad(self) -> float:
        """Mean gap between adjacent rows over the whole fan."""
        table = self.elevation_table_rad
        return float((table[-1] - table[0]) / max(len(table) - 1, 1))

    @property
    def up_axis(self) -> np.ndarray:
        """The sensor's own up axis, expressed in the cloud frame."""
        return self.cloud_rotation[:, 2].copy()

    @property
    def points_per_scan(self) -> int:
        """Rays per revolution or frame, before any miss or dropout."""
        lo, hi = self.azimuth_fov_rad or (0.0, 2.0 * np.pi)
        columns = int(np.ceil((hi - lo) / self.azimuth_step_rad))
        return columns * len(self.elevation_rad)


def _vlp32c() -> SensorModel:
    vertical, rotational = load_beam_table()
    return SensorModel(
        name="vlp32c",
        elevation_rad=vertical,
        azimuth_offset_rad=rotational,
        azimuth_step_rad=float(VELODYNE_10HZ_AZIMUTH_STEP),
        description=(
            "Velodyne VLP-32C, Nebula VLP32.yaml: 32 lasers, -25 to +15 deg, "
            "gaps 0.33 to 9.36 deg"
        ),
    )


# Nebula VLP16.yaml, laser-id order: -15 to +15 deg in 2 deg steps, interleaved.
VLP16_ELEVATION_DEG = np.array(
    [-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
     -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0]
)
# Nebula VLP16_hires.yaml (Puck Hi-Res): -10 to +10 deg in 4/3 deg steps.
VLP16_HIRES_ELEVATION_DEG = np.array(
    [-10.0, 0.667, -8.667, 2.0, -7.333, 3.333, -6.0, 4.667,
     -4.667, 6.0, -3.333, 7.333, -2.0, 8.667, -0.667, 10.0]
)


def _vlp16() -> SensorModel:
    return SensorModel(
        name="vlp16",
        elevation_rad=np.radians(VLP16_ELEVATION_DEG),
        azimuth_offset_rad=np.zeros(16),
        azimuth_step_rad=float(VELODYNE_10HZ_AZIMUTH_STEP),
        description="Velodyne VLP-16 (Puck), Nebula VLP16.yaml: 16 lasers, +/-15 deg, 2 deg",
    )


def _vlp16_hires() -> SensorModel:
    return SensorModel(
        name="vlp16_hires",
        elevation_rad=np.radians(VLP16_HIRES_ELEVATION_DEG),
        azimuth_offset_rad=np.zeros(16),
        azimuth_step_rad=float(VELODYNE_10HZ_AZIMUTH_STEP),
        description=(
            "Velodyne VLP-16 Hi-Res, Nebula VLP16_hires.yaml: 16 lasers, "
            "+/-10 deg, 1.33 deg"
        ),
    )


ROBIN_W_LINES = 192
ROBIN_W_VERTICAL_FOV_DEG = 70.0
ROBIN_W_HORIZONTAL_FOV_DEG = 120.0
ROBIN_W_AZIMUTH_STEP_DEG = 0.1

#: Seyond native axes (x up, y right, z forward) from intrinsic (x forward,
#: y left, z up). This is the frame the Seyond driver publishes with
#: ``coordinate_mode: 0``, and the rotation AutoSDV's sensor_kit_calibration
#: (roll pi, pitch -pi/2) undoes. Symmetric, so it is its own inverse.
SEYOND_NATIVE_FROM_INTRINSIC = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
    ]
)


def _robin_w() -> SensorModel:
    half_v = 0.5 * ROBIN_W_VERTICAL_FOV_DEG
    half_h = np.radians(0.5 * ROBIN_W_HORIZONTAL_FOV_DEG)
    return SensorModel(
        name="robin_w",
        elevation_rad=np.radians(np.linspace(-half_v, half_v, ROBIN_W_LINES)),
        azimuth_offset_rad=np.zeros(ROBIN_W_LINES),
        azimuth_step_rad=float(np.radians(ROBIN_W_AZIMUTH_STEP_DEG)),
        azimuth_fov_rad=(-half_h, half_h),
        cloud_rotation=SEYOND_NATIVE_FROM_INTRINSIC,
        exact=False,
        description=(
            "Seyond Robin-W, MODEL: 192 lines uniform over 70 deg vertical, "
            "0.1 deg over 120 deg horizontal, Seyond native axes "
            "(coordinate_mode 0: x up, y right, z forward)"
        ),
    )


_FACTORIES = {
    "vlp32c": _vlp32c,
    "vlp16": _vlp16,
    "vlp16_hires": _vlp16_hires,
    "robin_w": _robin_w,
}
_CACHE: Dict[str, SensorModel] = {}

SENSOR_NAMES = tuple(_FACTORIES)
DEFAULT_SENSOR = "vlp32c"


def sensor_model(name: str = DEFAULT_SENSOR) -> SensorModel:
    """The named model. Unknown names are refused with the known ones listed."""
    key = str(name).strip().lower().replace("-", "_")
    if key not in _FACTORIES:
        raise ValueError(
            f"unknown sensor {name!r}; expected one of {list(SENSOR_NAMES)}"
        )
    if key not in _CACHE:
        _CACHE[key] = _FACTORIES[key]()
    return _CACHE[key]
