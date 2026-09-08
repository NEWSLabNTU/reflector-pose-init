"""VLP-32C beam geometry.

The per-laser elevation and azimuth corrections are the same values the Nebula
driver uses. They are loaded from the installed calibration file when it is
present and fall back to the embedded copy otherwise, so this module works on a
machine without Autoware — which is what lets the detector tests run anywhere.

Two properties of this table drive detector parameters, and neither is obvious
from the "32 rings, 40 degree field of view" summary:

- Elevation gaps run from 0.333 deg to 9.36 deg. The board's return count
  therefore depends on *where* in the fan it lands, not just on its range.
- The single 9.36 deg gap sits at the bottom of the fan, between -25.0 and
  -15.6 deg, which is what sets the detector's minimum working range.
"""

from typing import Tuple

import numpy as np

NEBULA_CALIBRATION = "/opt/ros/humble/share/nebula_decoders/calibration/velodyne/VLP32.yaml"

VERT_CORRECTION = np.array(
    [
        -0.436332, -0.017453, -0.029095, -0.272952, -0.197397, 0.0, -0.011641,
        -0.154339, -0.126606, 0.005812, -0.005812, -0.107303, -0.093078, 0.023265,
        0.011641, -0.069813, -0.081455, 0.029095, 0.017453, -0.064001, -0.058172,
        0.058172, 0.040719, -0.046548, -0.05236, 0.122173, 0.081455, -0.040719,
        -0.034907, 0.261799, 0.180345, -0.023265,
    ]
)
ROT_CORRECTION = np.array(
    [
        -0.024435, 0.073304, -0.024435, 0.024435, -0.024435, 0.024435, -0.073304,
        0.024435, -0.024435, 0.073304, -0.024435, 0.024435, -0.073304, 0.024435,
        -0.073304, 0.024435, -0.024435, 0.073304, -0.024435, 0.073304, -0.073304,
        0.024435, -0.024435, 0.024435, -0.024435, 0.024435, -0.024435, 0.073304,
        -0.073304, 0.024435, -0.024435, 0.024435,
    ]
)


def load_beam_table(path: str = NEBULA_CALIBRATION) -> Tuple[np.ndarray, np.ndarray]:
    """Elevation and azimuth corrections in radians, laser_id order."""
    try:
        import yaml  # lazy, so the fallback path needs no PyYAML

        with open(path) as handle:
            data = yaml.safe_load(handle)
        lasers = sorted(data["lasers"], key=lambda laser: laser["laser_id"])
        return (
            np.array([laser["vert_correction"] for laser in lasers]),
            np.array([laser["rot_correction"] for laser in lasers]),
        )
    except Exception:
        return VERT_CORRECTION.copy(), ROT_CORRECTION.copy()


def elevation_table() -> np.ndarray:
    """Sorted elevation angles in radians."""
    vertical, _ = load_beam_table()
    return np.sort(vertical)
