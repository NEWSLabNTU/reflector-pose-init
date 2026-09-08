"""Read the shared ROS parameter YAML without importing ROS."""

from dataclasses import fields
from pathlib import Path
from typing import Tuple

import yaml

from .anchor import AnchorParams
from .detector import DetectorParams


def default_config_path() -> str:
    """Installed package config, with a source-tree fallback for tests."""
    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        pass
    else:
        try:
            return str(
                Path(get_package_share_directory("golfcart_board_initializer"))
                / "config"
                / "board_initializer.param.yaml"
            )
        except PackageNotFoundError:
            pass
    return str(Path(__file__).parent.parent / "config" / "board_initializer.param.yaml")


def load_anchor_config(path: str) -> Tuple[AnchorParams, DetectorParams]:
    """Load anchoring and detector inputs from a ROS 2 parameter YAML file."""
    with open(path, encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    try:
        values = document["/**"]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            f"{path} must contain /**.ros__parameters in ROS 2 YAML format"
        ) from error

    pose = values.get("board_pose_in_map")
    if not isinstance(pose, list) or len(pose) != 6:
        raise ValueError(
            f"{path}: board_pose_in_map must be [x, y, z, roll, pitch, yaw] "
            "with angles in radians"
        )

    anchor_names = {field.name for field in fields(AnchorParams)}
    detector_names = {field.name for field in fields(DetectorParams)}
    anchor_values = {name: values[name] for name in anchor_names if name in values}
    anchor_values["board_pose_in_map"] = tuple(float(value) for value in pose)
    detector_values = {
        name: values[name]
        for name in detector_names
        if name in values and name not in {"viewpoint", "elevation_table_rad"}
    }
    if "extent_tolerance" in detector_values:
        detector_values["extent_tolerance"] = tuple(detector_values["extent_tolerance"])
    return AnchorParams(**anchor_values), DetectorParams(**detector_values)
