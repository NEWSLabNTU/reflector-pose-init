"""PLY and PCD round trips, with intensity as the thing that must survive."""

import struct

import numpy as np
import pytest

from golfcart_board_initializer.pointcloud_io import (
    PointCloud,
    convert,
    read_cloud,
    read_pcd,
    read_ply,
    write_pcd,
)


def sample_cloud(count=500, with_intensity=True):
    rng = np.random.default_rng(0)
    points = rng.uniform(-10.0, 10.0, size=(count, 3))
    intensity = None
    if with_intensity:
        # Spans both bands the VLP-32C distinguishes: diffuse 0-100, retro 101-255.
        intensity = np.concatenate(
            (rng.uniform(0, 100, count // 2), rng.uniform(101, 255, count - count // 2))
        )
    return PointCloud(points=points, intensity=intensity)


def write_ply(path, cloud, binary=True, intensity_name="intensity"):
    count = len(cloud)
    header = [
        "ply",
        f"format {'binary_little_endian' if binary else 'ascii'} 1.0",
        f"element vertex {count}",
        "property float x",
        "property float y",
        "property float z",
    ]
    if cloud.has_intensity:
        header.append(f"property float {intensity_name}")
    header.append("end_header")

    columns = [cloud.points[:, 0], cloud.points[:, 1], cloud.points[:, 2]]
    if cloud.has_intensity:
        columns.append(cloud.intensity)
    values = np.column_stack(columns).astype(np.float32)

    with open(path, "wb") as handle:
        handle.write(("\n".join(header) + "\n").encode("ascii"))
        if binary:
            handle.write(values.tobytes())
        else:
            np.savetxt(handle, values, fmt="%.6f")


def test_binary_ply_round_trip(tmp_path):
    cloud = sample_cloud()
    path = tmp_path / "cloud.ply"
    write_ply(path, cloud)

    loaded = read_ply(str(path))
    assert len(loaded) == len(cloud)
    assert np.allclose(loaded.points, cloud.points, atol=1e-4)
    assert np.allclose(loaded.intensity, cloud.intensity, atol=1e-3)


def test_ascii_ply_round_trip(tmp_path):
    cloud = sample_cloud(count=50)
    path = tmp_path / "cloud_ascii.ply"
    write_ply(path, cloud, binary=False)

    loaded = read_ply(str(path))
    assert np.allclose(loaded.points, cloud.points, atol=1e-4)
    assert np.allclose(loaded.intensity, cloud.intensity, atol=1e-3)


def test_glim_scalar_intensity_field_is_recognised(tmp_path):
    """GLIM names the field scalar_intensity; PCL and Autoware want intensity."""
    cloud = sample_cloud(count=100)
    path = tmp_path / "glim.ply"
    write_ply(path, cloud, intensity_name="scalar_intensity")

    loaded = read_ply(str(path))
    assert loaded.has_intensity
    assert np.allclose(loaded.intensity, cloud.intensity, atol=1e-3)


def test_pcd_binary_round_trip(tmp_path):
    cloud = sample_cloud()
    path = tmp_path / "cloud.pcd"
    write_pcd(str(path), cloud)

    loaded = read_pcd(str(path))
    assert np.allclose(loaded.points, cloud.points, atol=1e-4)
    assert np.allclose(loaded.intensity, cloud.intensity, atol=1e-3)


def test_pcd_ascii_round_trip(tmp_path):
    cloud = sample_cloud(count=80)
    path = tmp_path / "cloud_ascii.pcd"
    write_pcd(str(path), cloud, binary=False)

    loaded = read_pcd(str(path))
    assert np.allclose(loaded.points, cloud.points, atol=1e-4)
    assert np.allclose(loaded.intensity, cloud.intensity, atol=1e-3)


def test_written_pcd_header_matches_pcl_expectations(tmp_path):
    path = tmp_path / "fields.pcd"
    write_pcd(str(path), sample_cloud(count=10))

    header = path.read_bytes().split(b"DATA")[0].decode("ascii")
    assert "FIELDS x y z intensity" in header
    assert "TYPE F F F F" in header
    assert "POINTS 10" in header


def test_conversion_preserves_intensity(tmp_path):
    """The conversion Open3D would silently break."""
    cloud = sample_cloud()
    source = tmp_path / "glim.ply"
    destination = tmp_path / "map.pcd"
    write_ply(source, cloud)

    convert(str(source), str(destination))
    loaded = read_cloud(str(destination))

    assert loaded.has_intensity
    assert np.allclose(loaded.intensity, cloud.intensity, atol=1e-3)
    assert loaded.intensity.max() > 101  # the retro band survived


def test_cloud_without_intensity_round_trips(tmp_path):
    cloud = sample_cloud(with_intensity=False)
    path = tmp_path / "plain.pcd"
    write_pcd(str(path), cloud)

    loaded = read_pcd(str(path))
    assert not loaded.has_intensity
    assert np.allclose(loaded.points, cloud.points, atol=1e-4)


def test_compressed_pcd_is_refused_not_half_parsed(tmp_path):
    path = tmp_path / "compressed.pcd"
    path.write_bytes(
        b"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 1\nHEIGHT 1\n"
        b"POINTS 1\nDATA binary_compressed\n" + struct.pack("<3f", 0, 0, 0)
    )

    with pytest.raises(ValueError, match="binary_compressed"):
        read_pcd(str(path))


def test_unknown_extension_is_refused(tmp_path):
    path = tmp_path / "cloud.xyz"
    path.write_text("0 0 0\n")

    with pytest.raises(ValueError, match=r"\.ply or \.pcd"):
        read_cloud(str(path))
