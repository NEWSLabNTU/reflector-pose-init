"""The share/ copy of the canonical config must not drift from core's.

Launch files need a ``$(find-pkg-share ...)`` path, and ``reflective_pose_core``
is a Python distribution whose data directory is not one. So this package ships
a copy -- and a copy of a file whose comments record measurements is exactly the
kind of thing that quietly stops matching. Byte-identical, or this fails.
"""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
COPY = PACKAGE_ROOT / "config" / "reflective_pose.yaml"
CANONICAL = (
    PACKAGE_ROOT.parent
    / "reflective_pose_core"
    / "reflective_pose_core"
    / "data"
    / "reflective_pose.yaml"
)


def test_share_config_is_byte_identical_to_core():
    assert COPY.is_file(), f"{COPY} is missing"
    assert CANONICAL.is_file(), f"{CANONICAL} is missing"
    assert COPY.read_bytes() == CANONICAL.read_bytes(), (
        f"{COPY} has drifted from {CANONICAL}; copy the canonical file over it "
        "rather than editing the copy"
    )
