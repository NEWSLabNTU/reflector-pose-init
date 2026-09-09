"""Make the sibling packages importable from a plain checkout.

The tests need ``reflective_pose_core`` (what the CLI drives) and
``reflective_pose_sim`` (which builds the synthetic map cloud they drive it
with). Both are real distributions, so an installed workspace or a ``pip
install -e`` needs nothing from this file; it exists so that ``pytest test``
also works in a checkout where nothing has been installed yet.

It adds package roots only. It deliberately does not put another package's
``test/`` directory on the path: reaching into a neighbour's tests for a
fixture is what tied the old single-package test suite together, and it is the
thing the split is undoing.
"""

import sys
from importlib.util import find_spec
from pathlib import Path

PACKAGES = Path(__file__).resolve().parents[2]

for name in ("reflective_pose_core", "reflective_pose_sim"):
    if find_spec(name) is not None:
        continue
    root = str(PACKAGES / name)
    if root not in sys.path:
        sys.path.insert(0, root)
