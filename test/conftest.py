"""Make the package importable without installing it.

The simulator lives inside the package rather than beside the tests, so the same
scene definitions drive both the offline test matrix and the live scene
publisher.
"""

import os
import sys

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEST_ROOT = os.path.dirname(os.path.abspath(__file__))

# TEST_ROOT too, so the CLI tests can reuse the map-cloud builder in
# test_anchor.py rather than keeping a second copy of it.
for path in (PACKAGE_ROOT, TEST_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)
