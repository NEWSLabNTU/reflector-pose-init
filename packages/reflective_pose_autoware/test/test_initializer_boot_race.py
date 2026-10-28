"""A cart parked at the board while the stack boots (golf-cart Phase 8, L8).

The detector fires about once a second from the first scan, and the map can
reach the scan matcher seconds later: on the 2026-10-02 replay, five aligns
failed with "No map loaded" and the initializer stopped for good before the map
arrived. This replays that order over real ROS transport, which the policy
tests do not: the initializer runs as it does on the vehicle, the scan matcher
is a publisher of its loaded map plus an initialize service that fails until
the map is out, and the detector is a latched pose publisher.

Needs rclpy and the Autoware messages. Spins for a few seconds.
"""

import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_vehicle_msgs")
pytest.importorskip("autoware_localization_msgs")

from autoware_localization_msgs.srv import InitializeLocalization  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402

from reflective_pose_autoware.initializer_node import (  # noqa: E402
    DEFAULT_BOARD_POSE_TOPIC,
    MATCHER_MAP_TOPIC,
    BoardPoseInitializer,
    State,
)

# Detections at 5 Hz and the map after 2 s: ten failed aligns before it, twice
# the budget, so a node that counted them would have failed for good.
POSE_PERIOD = 0.2
MAP_DELAY = 2.0
MAX_ATTEMPTS = 5


class FakeMatcher:
    """The scan matcher and pose initializer, as the board initializer sees them."""

    def __init__(self, node):
        self.has_map = False
        self.calls = 0
        self.calls_before_map = 0
        # Volatile, as cuda_ndt publishes it.
        self.map_pub = node.create_publisher(PointCloud2, MATCHER_MAP_TOPIC, 1)
        node.create_service(
            InitializeLocalization, "/localization/initialize", self.on_initialize
        )

    def publish_map(self):
        self.has_map = True
        self.map_pub.publish(PointCloud2())

    def on_initialize(self, request, response):
        self.calls += 1
        if not self.has_map:
            self.calls_before_map += 1
        response.status.success = self.has_map
        response.status.message = "" if self.has_map else "NDT align server failed"
        return response


@pytest.fixture
def context():
    # A domain of its own: the topic and service names are the vehicle's.
    domain = 100 + os.getpid() % 100
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    yield ctx
    rclpy.shutdown(context=ctx)


def run_boot(context, ignore_failures_before_map):
    initializer = BoardPoseInitializer(
        context=context,
        parameter_overrides=[
            Parameter("max_attempts", value=MAX_ATTEMPTS),
            Parameter("min_attempt_spacing", value=0.0),
            Parameter("ignore_failures_before_map", value=ignore_failures_before_map),
        ],
    )
    stack = rclpy.create_node("fake_stack", context=context)
    matcher = FakeMatcher(stack)
    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    detector = stack.create_publisher(PoseWithCovarianceStamped, DEFAULT_BOARD_POSE_TOPIC, latched)

    def detect():
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        detector.publish(pose)

    start = time.monotonic()
    stack.create_timer(POSE_PERIOD, detect)
    map_timer = None

    def map_out():
        matcher.publish_map()
        map_timer.cancel()

    map_timer = stack.create_timer(MAP_DELAY, map_out)

    # The initializer blocks in wait_for_service inside a callback, so the fake
    # stack spins on its own executor or its service could never answer.
    executors = [SingleThreadedExecutor(context=context) for _ in range(2)]
    executors[0].add_node(initializer)
    executors[1].add_node(stack)
    threads = [threading.Thread(target=e.spin, daemon=True) for e in executors]
    for thread in threads:
        thread.start()
    while time.monotonic() - start < MAP_DELAY + 3.0:
        if initializer._state in (State.DONE, State.FAILED):
            break
        time.sleep(0.05)
    for executor in executors:
        executor.shutdown()
    state, attempts = initializer._state, initializer._attempts
    initializer.destroy_node()
    stack.destroy_node()
    return state, attempts, matcher


def test_parked_at_the_board_through_boot_initializes(context):
    state, attempts, matcher = run_boot(context, ignore_failures_before_map=True)

    assert matcher.calls_before_map >= MAX_ATTEMPTS
    assert state is State.DONE
    # 0 or 1: the call that succeeds may have gone out just before the map
    # reached the initializer, and then it was not counted either.
    assert attempts <= 1


def test_counting_every_failure_gives_up_before_the_map(context):
    """The control: the vehicle's behaviour before L8, on the same timeline."""
    state, attempts, matcher = run_boot(context, ignore_failures_before_map=False)

    assert state is State.FAILED
    assert attempts == MAX_ATTEMPTS
    assert matcher.calls == matcher.calls_before_map
