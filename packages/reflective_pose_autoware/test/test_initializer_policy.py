"""The three decisions this package exists to make.

The speed gate, the attempt budget and the fallback policy were verified by
reading in the old node, because they were tangled with cloud accumulation and
TF. They are separable now, so they are tested: the split moved them across a
topic boundary and the roadmap calls that out as the risk of the split.

Needs rclpy and the Autoware messages. It does not spin, publish or call a
service -- the client is replaced with a stub that records requests.
"""

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_vehicle_msgs")
pytest.importorskip("autoware_localization_msgs")

from autoware_vehicle_msgs.msg import VelocityReport  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402

from reflective_pose_autoware.initializer_node import (  # noqa: E402
    BoardPoseInitializer,
    State,
)


class StubClient:
    """Stands in for the InitializeLocalization client."""

    def __init__(self):
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        return True

    def call_async(self, request):
        self.requests.append(request)

        class Future:
            def add_done_callback(self, callback):
                pass

        return Future()


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    instance = BoardPoseInitializer()
    instance._params.min_attempt_spacing = 0.0
    instance._client = StubClient()
    yield instance
    instance.destroy_node()


def board_pose(node, stamp=None):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = (stamp or node.get_clock().now()).to_msg()
    message.pose.pose.position.x = 1.0
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance = [0.0] * 36
    message.pose.covariance[0] = 0.09
    return message


def velocity(speed):
    report = VelocityReport()
    report.longitudinal_velocity = speed
    return report


def test_moving_vehicle_does_not_spend_an_attempt(node):
    node._on_velocity(velocity(1.0))
    node._on_board_pose(board_pose(node))
    assert node._attempts == 0
    assert node._client.requests == []


def test_pose_accumulated_while_moving_is_dropped(node):
    """The old node cleared its accumulation when the vehicle moved.

    Accumulation is on the far side of the topic now, so the same protection is
    applied by stamp: a pose from before the last motion is not trusted.
    """
    node._on_velocity(velocity(1.0))
    stale = board_pose(node, rclpy.time.Time(seconds=1))
    node._on_velocity(velocity(0.0))
    node._on_board_pose(stale)
    assert node._attempts == 0
    assert node._client.requests == []


def test_stopped_vehicle_initializes_with_the_detector_pose(node):
    node._on_matcher_map(b"")
    node._on_velocity(velocity(0.0))
    node._on_board_pose(board_pose(node))

    assert node._attempts == 1
    (request,) = node._client.requests
    # AUTO, so NDT align refines the guess rather than taking it as truth.
    assert request.method == 0
    (sent,) = request.pose_with_covariance
    assert sent.header.frame_id == "map"
    assert sent.pose.pose.position.x == 1.0
    # The detector's covariance is forwarded, not recomputed here.
    assert sent.pose.covariance[0] == 0.09


def test_budget_is_spent_then_failure_is_terminal(node):
    node._params.max_attempts = 3
    node._params.fallback_to_user_defined_pose = False
    node._on_matcher_map(b"")

    for _ in range(3):
        node._on_board_pose(board_pose(node))
        node._attempt_failed("service rejected")

    assert node._attempts == 3
    assert node._state is State.FAILED
    # Terminal: further publications are ignored.
    node._on_board_pose(board_pose(node))
    assert node._attempts == 3


def test_fallback_sends_an_empty_request_when_enabled(node):
    node._params.max_attempts = 2
    node._params.fallback_to_user_defined_pose = True
    node._on_matcher_map(b"")

    for _ in range(2):
        node._on_board_pose(board_pose(node))
        node._attempt_failed("service rejected")

    assert node._state is State.DONE
    fallback = node._client.requests[-1]
    assert fallback.pose_with_covariance == []
    assert fallback.method == 0


def fail_attempts(node, count):
    for _ in range(count):
        node._on_board_pose(board_pose(node))
        node._attempt_failed("initialize service rejected")


def test_failures_before_the_matcher_has_a_map_are_free(node):
    """L8: a cart parked at the board while the stack boots.

    The detector fires about once a second from the first scan, and every
    align fails until the map reaches the matcher, 8.6 s after playback on
    2026-10-02. Those failures say nothing about the board, so they must not
    spend the budget: the call still goes out (it is what asks for the tiles),
    and the first attempt that counts is the first one after the map.
    """
    node._params.max_attempts = 3

    fail_attempts(node, 10)
    assert node._state is State.WAIT_POSE
    assert node._attempts == 0
    assert len(node._client.requests) == 10

    node._on_matcher_map(b"")
    fail_attempts(node, 3)
    assert node._attempts == 3
    assert node._state is State.FAILED


def test_a_failure_counts_by_what_was_known_when_the_call_went_out(node):
    """The tile reply can land between the call and its failed response.

    That align ran without a map, so it is free even though the map is known
    by the time the failure is read.
    """
    node._params.max_attempts = 1
    node._on_board_pose(board_pose(node))
    node._on_matcher_map(b"")
    node._attempt_failed("initialize service rejected")

    assert node._attempts == 0
    assert node._state is State.WAIT_POSE


def test_counting_from_the_first_failure_is_still_available(node):
    node._params.max_attempts = 2
    node._params.ignore_failures_before_map = False

    fail_attempts(node, 2)
    assert node._attempts == 2
    assert node._state is State.FAILED

def test_not_ready_stack_spends_no_attempt(node):
    """A pose before the wheel speed and NDT's cloud flow is not an attempt."""
    node._on_matcher_map(b"")
    node._params.require_velocity = True
    node._params.ready_pointcloud_topic = "/localization/util/downsample/pointcloud"
    node._on_board_pose(board_pose(node))
    assert node._attempts == 0 and node._client.requests == []
    assert "wheel speed" in node._last_reason

    node._params.stopped_for = 0.0      # the stop's duration has its own test
    node._on_velocity(velocity(0.0))
    node._on_board_pose(board_pose(node))
    assert node._attempts == 0 and node._client.requests == []
    assert "NDT" in node._last_reason

    node._on_points(None)
    node._on_board_pose(board_pose(node))
    assert node._attempts == 1 and len(node._client.requests) == 1


def test_stale_data_is_not_ready(node):
    node._params.require_velocity = True
    node._params.ready_timeout = 1.0
    node._last_velocity_time = node.get_clock().now() - rclpy.duration.Duration(seconds=5)
    node._on_board_pose(board_pose(node))
    assert node._attempts == 0


def test_readiness_is_off_by_default(node):
    """No vehicle interface, no cloud: the old behaviour, the gate open."""
    node._on_matcher_map(b"")
    node._on_board_pose(board_pose(node))
    assert node._attempts == 1


def test_stop_must_be_held_as_long_as_autoware_checks(node):
    """Autoware's stop check wants stop_check_duration (3 s) of stopped twists."""
    node._on_matcher_map(b"")
    node._params.require_velocity = True
    node._params.stopped_for = 3.0
    node._on_velocity(velocity(0.0))
    node._on_board_pose(board_pose(node))
    assert node._attempts == 0 and "stopped for" in node._last_reason
    node._stopped_since = node.get_clock().now() - rclpy.duration.Duration(seconds=3.5)
    node._on_board_pose(board_pose(node))
    assert node._attempts == 1


def test_motion_restarts_the_stop(node):
    node._params.require_velocity = True
    node._params.stopped_for = 3.0
    node._stopped_since = node.get_clock().now() - rclpy.duration.Duration(seconds=10)
    node._on_velocity(velocity(0.5))
    assert node._stopped_since is None
    node._on_velocity(velocity(0.0))
    node._on_board_pose(board_pose(node))
    assert node._attempts == 0


def test_attempts_are_spaced(node):
    node._params.min_attempt_spacing = 1.0
    node._params.require_velocity = False
    node._params.ready_pointcloud_topic = ""
    node._matcher_has_map = True
    msg = PoseWithCovarianceStamped()
    node._on_board_pose(msg)
    assert len(node._client.requests) == 1
    node._attempt_failed("rejected")
    assert node._state is State.WAIT_POSE
    node._on_board_pose(msg)
    assert len(node._client.requests) == 1  # inside min_attempt_spacing
    node._last_attempt_end = node.get_clock().now() - rclpy.duration.Duration(seconds=2)
    node._on_board_pose(msg)
    assert len(node._client.requests) == 2


def test_pose_wait_counts_from_the_last_pose_seen(node):
    node._pose_wait_timeout = 60.0
    node._last_pose_rx = node.get_clock().now()
    node._check_pose_wait()
    assert node._state is State.WAIT_POSE
    node._last_pose_rx = node.get_clock().now() - rclpy.duration.Duration(seconds=61)
    node._check_pose_wait()
    assert node._state is State.FAILED


def _ready(node):
    node._params.require_velocity = False
    node._params.ready_pointcloud_topic = ""
    node._matcher_has_map = True


def test_reinitialize_rearms_after_success_and_acts_on_the_latest_pose(node):
    from std_srvs.srv import Trigger

    _ready(node)
    msg = PoseWithCovarianceStamped()
    node._on_board_pose(msg)
    node._state = State.DONE  # the first initialization succeeded
    assert len(node._client.requests) == 1

    response = node._on_reinitialize(Trigger.Request(), Trigger.Response())
    assert response.success
    assert len(node._client.requests) == 2
    assert node._attempts == 1  # the budget restarted


def test_reinitialize_without_a_fresh_pose_arms_and_waits(node):
    from std_srvs.srv import Trigger

    _ready(node)
    node._state = State.FAILED
    response = node._on_reinitialize(Trigger.Request(), Trigger.Response())
    assert response.success and "armed" in response.message
    assert node._state is State.WAIT_POSE
    assert node._client.requests == []
    node._on_board_pose(PoseWithCovarianceStamped())  # the next pose is acted on
    assert len(node._client.requests) == 1


def test_reinitialize_refused_while_a_call_is_in_flight(node):
    from std_srvs.srv import Trigger

    _ready(node)
    node._on_board_pose(PoseWithCovarianceStamped())
    assert node._state is State.CALL_SERVICE
    response = node._on_reinitialize(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert len(node._client.requests) == 1
