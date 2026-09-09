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
pytest.importorskip("tier4_localization_msgs")

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

    for _ in range(2):
        node._on_board_pose(board_pose(node))
        node._attempt_failed("service rejected")

    assert node._state is State.DONE
    fallback = node._client.requests[-1]
    assert fallback.pose_with_covariance == []
    assert fallback.method == 0
