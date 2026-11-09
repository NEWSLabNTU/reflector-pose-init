"""The AD API entry point: the RViz "Initialize with GNSS" button in board mode.

Autoware's RViz panel calls /api/localization/initialize with an empty pose
list. With ``adapi_initialize_service`` set, this node serves that name: an
empty request re-runs the board initialization and answers with its outcome,
and a request carrying a pose is forwarded unchanged to
``adapi_forward_service`` (default_adapi's server, moved there by the launch).

Over real ROS transport on a domain of its own, as the node runs on the
vehicle (multi-threaded executor). The pose initializer, the detector and
default_adapi are stubs. Needs rclpy and the Autoware messages. Spins.
"""

import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_vehicle_msgs")
pytest.importorskip("autoware_localization_msgs")
pytest.importorskip("autoware_adapi_v1_msgs")

from autoware_adapi_v1_msgs.srv import InitializeLocalization as AdapiInit  # noqa: E402
from autoware_localization_msgs.srv import InitializeLocalization  # noqa: E402
from autoware_vehicle_msgs.msg import VelocityReport  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile  # noqa: E402

from reflective_pose_autoware.initializer_node import (  # noqa: E402
    DEFAULT_BOARD_POSE_TOPIC,
    VELOCITY_TOPIC,
    BoardPoseInitializer,
    State,
)

API = "/api/localization/initialize"
FORWARD = "/api/localization/initialize_default"


class FakeStack:
    """Autoware's pose initializer, default_adapi and the detector, as stubs."""

    def __init__(self, node):
        self.node = node
        self.initialize_calls = []
        self.default_calls = []
        self.accept = True
        node.create_service(InitializeLocalization, "/localization/initialize", self.on_initialize)
        node.create_service(AdapiInit, FORWARD, self.on_default)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.detector = node.create_publisher(PoseWithCovarianceStamped, DEFAULT_BOARD_POSE_TOPIC, latched)
        self.velocity = node.create_publisher(VelocityReport, VELOCITY_TOPIC, 10)

    def on_initialize(self, request, response):
        self.initialize_calls.append(request)
        response.status.success = self.accept
        response.status.message = "" if self.accept else "NDT align server failed"
        return response

    def on_default(self, request, response):
        self.default_calls.append(request)
        response.status.success = True
        response.status.message = "default_adapi stub"
        return response

    def detect(self, x=3.0):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.pose.position.x = x
        pose.pose.pose.orientation.w = 1.0
        self.detector.publish(pose)


@pytest.fixture
def system():
    domain = 100 + (os.getpid() + 37) % 100
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=domain)
    initializer = BoardPoseInitializer(
        context=ctx,
        parameter_overrides=[
            Parameter("adapi_initialize_service", value=API),
            Parameter("adapi_forward_service", value=FORWARD),
            Parameter("adapi_board_timeout", value=3.0),
            Parameter("min_attempt_spacing", value=0.0),
            Parameter("ignore_failures_before_map", value=False),
        ],
    )
    stack_node = rclpy.create_node("fake_stack", context=ctx)
    stack = FakeStack(stack_node)
    caller = rclpy.create_node("rviz_panel", context=ctx)
    client = caller.create_client(AdapiInit, API)

    executors = [MultiThreadedExecutor(context=ctx), MultiThreadedExecutor(context=ctx)]
    executors[0].add_node(initializer)
    executors[1].add_node(stack_node)
    executors[1].add_node(caller)
    threads = [threading.Thread(target=e.spin, daemon=True) for e in executors]
    for thread in threads:
        thread.start()
    assert client.wait_for_service(timeout_sec=5.0)

    def call(request, timeout=10.0):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        assert done.wait(timeout), "no response"
        return future.result()

    yield initializer, stack, call
    for executor in executors:
        executor.shutdown()
    initializer.destroy_node()
    stack_node.destroy_node()
    caller.destroy_node()
    rclpy.shutdown(context=ctx)


def wait_for(predicate, timeout=5.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def test_empty_request_reruns_the_board_initialization(system):
    initializer, stack, call = system
    stack.detect()
    assert wait_for(lambda: initializer._state is State.DONE)
    assert len(stack.initialize_calls) == 1

    # The button, after a first initialization: initializes again from the board.
    response = call(AdapiInit.Request())
    assert response.status.success, response.status.message
    assert response.status.message == "initialized from the board"
    assert len(stack.initialize_calls) == 2
    assert stack.initialize_calls[-1].pose_with_covariance[0].pose.pose.position.x == 3.0
    assert stack.default_calls == []  # default_adapi was not asked


def test_empty_request_waits_for_the_next_board_pose(system):
    initializer, stack, call = system
    # No board seen yet: the response comes once a detection lands.
    threading.Timer(1.0, stack.detect, kwargs={"x": 7.0}).start()
    start = time.monotonic()
    response = call(AdapiInit.Request())
    assert response.status.success, response.status.message
    assert time.monotonic() - start >= 0.9
    assert stack.initialize_calls[-1].pose_with_covariance[0].pose.pose.position.x == 7.0


def test_empty_request_without_a_board_times_out_with_an_error(system):
    initializer, stack, call = system
    response = call(AdapiInit.Request())
    assert not response.status.success
    assert response.status.code == AdapiInit.Response.ERROR_ESTIMATION
    assert "no board pose" in response.status.message
    assert stack.initialize_calls == []
    # Still armed: a later detection initializes.
    stack.detect()
    assert wait_for(lambda: initializer._state is State.DONE)


def test_empty_request_while_moving_reports_unsafe(system):
    initializer, stack, call = system
    report = VelocityReport()
    report.longitudinal_velocity = 1.0
    for _ in range(5):
        stack.velocity.publish(report)
        time.sleep(0.05)
    assert wait_for(lambda: initializer._speed > 0.5)
    stack.detect()
    response = call(AdapiInit.Request())
    assert not response.status.success
    assert response.status.code == AdapiInit.Response.ERROR_UNSAFE
    assert "moving" in response.status.message
    assert stack.initialize_calls == []


def test_request_with_a_pose_is_forwarded_unchanged(system):
    initializer, stack, call = system
    request = AdapiInit.Request()
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = 42.0
    pose.pose.pose.orientation.w = 1.0
    request.pose = [pose]

    response = call(request)
    assert response.status.success
    assert response.status.message == "default_adapi stub"
    assert len(stack.default_calls) == 1
    assert stack.default_calls[0].pose[0].pose.pose.position.x == 42.0
    assert stack.initialize_calls == []  # the board path was not touched
    assert initializer._state is State.WAIT_POSE


def test_not_served_by_default():
    ctx = rclpy.context.Context()
    rclpy.init(context=ctx, domain_id=100 + (os.getpid() + 71) % 100)
    node = BoardPoseInitializer(context=ctx)
    try:
        assert node._params.adapi_initialize_service == ""
        assert node._adapi_forward is None
        names = [name for name, _types in node.get_service_names_and_types_by_node(
            "board_pose_initializer", "/")]
        assert API not in names
    finally:
        node.destroy_node()
        rclpy.shutdown(context=ctx)
