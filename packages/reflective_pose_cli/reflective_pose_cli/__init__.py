"""Terminal tools for the reflective board detector. No ROS, by design.

The one command here anchors a finished SLAM cloud to the board. Its failure
path used to open RViz through a lazy ``import rclpy``, which meant the package
that a mapping person runs on a laptop depended on a vehicle stack. It now
writes a ``.npz`` instead, and ``reflective_pose_ros`` draws it. Nothing under
this package may import ``rclpy`` or a message package, at module level or
inside a function.
"""
