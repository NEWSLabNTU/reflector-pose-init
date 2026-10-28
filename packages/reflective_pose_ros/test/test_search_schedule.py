"""The search backs off in steps while no board is in view, and not while one is."""

from reflective_pose_ros.search_schedule import SearchSchedule

NEVER = float("inf")


def test_steps_with_no_board_ever_seen():
    s = SearchSchedule()
    got = [s.interval(m, NEVER) for m in (1, 4, 5, 7, 8, 10, 11, 13, 14, 100)]
    assert got == [0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 1.0, 1.0, 2.0, 2.0]


def test_cap():
    assert SearchSchedule(interval_max=0.5).interval(50, NEVER) == 0.5


def test_a_miss_just_after_a_detection_does_not_back_off():
    s = SearchSchedule()
    assert s.interval(20, 0.4) == 0.0
    assert s.interval(20, 1.5) == 2.0
