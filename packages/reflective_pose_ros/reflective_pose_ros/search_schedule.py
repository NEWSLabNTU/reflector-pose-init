"""How often the detector looks for the board while it sees none.

Pure arithmetic, no ROS: the node asks :meth:`SearchSchedule.interval` how long
to leave the sensor alone after the latest miss.

While the board is in view the node detects on every cloud and never backs off.
A miss right after a detection is not yet "no board": for ``lost_grace`` seconds
the schedule stays at full rate, so a flicker or one smeared batch does not slow
the search down. After that, consecutive misses step the interval up to
``interval_max``; any detection resets it.
"""

from dataclasses import dataclass

#: (misses at which the step starts, interval in seconds). The last step is
#: capped by ``interval_max``.
STEPS = ((14, 2.0), (11, 1.0), (8, 0.5), (5, 0.25))


@dataclass
class SearchSchedule:
    interval_max: float = 2.0
    lost_grace: float = 1.0

    def interval(self, misses: int, since_detection: float) -> float:
        """Seconds to wait before the next attempt; 0 means the next cloud.

        ``misses`` is the consecutive failed attempts since the last detection
        (or since start), ``since_detection`` the seconds since that detection;
        pass ``float("inf")`` when there has not been one.
        """
        if since_detection < self.lost_grace:
            return 0.0
        for start, interval in STEPS:
            if misses >= start:
                return min(interval, self.interval_max)
        return 0.0
