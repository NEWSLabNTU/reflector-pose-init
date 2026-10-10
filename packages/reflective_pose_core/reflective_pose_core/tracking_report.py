"""Offline evaluation of tracking mode on recorded scans. No ROS.

``board_tracking_node`` answers one question per scan: where is the board?
Before a field session the questions are different, and they are about the
detector file rather than about any one scan:

* **How often is the board found, by range?** Detection rate per range bin,
  and for the misses the gate that rejected the board.
* **What does it cost?** Detection time per scan, as the node spends it.
* **Where should the intensity gate be?** The intensity of the points on the
  board against everything else in the gated range and height band, so the
  threshold is set from data: the detector's ``intensity_threshold`` is the
  one number in the profile that is a property of the sensor and the sheeting
  rather than of the geometry, and both profiles ship it unmeasured.
* **How high is the board?** The detected centre above the ground, against
  the profile's centre-height gate -- the gate that decides whether a VLP-16
  sees a chest-height board at the lab's standoff at all.

Feed :class:`TrackingReport` one scan at a time (``add_scan``), exactly as the
node would see it: the cloud in the sensor frame, the static ``base <- sensor``
transform, the profile's runtime parameters, one scan per detection. A scan
counts as a detection when the node would have *published* it: a lone
survivor at or above ``min_confidence`` (``reflective_pose_ros.decision.judge``).

Range bins need the board's range in every scan, including the scans where it
was missed. A simulator knows it (``truth_range``). A recording of a board held
at several static distances does not, so a miss is assigned the range of the
nearest accepted detection within ``assign_window`` seconds -- right for a
board that stands still, which is how the recording is meant to be made -- and
a miss with no detection that close is counted under ``unassigned``.

Board points are taken from the accepted detection's fitted rectangle, shrunk
by ``edge_margin`` so edge returns and blooming do not blur the board's side of
the histogram, and including every point in it whatever its intensity: the
cluster itself already passed the current threshold, and judging a threshold
by the points it let through would be circular. Background points are all the
other points inside the range gate, at any height, farther than
``clearance`` from the board.
"""

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .config import Config
from .detector import DetectResult, Status, detect_board

#: Intensity histogram resolution: one bin per unit over [0, HIST_MAX).
HIST_MAX = 256


@dataclass
class ScanResult:
    t: float
    accepted: bool
    outcome: str                 # 'detected', 'low_confidence', 'ambiguous', or a rejection
    detect_ms: float
    range_m: float = float("nan")         # horizontal, base_link origin to board centre
    bearing_deg: float = float("nan")
    centre_height: float = float("nan")   # above ground
    n_points: int = 0
    confidence: float = 0.0
    truth_range: Optional[float] = None
    assigned_range: Optional[float] = None


@dataclass
class BinStats:
    lo: float
    hi: float
    scans: int = 0
    detected: int = 0
    heights: List[float] = field(default_factory=list)
    ranges: List[float] = field(default_factory=list)
    outcomes: Dict[str, int] = field(default_factory=dict)

    @property
    def rate(self) -> float:
        return self.detected / self.scans if self.scans else float("nan")


def outcome_of(result: DetectResult, min_confidence: float) -> Tuple[bool, str]:
    """(published, why) -- the node's verdict, without the ROS message types."""
    if result.status is Status.AMBIGUOUS:
        return False, "ambiguous"
    if result.status is not Status.OK:
        if result.rejections:
            counts: Dict[str, int] = {}
            for r in result.rejections:
                counts[r.reason] = counts.get(r.reason, 0) + 1
            return False, max(counts, key=counts.get)
        return False, "too_few_points" if result.n_after_gates else "no_retro_points"
    if result.detection.confidence < min_confidence:
        return False, "low_confidence"
    return True, "detected"


def _hist(values: np.ndarray) -> np.ndarray:
    idx = np.clip(np.floor(values).astype(np.int64), 0, HIST_MAX - 1)
    return np.bincount(idx, minlength=HIST_MAX)


def percentile_from_hist(hist: np.ndarray, q: float) -> float:
    """The q-th percentile (0..100) of a unit-bin histogram, bin lower edges."""
    total = hist.sum()
    if total == 0:
        return float("nan")
    cdf = np.cumsum(hist) / total
    return float(np.searchsorted(cdf, q / 100.0))


class TrackingReport:
    """Accumulates per-scan results and intensity histograms."""

    def __init__(
        self,
        config: Config,
        transform_base_sensor: np.ndarray,
        *,
        min_confidence: Optional[float] = None,
        edge_margin: float = 0.05,
        clearance: float = 0.3,
        assign_window: float = 1.0,
    ):
        self.config = config
        self.params = config.runtime_detector
        self.height_offset = float(config.detector.runtime.base_link_height_above_ground)
        self.transform = np.asarray(transform_base_sensor, dtype=np.float64)
        self.min_confidence = (
            float(self.params.min_confidence) if min_confidence is None else float(min_confidence)
        )
        self.edge_margin = edge_margin
        self.clearance = clearance
        self.assign_window = assign_window
        self.scans: List[ScanResult] = []
        self.board_hist = np.zeros(HIST_MAX, dtype=np.int64)
        self.background_hist = np.zeros(HIST_MAX, dtype=np.int64)
        self.background_retro_per_scan: List[int] = []

    # -- one scan -------------------------------------------------------------

    def add_scan(
        self,
        t: float,
        points: np.ndarray,
        intensity: np.ndarray,
        truth_range: Optional[float] = None,
    ) -> ScanResult:
        points = np.asarray(points, dtype=np.float64)
        intensity = np.asarray(intensity, dtype=np.float64)
        started = time.perf_counter()
        result = detect_board(
            points, intensity, self.transform, self.params, height_offset=self.height_offset
        )
        detect_ms = 1e3 * (time.perf_counter() - started)
        accepted, why = outcome_of(result, self.min_confidence)
        scan = ScanResult(t=float(t), accepted=accepted, outcome=why, detect_ms=detect_ms,
                          truth_range=truth_range)
        detection = result.detection if result.status is Status.OK else None
        if detection is not None:
            centre = self.transform[:3, :3] @ detection.centre + self.transform[:3, 3]
            scan.range_m = float(np.hypot(centre[0], centre[1]))
            scan.bearing_deg = float(np.degrees(np.arctan2(centre[1], centre[0])))
            scan.centre_height = float(centre[2] + self.height_offset)
            scan.n_points = int(detection.n_points)
            scan.confidence = float(detection.confidence)
        self._intensities(points, intensity, detection if accepted else None)
        self.scans.append(scan)
        return scan

    def _intensities(self, points, intensity, detection):
        p = self.params
        ranges = np.linalg.norm(points, axis=1)
        # Background is everything in the range gate, at any height: the
        # height band alone can hold nothing but other retroreflectors in an
        # open hall, and the floor and walls are the diffuse surfaces the
        # threshold exists to reject.
        gated = (ranges >= p.range_min) & (ranges <= p.range_max)
        background = gated
        if detection is not None:
            local = points - detection.centre
            u = local @ detection.right
            v = local @ detection.up
            w = local @ detection.normal
            half_w = 0.5 * self.config.board.width
            half_h = 0.5 * self.config.board.height
            thick = max(p.planarity_max_thickness, 0.05)
            on_board = (
                (np.abs(w) <= thick)
                & (np.abs(u) <= half_w - self.edge_margin)
                & (np.abs(v) <= half_h - self.edge_margin)
            )
            near = (
                (np.abs(w) <= thick + self.clearance)
                & (np.abs(u) <= half_w + self.clearance)
                & (np.abs(v) <= half_h + self.clearance)
            )
            self.board_hist += _hist(intensity[on_board])
            background = gated & ~near
            self.background_hist += _hist(intensity[background])
            self.background_retro_per_scan.append(
                int(np.count_nonzero(intensity[background] >= p.intensity_threshold)))
        # Without an accepted detection the board, if present, is somewhere in
        # the gated points, so they cannot be called background.

    # -- summaries -------------------------------------------------------------

    def assign_ranges(self) -> None:
        """Give every scan a range: the truth, else the nearest accepted
        detection within assign_window seconds."""
        hits = [(s.t, s.range_m) for s in self.scans if s.accepted]
        ht = np.array([h[0] for h in hits])
        hr = np.array([h[1] for h in hits])
        for s in self.scans:
            if s.truth_range is not None:
                s.assigned_range = s.truth_range
            elif s.accepted:
                s.assigned_range = s.range_m
            elif len(ht):
                k = int(np.argmin(np.abs(ht - s.t)))
                s.assigned_range = float(hr[k]) if abs(ht[k] - s.t) <= self.assign_window else None
            else:
                s.assigned_range = None

    def bins(self, edges: Sequence[float]) -> Tuple[List[BinStats], BinStats]:
        self.assign_ranges()
        out = [BinStats(lo, hi) for lo, hi in zip(edges[:-1], edges[1:])]
        unassigned = BinStats(float("nan"), float("nan"))
        for s in self.scans:
            target = unassigned
            if s.assigned_range is not None:
                for b in out:
                    if b.lo <= s.assigned_range < b.hi:
                        target = b
                        break
            target.scans += 1
            target.outcomes[s.outcome] = target.outcomes.get(s.outcome, 0) + 1
            if s.accepted:
                target.detected += 1
                target.heights.append(s.centre_height)
                target.ranges.append(s.range_m)
        return out, unassigned

    def suggest_threshold(self) -> Dict[str, float]:
        """A threshold from the two histograms, and what it and the current one do."""
        bh, gh = self.board_hist, self.background_hist
        out: Dict[str, float] = {
            "current": float(self.params.intensity_threshold),
            "board_points": float(bh.sum()),
            "background_points": float(gh.sum()),
        }
        if bh.sum() == 0:
            out["suggested"] = float("nan")
            out["separable"] = float("nan")
            return out
        b5 = percentile_from_hist(bh, 5.0)
        b1 = percentile_from_hist(bh, 1.0)
        has_bg = gh.sum() > 0
        out.update(board_p1=b1, board_p5=b5, board_p50=percentile_from_hist(bh, 50.0),
                   background_p99=percentile_from_hist(gh, 99.0) if has_bg else float("nan"),
                   background_p999=percentile_from_hist(gh, 99.9) if has_bg else float("nan"),
                   background_max=float(np.flatnonzero(gh).max()) if has_bg else float("nan"))
        # Youden's J over every threshold: the fraction of board points kept
        # minus the fraction of background passed. Its plateau is the gap
        # between the two populations; the suggestion is the plateau's middle,
        # as far from both as the data allow. Other retroreflectors in the
        # background (a vest, a sign) sit above any useful threshold and only
        # lower the plateau, they do not move it: only geometry rejects them.
        kept = np.cumsum(bh[::-1])[::-1] / bh.sum()
        passed = (np.cumsum(gh[::-1])[::-1] / gh.sum()) if has_bg else np.zeros(HIST_MAX)
        j = kept - passed
        good = j >= j.max() - 0.001
        runs, start = [], None
        for i, g in enumerate(np.append(good, False)):
            if g and start is None:
                start = i
            elif not g and start is not None:
                runs.append((start, i - 1))
                start = None
        lo, hi = max(runs, key=lambda r: (r[1] - r[0], -r[0]))
        out["suggested"] = float(round(0.5 * (lo + hi)))
        out["plateau_lo"], out["plateau_hi"] = float(lo), float(hi)
        k = int(out["suggested"])
        out["separable"] = float(kept[k] >= 0.99 and passed[k] <= 0.001)
        out["retro_background"] = float(passed[k])

        def frac_at_or_above(hist, thr):
            if not hist.sum():
                return float("nan")
            return float(hist[int(np.ceil(thr)):].sum() / hist.sum())

        for name in ("current", "suggested"):
            thr = out[name]
            out[f"{name}_board_kept"] = frac_at_or_above(bh, thr)
            out[f"{name}_background_passed"] = frac_at_or_above(gh, thr)
        return out

    def latency(self) -> Dict[str, float]:
        ms = np.array([s.detect_ms for s in self.scans])
        if len(ms) == 0:
            return {}
        return dict(mean=float(ms.mean()), p50=float(np.percentile(ms, 50)),
                    p95=float(np.percentile(ms, 95)), max=float(ms.max()), scans=len(ms))

    def format(self, edges: Sequence[float], extra_latency: Optional[Dict[str, float]] = None,
               label: str = "") -> str:
        bins, unassigned = self.bins(edges)
        p = self.params
        lines = []
        if label:
            lines.append(label)
        n = len(self.scans)
        hits = sum(s.accepted for s in self.scans)
        lines.append(f"scans {n}, detections {hits} ({100.0 * hits / n:.0f} %)"
                     if n else "no scans")
        lines.append("")
        lines.append(f"{'range bin':<12}{'scans':>6}{'det':>5}{'rate':>7}{'range':>8}"
                     f"{'centre h':>13}  misses")
        for b in bins + ([unassigned] if unassigned.scans else []):
            if not b.scans:
                continue
            name = "unassigned" if np.isnan(b.lo) else f"{b.lo:.1f}-{b.hi:.1f} m"
            rng = f"{np.mean(b.ranges):.2f}" if b.ranges else "-"
            h = (f"{np.mean(b.heights):.2f}+-{np.std(b.heights):.2f}" if b.heights else "-")
            misses = {k: v for k, v in b.outcomes.items() if k != "detected"}
            miss = ", ".join(f"{k} {v}" for k, v in sorted(misses.items(), key=lambda kv: -kv[1]))
            lines.append(f"{name:<13}{b.scans:>5}{b.detected:>5}{100 * b.rate:>6.0f}%"
                         f"{rng:>8}{h:>13}  {miss}")
        lines.append("")
        heights = [s.centre_height for s in self.scans if s.accepted]
        gate_lo = p.board_centre_height - p.centre_height_tolerance
        gate_hi = p.board_centre_height + p.centre_height_tolerance
        if heights:
            lines.append(
                f"board centre height  {np.mean(heights):.2f} m mean, "
                f"{np.min(heights):.2f}..{np.max(heights):.2f} m "
                f"(gate {gate_lo:.2f}..{gate_hi:.2f} m)")
        lat = self.latency()
        if lat:
            lines.append(
                f"detect time          {lat['mean']:.1f} ms mean, {lat['p50']:.1f} p50, "
                f"{lat['p95']:.1f} p95, {lat['max']:.1f} max per scan")
        for name, value in (extra_latency or {}).items():
            lines.append(f"{name:<21}{value}")
        lines.append("")
        th = self.suggest_threshold()
        if th["board_points"] == 0:
            lines.append("intensity: no accepted detection, so no board points to measure")
        else:
            lines.append(
                f"intensity, board      {int(th['board_points'])} points: p1 {th['board_p1']:.0f}, "
                f"p5 {th['board_p5']:.0f}, median {th['board_p50']:.0f}")
            lines.append(
                f"intensity, background {int(th['background_points'])} points: "
                f"p99 {th['background_p99']:.0f}, p99.9 {th['background_p999']:.0f}, "
                f"max {th['background_max']:.0f}")
            retro = self.background_retro_per_scan
            if retro:
                lines.append(
                    f"background points at/above the current threshold: "
                    f"{np.mean(retro):.1f} per scan (max {max(retro)})")
            lines.append(
                f"threshold now        {th['current']:.0f}: keeps "
                f"{100 * th['current_board_kept']:.1f} % of board points, passes "
                f"{100 * th['current_background_passed']:.3f} % of background")
            verdict = (f"middle of the gap {th['plateau_lo']:.0f}..{th['plateau_hi']:.0f}"
                       + ("" if th["separable"] else
                          "; what background still passes is retroreflective, and only "
                          "the geometric gates reject it"))
            lines.append(
                f"suggested            {th['suggested']:.0f}: keeps "
                f"{100 * th['suggested_board_kept']:.1f} % of board points, passes "
                f"{100 * th['suggested_background_passed']:.3f} % of background ({verdict})")
        return "\n".join(lines)


def default_bins(range_max: float, width: float = 1.0) -> List[float]:
    """Bins one `width` wide centred on whole metres: a board held at 2, 4,
    6, 8 m lands in the middle of one."""
    edges = [0.0, 0.5 * width]
    while edges[-1] < range_max + width:
        edges.append(edges[-1] + width)
    return edges
