"""WH110 output worker with a shared A/B hold and measured-position stop.

All SDK access belongs to this worker, independently of MANUS/retargeting. The
normal standalone hand backend is unchanged; full-real selects this backend.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import queue
import threading
import time

import numpy as np
import yaml

from example.operator_hold import HOME, TELEOP, IntentReader, TIMEOUT

TELEOP_SOFT_START_S = 0.5


def load_open_pose(path, side):
    data = yaml.safe_load(Path(path).read_text())
    if data.get("hand_model") != "wh110" or data.get("units") != "rad":
        raise ValueError("Open pose must declare hand_model: wh110 and units: rad")
    target = np.asarray(data[side], dtype=float)
    if target.shape != (5, 4) or not np.isfinite(target).all():
        raise ValueError("Open pose must contain finite 5 x 4 WH110 joint angles")
    return target


@dataclass
class HandTarget:
    position: np.ndarray
    sampled_at: float
    token: str


class HandMotion:
    """Stop captures feedback, never the last distant target or filter state."""
    def __init__(self, opened, lower, upper, *, open_speed=0.6):
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        self.opened = np.asarray(opened)
        if (self.lower.shape != (5, 4) or self.upper.shape != (5, 4)
                or not np.isfinite(self.lower).all() or not np.isfinite(self.upper).all()
                or np.any(self.lower >= self.upper)):
            raise ValueError("Invalid device joint limits")
        if np.any(self.opened < self.lower) or np.any(self.opened > self.upper):
            raise ValueError("Open-hand pose exceeds this device's joint limits")
        self.open_speed = open_speed
        self.output = None
        self.active = False
        self.token = ""
        self.last_time = None
        self.teleop_started_at = None
        self.teleop_start_pose = None
        self.teleop_filtered_goal = None

    def step(self, intent, actual, packet, *, now):
        actual = np.asarray(actual, dtype=float)
        if actual.shape != (5, 4) or not np.isfinite(actual).all():
            raise ValueError("Invalid WH110 feedback")
        dt = min(0.04, max(0.0, now - self.last_time)) if self.last_time is not None else 0.0
        self.last_time = now
        goal = None
        if intent.fresh and intent.mode == HOME:
            goal = self.opened
        elif (intent.fresh and intent.mode == TELEOP and packet is not None
              and packet.token == intent.token and 0 <= now-packet.sampled_at <= TIMEOUT):
            q = np.asarray(packet.position, dtype=float)
            if q.shape == (5, 4) and np.isfinite(q).all():
                goal = np.clip(q, self.lower, self.upper)
        if goal is None:
            if self.active or self.output is None:
                self.output = actual.copy()
            self.active = False
            self.teleop_started_at = None
        else:
            starting = not self.active or self.token != intent.token
            if starting:
                self.output = actual.copy()
            if intent.mode == HOME:
                self.teleop_started_at = None
                self.output += np.clip(goal-self.output, -self.open_speed*dt, self.open_speed*dt)
            else:
                # Start when the first valid target arrives, not while waiting
                # for MANUS. A new hold or recovery starts from measured pose.
                if starting or self.teleop_started_at is None:
                    self.teleop_started_at = now
                    self.teleop_start_pose = actual.copy()
                    self.teleop_filtered_goal = actual.copy()
                # Preserve the existing 5 Hz target filter independently of
                # the startup blend so it joins normal following continuously.
                alpha = 1.0 - math.exp(-2*math.pi*5.0*dt)
                self.teleop_filtered_goal += alpha * (goal-self.teleop_filtered_goal)
                progress = min(1.0, max(0.0, (now-self.teleop_started_at)/TELEOP_SOFT_START_S))
                # Quintic easing has zero slope/curvature at both ends.
                blend = progress**3 * (10.0 + progress * (-15.0 + 6.0*progress))
                self.output = self.teleop_start_pose + blend * (self.teleop_filtered_goal-self.teleop_start_pose)
            self.active = True
        self.token = intent.token
        return self.output.copy(), self.active


class HeldWh110Backend:
    def __init__(self, *, state_path, open_pose, hand_serial="", hand_factory=None):
        self.state_path = Path(state_path)
        self.open_pose = np.asarray(open_pose, dtype=float)
        self.serial = hand_serial
        self.hand_factory = hand_factory
        self.targets = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, name="wh110-hold-output", daemon=True)
        self.thread.start()
        if not self.ready.wait(4.0):
            self.stopping.set()
            raise RuntimeError("WH110 hold worker initialization timed out")
        self.check()

    def check(self):
        if self.error is not None:
            raise RuntimeError(f"WH110 hold worker stopped: {self.error}") from self.error

    def send(self, qpos, *, sampled_at, token):
        self.check()
        packet = HandTarget(np.asarray(qpos, dtype=float).reshape(5, 4).copy(), sampled_at, token)
        try:
            self.targets.get_nowait()
        except queue.Empty:
            pass
        self.targets.put_nowait(packet)

    def _run(self):
        hand = controller = None
        try:
            import wujihandpy
            factory = self.hand_factory or wujihandpy.Hand
            hand = factory(serial_number=self.serial) if self.serial else factory()
            hand.write_joint_enabled(False)
            actual = hand.read_joint_actual_position(timeout=0.2)
            motion = HandMotion(self.open_pose, hand.read_joint_lower_limit(timeout=0.2),
                                hand.read_joint_upper_limit(timeout=0.2))
            # >= 1 kHz disables firmware low-pass: a release can replace the
            # command immediately; the cancellable 5 Hz filter is above.
            controller = hand.realtime_controller(
                enable_upstream=True, filter=wujihandpy.filter.LowPass(cutoff_freq=1000.0))
            controller.set_joint_target_position(actual)
            reader = IntentReader(self.state_path)
            packet, enabled = None, False
            self.ready.set()
            while not self.stopping.is_set():
                started = time.monotonic()
                try:
                    packet = self.targets.get_nowait()
                except queue.Empty:
                    pass
                # Synchronous, bounded feedback detects disconnects. The SDK's
                # cached realtime getter has no age or disconnect indication.
                actual = hand.read_joint_actual_position(timeout=0.05)
                now = time.monotonic()
                intent = reader.read(now=now)
                target, active = motion.step(intent, actual, packet, now=now)
                controller.set_joint_target_position(target)
                if active and not enabled:
                    # Seed target from actual feedback before energizing.
                    hand.write_joint_enabled(True, timeout=0.05)
                    enabled = True
                self.stopping.wait(max(0.0, 0.02-(time.monotonic()-started)))
        except BaseException as exc:
            self.error = exc
        finally:
            self.ready.set()
            if hand is not None:
                try:
                    hand.write_joint_enabled(False)
                finally:
                    if controller is not None:
                        controller.close()

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            raise RuntimeError("WH110 output worker did not finish shutdown")
