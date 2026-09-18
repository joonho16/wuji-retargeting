"""Shared, expiring operator intent for the ROS arm and non-ROS hand processes.

The state directory is bind-mounted into both containers. Linux monotonic time
is shared by these containers; a stopped writer can never leave a held key on.
"""
from __future__ import annotations

from dataclasses import dataclass
import fcntl
import json
import math
import os
from pathlib import Path
import time
import uuid

STOP, TELEOP, HOME = "stop", "teleop", "home"
TIMEOUT = 0.1


@dataclass(frozen=True)
class Intent:
    mode: str = STOP
    token: str = ""
    fresh: bool = False
    monotonic_s: float | None = None


class HoldKeys:
    """Require released keys after startup, lost focus or a two-key conflict."""

    def __init__(self):
        self.armed = False
        self.previous = STOP

    def update(self, *, a: bool, b: bool, focused: bool) -> str:
        if not focused or (a and b):
            self.armed = False
            self.previous = STOP
            return STOP
        if not a and not b:
            self.armed = True
            self.previous = STOP
            return STOP
        mode = TELEOP if a else HOME
        if not self.armed or self.previous not in (STOP, mode):
            self.armed = False
            return STOP
        self.previous = mode
        return mode


class IntentWriter:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = self.path.with_suffix(".lock").open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise RuntimeError("An operator window is already running for this stack")
        self.session = uuid.uuid4().hex
        self.generation = 0
        self.mode = STOP
        self.publish(STOP)

    def publish(self, mode: str, *, now: float | None = None):
        if mode not in (STOP, TELEOP, HOME):
            raise ValueError("Unknown operator mode")
        if mode != self.mode:
            self.generation += 1
            self.mode = mode
        data = dict(version=1, mode=mode, session=self.session,
                    generation=self.generation, monotonic=time.monotonic() if now is None else now)
        temp = self.path.with_name(f".{self.path.name}.{self.session}.tmp")
        temp.write_text(json.dumps(data, allow_nan=False), encoding="utf-8")
        os.replace(temp, self.path)

    def close(self):
        try:
            self.publish(STOP)
        finally:
            self.lock.close()


def read_intent(path: Path, *, now: float | None = None) -> Intent:
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        stamp = data["monotonic"]
        mode, session, generation = data["mode"], data["session"], data["generation"]
        if (data["version"] != 1 or mode not in (STOP, TELEOP, HOME)
                or not isinstance(session, str) or len(session) != 32
                or type(generation) is not int or generation < 0
                or type(stamp) not in (int, float) or not math.isfinite(stamp)):
            return Intent()
        age = (time.monotonic() if now is None else now) - stamp
        if not 0 <= age <= TIMEOUT:
            return Intent()
        return Intent(mode, f"{session}:{generation}", True, float(stamp))
    except (OSError, ValueError, KeyError, TypeError):
        return Intent()


class IntentReader:
    """Do not resume held motion across stale input or process restarts."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.armed = False
        self.session = None

    def read(self, *, now: float | None = None) -> Intent:
        intent = read_intent(self.path, now=now)
        session = intent.token.split(":")[0]
        if not intent.fresh or session != self.session:
            self.armed = False
        self.session = session
        if intent.fresh and intent.mode == STOP:
            self.armed = True
        return intent if self.armed else Intent(fresh=intent.fresh)
