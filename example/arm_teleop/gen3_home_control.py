"""Nonblocking handover to saved actions on the driver's existing Kortex session."""
import time

from example.operator_hold import HOME

COMMAND_TOPIC = "/gen3_vive/saved_action/command"
STATUS_TOPIC = "/gen3_vive/saved_action/status"


class HomeControl:
    def __init__(self, *, action_name, switch, publish, log=print):
        if not action_name.strip() or "\n" in action_name or len(action_name) > 128:
            raise ValueError("Invalid saved action name")
        self.action_name, self.switch, self.publish, self.log = action_name, switch, publish, log
        self.state = "idle"
        self.token = ""
        self.future = None
        self.deadline = 0.0
        self.error = None
        self.status = None
        self.status_time = None

    @property
    def teleop_allowed(self):
        return self.state == "idle"

    def on_status(self, wire, *, now=None):
        fields = wire.split("\n")
        if len(fields) == 2 and fields[0] == self.token and fields[1] in (
                "running", "complete", "stopped", "failed"):
            self.status = fields[1]
            self.status_time = time.monotonic() if now is None else now

    def _send(self, held, now, stamp=None):
        if self.token:
            self.publish(f"{'H' if held else 'S'}\n{self.token}\n"
                         f"{int((now if stamp is None else stamp) * 1e9)}\n{self.action_name}")

    def _switch(self, activate, now):
        self.future = self.switch(activate)
        if self.future is None:
            raise RuntimeError("Controller switch service unavailable")
        self.state = "restoring" if activate else "disabling"
        self.deadline = now + 3.0

    def tick(self, intent, *, controller_ready, now=None):
        now = time.monotonic() if now is None else now
        held = intent.fresh and intent.mode == HOME and intent.token == self.token
        try:
            if self.state == "idle":
                if intent.mode == HOME and intent.fresh and controller_ready and intent.token != self.token:
                    self.token, self.status, self.status_time = intent.token, None, None
                    self._switch(False, now)
            elif self.state in ("disabling", "restoring"):
                if self.future.done():
                    if not self.future.result().ok:
                        raise RuntimeError("Controller manager rejected the switch")
                    if self.state == "restoring":
                        self.state = "idle"
                    elif held:
                        self.state = "running"
                        self.deadline = now + 4.0  # first lookup may take two bounded RPCs
                    else:
                        self._switch(True, now)  # no action command was sent
                elif now > self.deadline:
                    raise RuntimeError("Controller switch timed out; restart the stack")
            elif self.state == "running":
                if not held:
                    self.state = "stopping"
                    self.status = None  # require a stop acknowledgement after release
                    self.deadline = now + 4.0
                elif self.status in ("failed", "stopped"):
                    self.state = "held"
                    self.log("Saved action stopped; release B before retrying")
                elif self.status == "complete":
                    self.state = "held"
                elif self.status_time is not None and now - self.status_time > 0.5:
                    raise RuntimeError("Saved action driver heartbeat lost; restart the stack")
                elif self.status is None and now > self.deadline:
                    raise RuntimeError("Saved action driver did not respond; rebuild/restart the stack")
            elif self.state == "held" and not held:
                self.state, self.status = "stopping", None
                self.deadline = now + 4.0
            elif self.state == "stopping":
                if self.status == "stopped" and self.status_time is not None and now-self.status_time < 0.5:
                    self._switch(True, now)
                elif now > self.deadline:
                    raise RuntimeError("Saved action stop was not acknowledged; restart the stack")
            if self.state in ("running", "held"):
                self._send(held, now, intent.monotonic_s if held else None)
            elif self.state in ("stopping", "fault"):
                self._send(False, now)
        except Exception as exc:
            self.state, self.error = "fault", str(exc)
            self.log(f"Home control stopped: {exc}")
            self._send(False, now)

    def close(self):
        self._send(False, time.monotonic())
        # Driver also expires the original operator lease in 100 ms. Never
        # reactivate Twist during shutdown or after a missing stop acknowledgement.
