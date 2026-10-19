"""Single-owner, fail-closed follower hardware session."""

import logging
import threading
import time

from cf_robot.hardware import joint_positions
from cf_robot.protocol import ProtocolError, validate_action

LOG = logging.getLogger(__name__)
MAX_STEP_DEG = 5.0
WATCHDOG_S = 0.5


class FollowerSession:
    """One hardware-owning thread; the network loop only replaces its latest target."""

    def __init__(self, factory, max_step=MAX_STEP_DEG, watchdog=WATCHDOG_S):
        self.factory = factory
        self.max_step = max_step
        self.watchdog = watchdog
        self.ready = threading.Event()
        self.stopped = threading.Event()
        self.wakeup = threading.Event()
        self.lock = threading.Lock()
        self.state = "connecting"
        self.error = None
        self.initial_observation = None
        self.target = None
        self.received_at = 0.0
        self.thread = threading.Thread(target=self._run, name="follower-control", daemon=True)

    def start(self):
        self.thread.start()

    def wait_ready(self, timeout=15):
        return self.ready.wait(timeout) and self.state == "ready"

    def submit(self, action):
        with self.lock:
            if self.state != "ready":
                raise ProtocolError(self.error or "follower is not ready")
            self.target = action
            self.received_at = time.monotonic()
            self.wakeup.set()

    def stop(self):
        self.stopped.set()
        self.wakeup.set()
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            LOG.error("Hardware call has not returned; refusing another operator until it exits")
            self.thread.join()

    def _run(self):
        robot = None
        try:
            robot = self.factory()
            # Calibration is performed explicitly before running the service.
            robot.connect(calibrate=False)
            previous = validate_action(joint_positions(robot.get_observation()))
            self.initial_observation = dict(previous)
            with self.lock:
                self.state = "ready"
            self.ready.set()
            armed = False
            while not self.stopped.is_set():
                self.wakeup.wait(timeout=0.02)
                self.wakeup.clear()
                with self.lock:
                    target, received_at = self.target, self.received_at
                    self.target = None
                if self.stopped.is_set():
                    break
                if armed and time.monotonic() - received_at > self.watchdog:
                    raise ProtocolError("control stream timed out; reconnect and align arms")
                if target is None:
                    continue
                if time.monotonic() - received_at > self.watchdog:
                    raise ProtocolError("queued command expired")
                jumps = {
                    name: target[name] - previous[name]
                    for name in previous
                    if abs(target[name] - previous[name]) > self.max_step
                }
                if jumps:
                    baseline_label = "follower" if not armed else "previous leader"
                    next_step = (
                        "align arms before starting"
                        if not armed
                        else "reduce per-sample movement and retry"
                    )
                    details = "\n".join(
                        f"  - {name}: {baseline_label}={previous[name]:.2f}, "
                        f"leader={target[name]:.2f}, delta={delta:+.2f}"
                        for name, delta in jumps.items()
                    )
                    raise ProtocolError(
                        f"joint jump exceeds {self.max_step:g}-degree limit:\n"
                        f"{details}\n"
                        f"{next_step}"
                    )
                robot.send_action(target)
                previous = target
                armed = True
        except Exception as exc:
            if isinstance(exc, ProtocolError):
                LOG.warning("Follower session stopped: %s", exc)
            else:
                LOG.exception("Follower hardware failed")
            with self.lock:
                self.error = str(exc)
                self.state = "error"
            self.ready.set()
        finally:
            if robot is not None:
                try:
                    robot.disconnect()
                except Exception:
                    LOG.exception("Could not disconnect follower")
            with self.lock:
                if self.state != "error":
                    self.state = "stopped"
            self.ready.set()
