"""Unit-aware bounded goals and measured-progress checks for any selected joints."""

from time import monotonic

from cf_robot.protocol import JOINTS, validate_action

MAX_GOAL_LEAD = 5.0
SETTLE_TOLERANCE = 1.0
MIN_PROGRESS = 0.25
NO_PROGRESS_TIMEOUT_S = 1.0


def joint_label(joint):
    unit = "percentage points" if joint == "gripper.pos" else "degrees"
    return f"J{JOINTS.index(joint) + 1} {joint} ({unit})"


class BoundedMotion:
    """Permit sustained catch-up while moving; timeout only on absent progress."""

    def __init__(self, joints=JOINTS, clock=None):
        self.joints = joints
        self.clock = clock or monotonic
        self.last_goals = None
        self.progress = {}

    def plan(self, target, measured):
        target = validate_action(target, self.joints)
        measured = validate_action(measured, self.joints)
        now = self.clock()
        applied = {}
        for joint in self.joints:
            desired, actual = target[joint], measured[joint]
            if self.last_goals is None and abs(desired - actual) > MAX_GOAL_LEAD:
                raise ValueError(
                    f"{joint_label(joint)} startup alignment exceeds {MAX_GOAL_LEAD:g}: "
                    f"leader={desired:.2f}, follower={actual:.2f}"
                )
            previous = None if self.last_goals is None else self.last_goals[joint]
            direction = 1 if desired > actual else -1
            pending = (
                previous is not None and abs(previous - actual) > SETTLE_TOLERANCE
                and (previous - actual) * (desired - actual) > 0
                and abs(desired - actual) > SETTLE_TOLERANCE
            )
            reference = self.progress.get(joint)
            if not pending or reference is None or reference["direction"] != direction:
                self.progress[joint] = {"position": actual, "at": now, "direction": direction}
            elif (actual - reference["position"]) * direction >= MIN_PROGRESS:
                reference.update(position=actual, at=now)
            elif now - reference["at"] >= NO_PROGRESS_TIMEOUT_S:
                raise RuntimeError(
                    f"{joint_label(joint)} no measured progress for {NO_PROGRESS_TIMEOUT_S:g}s: "
                    f"leader={desired:.2f}, follower={actual:.2f}, previous goal={previous:.2f}. "
                    "Stop and check for obstruction or an unexpectedly low speed setting."
                )
            applied[joint] = actual + max(
                -MAX_GOAL_LEAD, min(MAX_GOAL_LEAD, desired - actual),
            )
        return applied

    def committed(self, applied):
        self.last_goals = dict(applied)
