"""Preview and explicitly restore only the follower gripper's saved calibration."""

import hashlib
import json
import secrets
import time
from pathlib import Path

from cf_robot.calibration_inspection import calibration_record

MOTOR = "gripper"
PLAN_TTL_S = 60
POSITION_TOLERANCE_TICKS = 20
LIMIT_MARGIN_TICKS = 20
REGISTERS = {
    "homing_offset": "Homing_Offset",
    "range_min": "Min_Position_Limit",
    "range_max": "Max_Position_Limit",
}


def _read(bus, register):
    return int(bus.read(register, MOTOR, normalize=False, num_retry=2))


def _snapshot(device):
    bus = device.bus
    motor = bus.motors[MOTOR]
    mode = getattr(motor.norm_mode, "value", motor.norm_mode)
    if (
        device.id != "so101_follower" or motor.id != 6 or motor.model != "sts3215"
        or mode != "range_0_100" or bus.protocol_version != 0
    ):
        raise ValueError("restore supports only the SO101 follower's J6 STS3215 gripper")
    loaded = {
        name: calibration_record(record)
        for name, record in bus.calibration.items()
    }
    path = Path(device.calibration_fpath)
    content = path.read_bytes()
    if json.loads(content) != loaded:
        raise ValueError("calibration file differs from loaded calibration; restart monitoring")
    saved = loaded.get(MOTOR)
    if (
        not saved or any(type(saved.get(key)) is not int for key in (
            "id", "drive_mode", *REGISTERS,
        ))
        or saved["id"] != 6 or saved["drive_mode"] != 0
        or not -2047 <= saved["homing_offset"] <= 2047
        or not 0 <= saved["range_min"] < saved["range_max"] <= 4095
        or saved["range_max"] - saved["range_min"] <= 100
    ):
        raise ValueError("saved J6 calibration is invalid or has an unsupported direction")
    if bus.ping(MOTOR, num_retry=2, raise_on_error=True) != bus.model_number_table["sts3215"]:
        raise ValueError("J6 motor model verification failed")
    if _read(bus, "Operating_Mode") != 0:
        raise ValueError("J6 must be in position-control mode")
    if _read(bus, "Torque_Enable"):
        raise ValueError("J6 torque must already be OFF; secure the arm, then use J6 TD")
    live = {
        name: calibration_record(record) for name, record in bus.read_calibration().items()
    }
    raw = _read(bus, "Present_Position")
    # Feetech: Present_Position = Actual_Position - Homing_Offset.
    projected = raw + live[MOTOR]["homing_offset"] - saved["homing_offset"]
    if not saved["range_min"] + LIMIT_MARGIN_TICKS <= projected <= saved["range_max"] - LIMIT_MARGIN_TICKS:
        raise ValueError(
            f"projected J6 position {projected} ticks is not safely inside saved limits "
            f"{saved['range_min']}..{saved['range_max']} (margin {LIMIT_MARGIN_TICKS}); "
            "with torque OFF, move gently inward and preview again"
        )
    lock = _read(bus, "Lock")
    if lock not in (0, 1):
        raise ValueError("unexpected J6 EEPROM lock state")
    return {
        "file": str(path), "file_sha256": hashlib.sha256(content).hexdigest(),
        "saved": saved, "live": live, "raw": raw, "projected_raw": projected,
        "lock": lock,
    }


class GripperCalibrationRestore:
    """Session-scoped one-use previews; no arbitrary register or joint writes."""

    def __init__(self):
        self.plan = None
        self.tainted = False

    def preview(self, device):
        self.plan = None
        snapshot = _snapshot(device)
        self.plan = {
            **snapshot, "token": secrets.token_hex(16), "created_at": time.monotonic(),
        }
        return {
            "joint_id": 6, "current": snapshot["live"][MOTOR],
            "proposed": snapshot["saved"], "current_raw": snapshot["raw"],
            "projected_raw": snapshot["projected_raw"], "torque_enabled": False,
            "eeprom_lock": snapshot["lock"],
            "file": snapshot["file"], "file_sha256": snapshot["file_sha256"],
            "expires_in_s": PLAN_TTL_S, "token": self.plan["token"],
            "instruction": "J6 RESTORE CONFIRM",
            "warning": "Writes only J6 offset/limits. No goal writes; torque stays OFF. File and J1-J5 unchanged.",
        }

    def apply(self, device, token, stop):
        plan, self.plan = self.plan, None
        if not plan or not secrets.compare_digest(plan["token"], token):
            raise ValueError("no matching unused J6 preview; request J6 RESTORE PREVIEW")
        if time.monotonic() - plan["created_at"] > PLAN_TTL_S:
            raise ValueError("J6 preview expired; preview again")
        current = _snapshot(device)
        for key in ("file", "file_sha256", "saved", "live", "lock"):
            if current[key] != plan[key]:
                raise ValueError("calibration or servo configuration changed since preview; preview again")
        if abs(current["raw"] - plan["raw"]) > POSITION_TOLERANCE_TICKS:
            raise ValueError("J6 moved since preview; keep it still and preview again")
        if stop.is_set():
            raise ValueError("restore cancelled before writing")
        bus = device.bus
        writes = []
        self.tainted = True

        def write(register, value):
            if stop.is_set():
                raise RuntimeError("restore interrupted; settings may be partially changed")
            if _read(bus, "Torque_Enable"):
                raise RuntimeError("J6 torque changed unexpectedly; refusing calibration write")
            # EEPROM writes are never automatically retried after an uncertain outcome.
            bus.write(register, MOTOR, value, normalize=False, num_retry=0)
            writes.append(register)

        try:
            if current["lock"]:
                write("Lock", 0)
            fields = ["homing_offset", "range_min", "range_max"]
            if plan["saved"]["range_min"] >= current["live"][MOTOR]["range_max"]:
                fields = ["homing_offset", "range_max", "range_min"]
            for field in fields:
                write(REGISTERS[field], plan["saved"][field])
            for field, register in REGISTERS.items():
                if _read(bus, register) != plan["saved"][field]:
                    raise RuntimeError(f"J6 verification failed for {register}")
            raw = _read(bus, "Present_Position")
            if abs(raw - current["projected_raw"]) > POSITION_TOLERANCE_TICKS:
                raise RuntimeError("J6 position changed unexpectedly during restore")
            if current["lock"]:
                write("Lock", current["lock"])
            if _read(bus, "Lock") != current["lock"]:
                raise RuntimeError("J6 EEPROM lock verification failed")
            after = {
                name: calibration_record(record)
                for name, record in bus.read_calibration().items()
            }
            if after.get(MOTOR) != plan["saved"]:
                raise RuntimeError("J6 calibration readback does not match the saved record")
            if {key: value for key, value in after.items() if key != MOTOR} != {
                key: value for key, value in current["live"].items() if key != MOTOR
            }:
                raise RuntimeError("J1-J5 configuration changed externally during restore")
            if _read(bus, "Torque_Enable"):
                raise RuntimeError("J6 torque is unexpectedly enabled; inspect the arm")
            if hashlib.sha256(Path(plan["file"]).read_bytes()).hexdigest() != plan["file_sha256"]:
                raise RuntimeError("calibration file changed during restore")
            self.tainted = False
            return {
                "before": current["live"][MOTOR], "after": after[MOTOR],
                "raw": raw, "torque_enabled": False, "writes": writes,
            }
        except Exception as exc:  # noqa: BLE001 - preserve partial-write uncertainty.
            raise RuntimeError(
                f"J6 restore failed: {exc}. Settings may be partially changed; no rollback or "
                "automatic retry was attempted. Torque was never enabled. Inspect J6 and "
                "use a new preview before retrying; do not resume teleoperation."
            ) from exc
