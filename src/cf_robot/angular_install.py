"""Preview/confirm selected hardware limits plus matching angular references."""

from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import stat
import tempfile
import time

from cf_robot.calibration_inspection import calibration_record
from cf_robot.protocol import JOINTS


def _digest(content):
    return None if content is None else hashlib.sha256(content).hexdigest()


def _atomic_write(path, content, mode):
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as output:
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class AngularInstall:
    def __init__(self, side="follower"):
        if side not in ("leader", "follower"):
            raise ValueError("invalid arm side")
        self.side = side
        self.plan = None
        self.write_started = False

    def _snapshot(self, device, joint_id):
        if type(joint_id) is not int or not 1 <= joint_id <= 5:
            raise ValueError("angular installation supports J1-J5, never J6")
        name = JOINTS[joint_id - 1].removesuffix(".pos")
        bus = device.bus
        motor = bus.motors[name]
        mode = getattr(motor.norm_mode, "value", motor.norm_mode)
        if (
            (self.side == "follower" and device.id != "so101_follower") or motor.id != joint_id
            or motor.model != "sts3215" or mode != "degrees"
            or bus.protocol_version != 0
        ):
            raise ValueError("limit installation supports J1-J5 STS3215 degree joints only")
        if bus.ping(name, num_retry=2, raise_on_error=True) != bus.model_number_table["sts3215"]:
            raise ValueError("motor model verification failed")
        if int(bus.read("Torque_Enable", name, normalize=False, num_retry=2)):
            raise ValueError(f"secure/support the arm first; J{joint_id} torque must already be OFF")
        if int(bus.read("Operating_Mode", name, normalize=False, num_retry=2)) != 0:
            raise ValueError("joint must be in position-control mode")
        path = Path(device.calibration_fpath)
        sdk_bytes = path.read_bytes()
        sdk = json.loads(sdk_bytes)
        loaded = {key: calibration_record(value) for key, value in bus.calibration.items()}
        if sdk != loaded:
            raise ValueError("SDK file changed since connection; restart monitoring")
        if sdk[name]["id"] != joint_id or sdk[name]["drive_mode"] != 0:
            raise ValueError("saved motor ID/direction is incompatible with the angular reference")
        live = {key: calibration_record(value) for key, value in bus.read_calibration().items()}
        profile_path = path.with_suffix(".cf-robot.json")
        profile_bytes = profile_path.read_bytes() if profile_path.exists() else None
        profile = json.loads(profile_bytes) if profile_bytes is not None else {
            "version": 1, "side": self.side, "arm_id": device.id, "joints": {},
        }
        if (
            profile.get("version") != 1 or profile.get("side") != self.side
            or profile.get("arm_id") != device.id or not isinstance(profile.get("joints"), dict)
        ):
            raise ValueError("existing angular profile has an incompatible identity/schema")
        return {
            "name": name, "sdk_path": path, "sdk_bytes": sdk_bytes, "sdk": sdk,
            "profile_path": profile_path, "profile_bytes": profile_bytes, "profile": profile,
            "live": live,
            "lock": int(bus.read("Lock", name, normalize=False, num_retry=2)),
            "raw": int(bus.read("Present_Position", name, normalize=False, num_retry=2)),
        }

    def preview(self, device, command):
        self.plan = None
        if command.get("write_hardware_limits") is not True:
            raise ValueError("explicit hardware-limit write intent is required")
        joint_id = command["joint_id"]
        snapshot = self._snapshot(device, joint_id)
        name = snapshot["name"]
        low, high = command["range_min"], command["range_max"]
        negative, positive = command["negative_ticks"], command["positive_ticks"]
        if not 0 <= low <= negative < positive <= high <= 4095 or high - low <= 100:
            raise ValueError("require MIN <= -90 ticks < +90 ticks <= MAX, within 0..4095")
        zero = (negative + positive) / 2
        scale = 180 / (positive - negative)
        if scale > 1 or max(abs((low - zero) * scale), abs((high - zero) * scale)) > 360:
            raise ValueError("reference points produce an unsupported angular scale/range")
        offset = snapshot["live"][name]["homing_offset"]
        if type(offset) is not int or not -2047 <= offset <= 2047:
            raise ValueError("invalid live homing offset")
        record = {
            "id": joint_id, "model": "sts3215", "homing_offset": offset,
            "zero_ticks": zero,
            "degrees_per_tick": scale, "positive_direction": "clockwise",
            "reference_points": [
                {"ticks": negative, "degrees": -90}, {"ticks": positive, "degrees": 90},
            ],
        }
        return self._make_plan(snapshot, joint_id, low, high, record)

    def preview_limits(self, device, joint_id, low, high):
        self.plan = None
        snapshot = self._snapshot(device, joint_id)
        record = deepcopy(snapshot["profile"]["joints"].get(snapshot["name"]))
        if not record:
            raise ValueError("install angular references before changing limits with this tool")
        if record["homing_offset"] != snapshot["live"][snapshot["name"]]["homing_offset"]:
            raise ValueError("angular references do not match the live offset; remeasure before writing")
        record.pop("range_min", None)
        record.pop("range_max", None)
        return self._make_plan(snapshot, joint_id, low, high, record)

    def _make_plan(self, snapshot, joint_id, low, high, record):
        name = snapshot["name"]
        if (
            type(record.get("id")) is not int or record["id"] != joint_id
            or record.get("model") != "sts3215"
            or type(record.get("homing_offset")) is not int
            or record["homing_offset"] != snapshot["live"][name]["homing_offset"]
            or any(
                type(record.get(key)) not in (int, float) or not math.isfinite(record[key])
                for key in ("zero_ticks", "degrees_per_tick")
            )
            or not 0 < abs(record["degrees_per_tick"]) <= 1
        ):
            raise ValueError("invalid or mismatched angular reference record")
        if type(low) is not int or type(high) is not int or not 0 <= low < high <= 4095 or high - low <= 100:
            raise ValueError("invalid hardware limits")
        if not low <= record["zero_ticks"] <= high:
            raise ValueError("angular zero is outside proposed hardware limits")
        angles = [(raw - record["zero_ticks"]) * record["degrees_per_tick"] for raw in (low, high)]
        if max(abs(value) for value in angles) > 360:
            raise ValueError("hardware range produces unsupported angular limits")
        if not low + 20 <= snapshot["raw"] <= high - 20:
            raise ValueError("with torque OFF, position the supported joint safely inside the new limits (20-tick margin)")
        if snapshot["lock"] not in (0, 1):
            raise ValueError("unexpected EEPROM lock state")
        offset = snapshot["live"][name]["homing_offset"]
        sdk = deepcopy(snapshot["sdk"])
        sdk[name].update(homing_offset=offset, range_min=low, range_max=high)
        profile = deepcopy(snapshot["profile"])
        profile["joints"][name] = record
        token = secrets.token_hex(16)
        self.plan = {
            **snapshot, "joint_id": joint_id, "proposed_sdk": sdk,
            "proposed_profile": profile, "token": token, "created_at": time.monotonic(),
        }
        return {
            "joint_id": joint_id, "token": token, "expires_in_s": 60,
            "live_homing_offset_preserved": offset,
            "saved_sdk_before": snapshot["sdk"][name], "saved_sdk_after": sdk[name],
            "angular_profile": record, "current_raw": snapshot["raw"],
            "angular_range": angles,
            "hardware_before": snapshot["live"][name],
            "hardware_after": {**snapshot["live"][name], "range_min": low, "range_max": high},
            "hardware_writes": {
                register: value for register, field, value in (
                    ("Min_Position_Limit", "range_min", low),
                    ("Max_Position_Limit", "range_max", high),
                ) if snapshot["live"][name][field] != value
            },
            "sdk_file": str(snapshot["sdk_path"]), "profile_file": str(snapshot["profile_path"]),
            "instruction": f"J{joint_id} ANGULAR CONFIRM",
            "warning": "WRITES selected joint MIN/MAX EEPROM and matching files. Homing, torque, goals and other joints including J6 unchanged. Restart required.",
        }

    def apply(self, device, command, stop):
        plan, self.plan = self.plan, None
        if (
            not plan or command["joint_id"] != plan["joint_id"]
            or not secrets.compare_digest(command["token"], plan["token"])
            or command.get("confirmation") != f"APPLY ANGULAR AND LIMITS J{plan['joint_id']}"
        ):
            raise ValueError("no matching unused angular preview")
        if time.monotonic() - plan["created_at"] > 60:
            raise ValueError("angular preview expired; preview again")
        current = self._snapshot(device, plan["joint_id"])
        if (
            current["sdk_path"] != plan["sdk_path"]
            or current["profile_path"] != plan["profile_path"]
            or _digest(current["sdk_bytes"]) != _digest(plan["sdk_bytes"])
            or _digest(current["profile_bytes"]) != _digest(plan["profile_bytes"])
            or current["live"] != plan["live"]
            or current["lock"] != plan["lock"]
        ):
            raise ValueError("files or motor settings changed since preview; preview again")
        if abs(current["raw"] - plan["raw"]) > 20:
            raise ValueError("joint moved since preview; keep it still and preview again")
        proposed = plan["proposed_sdk"][plan["name"]]
        if not proposed["range_min"] + 20 <= current["raw"] <= proposed["range_max"] - 20:
            raise ValueError("joint is no longer safely inside proposed limits; preview again")
        if stop.is_set():
            raise ValueError("angular installation cancelled before writing")
        backups = []
        for key in ("sdk", "profile"):
            content = plan[f"{key}_bytes"]
            if content is not None:
                backup = Path(f"{plan[f'{key}_path']}.before-angular-{plan['token']}.bak")
                with backup.open("xb") as output:
                    output.write(content)
                    output.flush()
                    os.fsync(output.fileno())
                backups.append(str(backup))
        sdk_mode = stat.S_IMODE(plan["sdk_path"].stat().st_mode)
        live_backup = Path(f"{plan['sdk_path']}.before-limits-{plan['token']}.json")
        with live_backup.open("x", encoding="utf-8") as output:
            json.dump({
                "side": self.side, "joint_id": plan["joint_id"],
                "live_before": current["live"][plan["name"]], "lock": current["lock"],
            }, output, indent=2)
            output.flush()
            os.fsync(output.fileno())
        backups.append(str(live_backup))
        if stop.is_set():
            raise ValueError("angular installation cancelled before writing; backups only were created")
        self.write_started = True
        bus, name = device.bus, plan["name"]
        hardware_after = {
            **plan["live"][name],
            "range_min": plan["proposed_sdk"][name]["range_min"],
            "range_max": plan["proposed_sdk"][name]["range_max"],
        }
        writes = []

        def write(register, value):
            if stop.is_set():
                raise RuntimeError("limit update interrupted")
            if int(bus.read("Torque_Enable", name, normalize=False, num_retry=2)):
                raise RuntimeError("joint torque changed unexpectedly; refusing EEPROM write")
            writes.append(register)
            bus.write(register, name, value, normalize=False, num_retry=0)

        try:
            fields = [
                (register, field) for register, field in (
                    ("Min_Position_Limit", "range_min"), ("Max_Position_Limit", "range_max"),
                ) if hardware_after[field] != current["live"][name][field]
            ]
            if fields and current["lock"]:
                write("Lock", 0)
            if hardware_after["range_min"] >= current["live"][name]["range_max"]:
                fields.reverse()
            for register, field in fields:
                write(register, hardware_after[field])
            after = {key: calibration_record(value) for key, value in bus.read_calibration().items()}
            if after[name] != hardware_after:
                raise RuntimeError("hardware limit/homing readback verification failed")
            if {key: value for key, value in after.items() if key != name} != {
                key: value for key, value in current["live"].items() if key != name
            }:
                raise RuntimeError("another joint changed unexpectedly during limit update")
            if fields and current["lock"]:
                write("Lock", current["lock"])
            if int(bus.read("Lock", name, normalize=False, num_retry=2)) != current["lock"]:
                raise RuntimeError("EEPROM lock verification failed")
            if stop.is_set():
                raise RuntimeError("interrupted before saving matching files")
            if int(bus.read("Torque_Enable", name, normalize=False, num_retry=2)):
                raise RuntimeError("joint torque unexpectedly enabled after limit writes")
            raw = int(bus.read("Present_Position", name, normalize=False, num_retry=2))
            if (
                abs(raw - current["raw"]) > 20
                or not hardware_after["range_min"] + 20 <= raw <= hardware_after["range_max"] - 20
            ):
                raise RuntimeError("joint position changed unexpectedly during limit update")
            # A failure between hardware and file updates is detected by the
            # normal saved/live preflight; no automatic rollback is attempted.
            _atomic_write(
                plan["profile_path"],
                (json.dumps(plan["proposed_profile"], indent=2) + "\n").encode(), 0o600,
            )
            if stop.is_set():
                raise RuntimeError("interrupted between profile and SDK file writes")
            _atomic_write(
                plan["sdk_path"],
                (json.dumps(plan["proposed_sdk"], indent=2) + "\n").encode(), sdk_mode,
            )
            if (
                json.loads(plan["sdk_path"].read_bytes()) != plan["proposed_sdk"]
                or json.loads(plan["profile_path"].read_bytes()) != plan["proposed_profile"]
            ):
                raise RuntimeError("angular file readback verification failed")
            return {
                "joint_id": plan["joint_id"], "profile_file": str(plan["profile_path"]),
                "sdk_file": str(plan["sdk_path"]), "backups": backups,
                "angular_profile": plan["proposed_profile"]["joints"][plan["name"]],
                "servo_writes": writes, "hardware_after": after[name],
                "restart_required": True,
            }
        except Exception as exc:  # noqa: BLE001 - report partial file-write uncertainty.
            raise RuntimeError(
                f"Hardware/file limit update incomplete: {exc}. EEPROM may be partially changed; "
                f"no automatic retry or rollback. Keep torque OFF and do not teleoperate until "
                f"readback/files are repaired. Backups: {backups}. Attempted writes: {writes}."
            ) from exc
