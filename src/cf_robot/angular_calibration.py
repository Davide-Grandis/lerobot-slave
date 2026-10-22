"""Persistent per-arm angular references, independent of servo limit midpoints."""

import hashlib
import json
import math
from pathlib import Path

from cf_robot.protocol import JOINTS


def _field(record, name):
    return record.get(name) if isinstance(record, dict) else getattr(record, name, None)


def _pending_reason(record):
    if record.get("pending_range_max") is True:
        return "physical maximum is provisional; raw ticks only, no degree/goal control"
    if record.get("pending_zero") is True:
        return "physical zero is not yet recorded; raw ticks only, no degree control"
    return None


class AngularCalibration:
    """Overlay only configured degree conversions on one SDK bus instance."""

    def __init__(self, device, side, path):
        self.bus = device.bus
        self.path = Path(path)
        content = self.path.read_bytes()
        self.sha256 = hashlib.sha256(content).hexdigest()
        data = json.loads(content)
        if (
            not isinstance(data, dict) or data.get("version") != 1
            or data.get("side") != side or data.get("arm_id") != device.id
            or not isinstance(data.get("joints"), dict)
        ):
            raise ValueError(f"invalid angular calibration identity/schema: {path}")
        self.records = data["joints"]
        self.ids = {}
        self.errors = {}
        self.limits = {}
        self.hardware_limits = {}
        self.limit_sources = {}
        for name, record in self.records.items():
            if name not in {joint.removesuffix(".pos") for joint in JOINTS[:-1]}:
                raise ValueError("angular overrides support only J1-J5, never the gripper")
            if not isinstance(record, dict):
                raise ValueError(f"invalid angular record for {name}")
            motor = self.bus.motors[name]
            mode = getattr(motor.norm_mode, "value", motor.norm_mode)
            if (
                type(record.get("id")) is not int or record["id"] != motor.id
                or motor.id != JOINTS.index(f"{name}.pos") + 1
                or record.get("model") != motor.model or motor.model != "sts3215"
                or mode != "degrees" or type(record.get("homing_offset")) is not int
                or not -2047 <= record["homing_offset"] <= 2047
            ):
                raise ValueError(f"angular calibration motor identity/mode mismatch: {name}")
            saved = (self.bus.calibration or {}).get(name)
            if ("range_min" in record) != ("range_max" in record):
                raise ValueError(f"incomplete legacy angular limits for {name}")
            low = record.get("range_min", _field(saved, "range_min"))
            high = record.get("range_max", _field(saved, "range_max"))
            pending = record.get("pending_zero") is True
            zero, scale = record.get("zero_ticks"), record.get("degrees_per_tick")
            numbers = [low, high, scale] + ([] if pending else [zero])
            if any(type(value) not in (int, float) or not math.isfinite(value) for value in numbers):
                raise ValueError(f"non-finite angular calibration for {name}")
            if (
                type(low) is not int or type(high) is not int
                or not 0 <= low < high <= 4095 or high - low <= 100
                or not 0 < abs(scale) <= 1
                or (not pending and (
                    not low <= zero <= high
                    or max(abs((low - zero) * scale), abs((high - zero) * scale)) > 360
                ))
            ):
                raise ValueError(f"invalid angular range/scale for {name}")
            self.ids[motor.id] = name
            self.limits[name] = {"min": low, "max": high}
            self.limit_sources[name] = "legacy_software" if "range_min" in record else "hardware"
            self.errors[name] = "live encoder coordinate system not verified; connect first"
        self.original_connect = self.bus.connect
        self.original_normalize = self.bus._normalize
        self.original_unnormalize = self.bus._unnormalize
        self.original_write = self.bus.write
        self.original_sync_write = getattr(self.bus, "sync_write", None)

    def _check_saved(self, name):
        record = self.records[name]
        saved = (self.bus.calibration or {}).get(name)
        if (
            saved is None or _field(saved, "id") != record["id"]
            or _field(saved, "homing_offset") != record["homing_offset"]
            or _field(saved, "drive_mode") != 0
        ):
            raise ValueError(
                f"{name} angular calibration no longer matches the saved homing offset/identity; "
                "remeasure references before updating the profile"
            )

    def connect(self, *args, **kwargs):
        self.errors = {name: "live encoder coordinate system not verified" for name in self.records}
        result = self.original_connect(*args, **kwargs)
        for name, record in self.records.items():
            try:
                self._check_saved(name)
                offset = int(self.bus.read("Homing_Offset", name, normalize=False, num_retry=2))
                low = int(self.bus.read("Min_Position_Limit", name, normalize=False, num_retry=2))
                high = int(self.bus.read("Max_Position_Limit", name, normalize=False, num_retry=2))
                self.hardware_limits[name] = {"min": low, "max": high}
                if offset != record["homing_offset"]:
                    raise ValueError(
                        f"live homing offset {offset} differs from angular reference offset "
                        f"{record['homing_offset']}; do not reuse ticks from a different coordinate system"
                    )
                requested = self.limits[name]
                if "range_min" not in record:
                    self.limits[name] = {"min": low, "max": high}
                    saved = self.bus.calibration[name]
                    if (_field(saved, "range_min"), _field(saved, "range_max")) != (low, high):
                        raise ValueError("saved SDK limits differ from hardware limits; complete/repair the limit update")
                    requested = self.limits[name]
                if not 0 <= low < high <= 4095 or not low <= requested["min"] < requested["max"] <= high:
                    raise ValueError("angular working range is outside live servo limits")
                self.limit_sources[name] = (
                    "hardware" if requested == self.hardware_limits[name] else "legacy_software"
                )
                self.errors[name] = _pending_reason(record)
            except Exception as exc:  # noqa: BLE001 - keep raw inspection available on SDK errors.
                self.errors[name] = str(exc)
        return result

    def _verified(self, name):
        self._check_saved(name)
        pending = _pending_reason(self.records[name])
        if pending:
            raise ValueError(f"{name}: {pending}")
        if self.errors[name]:
            raise ValueError(f"{name} angular calibration: {self.errors[name]}")
        return self.records[name]

    def normalize(self, values):
        ordinary = {motor_id: value for motor_id, value in values.items() if motor_id not in self.ids}
        result = self.original_normalize(ordinary) if ordinary else {}
        for motor_id, value in values.items():
            if motor_id in self.ids:
                record = self._verified(self.ids[motor_id])
                result[motor_id] = (value - record["zero_ticks"]) * record["degrees_per_tick"]
        return result

    def unnormalize(self, values):
        ordinary = {motor_id: value for motor_id, value in values.items() if motor_id not in self.ids}
        result = self.original_unnormalize(ordinary) if ordinary else {}
        for motor_id, value in values.items():
            if motor_id in self.ids:
                record = self._verified(self.ids[motor_id])
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError("angular target must be finite")
                raw = record["zero_ticks"] + value / record["degrees_per_tick"]
                limits = self.limits[self.ids[motor_id]]
                if not limits["min"] <= raw <= limits["max"]:
                    raise ValueError("angular target is outside the calibrated working range")
                result[motor_id] = math.floor(raw + 0.5)
        return result

    def _check_write(self, data_name, motor, value, normalize):
        name = self.ids.get(motor) if isinstance(motor, int) else motor
        if name not in self.records:
            return
        if data_name in ("Homing_Offset", "Min_Position_Limit", "Max_Position_Limit"):
            self.errors[name] = "servo calibration changed; restart and verify the angular reference"
        if data_name == "Goal_Position" and not normalize:
            self._verified(name)
            limits = self.limits[name]
            if not math.isfinite(value) or not limits["min"] <= value <= limits["max"]:
                raise ValueError("raw target is outside the calibrated working range")

    def write(self, data_name, motor, value, *, normalize=True, num_retry=0):
        self._check_write(data_name, motor, value, normalize)
        return self.original_write(data_name, motor, value, normalize=normalize, num_retry=num_retry)

    def sync_write(self, data_name, values, *, normalize=True, num_retry=0):
        selected = values if isinstance(values, dict) else dict.fromkeys(self.records, values)
        for motor, value in selected.items():
            self._check_write(data_name, motor, value, normalize)
        return self.original_sync_write(data_name, values, normalize=normalize, num_retry=num_retry)

    def describe(self, name):
        if name not in self.records:
            return None
        record = self.records[name]
        error = self.errors[name]
        try:
            self._check_saved(name)
        except ValueError as exc:
            error = str(exc)
        return {
            **record, "file": str(self.path), "sha256": self.sha256,
            "zero_ticks": record.get("zero_ticks"),
            "state": "warning" if error else "verified", "error": error,
            "limits": dict(self.limits[name]),
            "hardware_limits": self.hardware_limits.get(name),
            "limits_source": self.limit_sources[name],
            "persistence": (
                "angular references in sidecar; limits in servo EEPROM"
                if self.limit_sources[name] == "hardware"
                else "legacy software bounds; confirmed hardware migration pending"
            ),
        }


def attach_angular_calibration(device, side):
    path = Path(device.calibration_fpath).with_suffix(".cf-robot.json")
    if not path.is_file():
        return device
    bus = device.bus
    if getattr(bus, "_cf_robot_angular_calibration", None) is not None:
        raise ValueError("angular calibration already attached")
    profile = AngularCalibration(device, side, path)
    # SDK reads/writes and direct monitor conversions use these same per-bus
    # hooks, avoiding display-only corrections or changes to other motor models.
    bus._cf_robot_angular_calibration = profile
    bus.connect = profile.connect
    bus._normalize = profile.normalize
    bus._unnormalize = profile.unnormalize
    bus.write = profile.write
    if profile.original_sync_write is not None:
        bus.sync_write = profile.sync_write
    return device
