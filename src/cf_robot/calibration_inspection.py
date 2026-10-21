"""Read-only calibration inspection and independent physical validation."""

import hashlib
import json
import math
import time
from dataclasses import asdict, is_dataclass
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from cf_robot.protocol import JOINTS

FIELDS = ("id", "drive_mode", "homing_offset", "range_min", "range_max")


def calibration_record(record):
    if record is None:
        return None
    if is_dataclass(record):
        return asdict(record)
    if isinstance(record, dict):
        return dict(record)
    return {field: getattr(record, field, None) for field in FIELDS}


def inspect_calibration(device, side, limits):
    """Inspect the loaded calibration and live bus without configuring motors."""
    bus = device.bus
    saved = {
        name: calibration_record(record)
        for name, record in (getattr(bus, "calibration", None) or {}).items()
    }
    report = {
        "side": side, "arm_id": getattr(device, "id", None),
        "port": getattr(getattr(device, "config", None), "port", None),
        "arm_type": getattr(device, "name", type(device).__name__),
        "inspected_at": time.time(), "joints": {}, "warnings": [],
        "physical_validation": "not validated",
    }
    try:
        report["lerobot_version"] = version("lerobot")
    except PackageNotFoundError:
        report["lerobot_version"] = "unavailable"
    report["loaded_sha256"] = hashlib.sha256(
        json.dumps(saved, sort_keys=True).encode()
    ).hexdigest()
    path = getattr(device, "calibration_fpath", None)
    report["file"] = {"path": str(path) if path else None}
    if path:
        try:
            path = Path(path)
            content = path.read_bytes()
            report["file"].update(
                sha256=hashlib.sha256(content).hexdigest(),
                modified_at=path.stat().st_mtime,
                matches_loaded=json.loads(content) == saved,
            )
            if not report["file"]["matches_loaded"]:
                report["warnings"].append("Calibration file differs from loaded calibration.")
        except (OSError, ValueError) as exc:
            report["warnings"].append(f"Cannot read calibration file: {exc}")
    else:
        report["warnings"].append("Calibration file path unavailable.")
    try:
        live = {
            name: calibration_record(record)
            for name, record in bus.read_calibration().items()
        }
    except Exception as exc:  # noqa: BLE001 - SDK errors must remain visible in inspection.
        live = {}
        report["warnings"].append(f"Live calibration unavailable: {exc}")
    if set(saved) != set(bus.motors):
        report["warnings"].append("Saved calibration joint names do not match configured motors.")
    ids = [motor.id for motor in bus.motors.values()]
    for joint in JOINTS:
        name = joint.removesuffix(".pos")
        motor = bus.motors[name]
        mode = getattr(motor, "norm_mode", None)
        mode = getattr(mode, "value", mode)
        unit = {"degrees": "deg", "range_0_100": "%", "range_m100_100": "normalized"}.get(mode)
        record = saved.get(name)
        actual = live.get(name)
        warnings = []
        item = {
            "id": motor.id, "model": getattr(motor, "model", None),
            "mode": mode, "unit": unit, "saved": record, "live": actual,
            "servo_limits": limits.get(joint), "warnings": warnings,
            "resolution": getattr(bus, "model_resolution_table", {}).get(
                getattr(motor, "model", None)
            ),
            "drive_mode_applied": bool(getattr(bus, "apply_drive_mode", False))
            and mode != "degrees",
        }
        if ids.count(motor.id) != 1:
            warnings.append("Duplicate configured motor ID.")
        if item["resolution"] is None:
            warnings.append("Encoder resolution unavailable.")
        try:
            item["detected_model_number"] = bus.ping(name, num_retry=2, raise_on_error=True)
            expected_model = getattr(bus, "model_number_table", {}).get(item["model"])
            if expected_model is None or item["detected_model_number"] != expected_model:
                warnings.append("Detected motor model differs from configured model or is unknown.")
        except Exception as exc:  # noqa: BLE001 - SDK-specific transport exceptions.
            item["detected_model_number"] = None
            warnings.append(f"Motor identity not verified: {exc}")
        if unit != ("%" if joint == "gripper.pos" else "deg"):
            warnings.append("Unexpected normalization mode; do not interpret this value as degrees/percent.")
        if not record or any(type(record.get(field)) is not int for field in FIELDS):
            warnings.append("Missing or incomplete saved calibration.")
        else:
            if record["id"] != motor.id:
                warnings.append("Saved motor ID differs from configured ID.")
            maximum = (item["resolution"] or 4096) - 1
            if not 0 <= record["range_min"] < record["range_max"] <= maximum:
                warnings.append("Invalid saved encoder range.")
            elif record["range_max"] - record["range_min"] <= 100:
                warnings.append("Saved encoder range is too small (at most 100 ticks).")
            else:
                try:
                    low, high = record["range_min"], record["range_max"]

                    def convert(raw, motor_id=motor.id):
                        return float(bus._normalize({motor_id: raw})[motor_id])

                    item["converted_range"] = [convert(low), convert(high)]
                    item["reference_ticks"] = (low + high) / 2
                    item["reference_value"] = convert((low + high) / 2)
                    item["positive_direction"] = (
                        "increasing encoder ticks" if convert(high) > convert(low)
                        else "decreasing encoder ticks"
                    )
                except (KeyError, ValueError, RuntimeError, AttributeError) as exc:
                    warnings.append(f"Conversion unavailable: {exc}")
            if actual:
                for field in ("homing_offset", "range_min", "range_max"):
                    if actual.get(field) != record[field]:
                        warnings.append(f"Saved/live mismatch: {field}.")
            else:
                warnings.append("Live calibration not verified.")
        try:
            item["operating_mode"] = int(
                bus.read("Operating_Mode", name, normalize=False, num_retry=2)
            )
            if item["operating_mode"] != 0:
                warnings.append("Servo is not in position-control mode (0).")
        except Exception as exc:  # noqa: BLE001 - SDK-specific register exceptions.
            item["operating_mode"] = None
            warnings.append(f"Operating mode unavailable: {exc}")
        angular_profile = getattr(bus, "_cf_robot_angular_calibration", None)
        angular = angular_profile.describe(name) if angular_profile is not None else None
        if angular is not None:
            item["angular_calibration"] = angular
            angular_limits = angular["limits"]
            # Keep the old wire field for existing operator assets. After
            # migration it aliases hardware bounds, not an editable extra range.
            item["working_range"] = angular_limits
            item["reference_ticks"] = angular["zero_ticks"]
            item["reference_value"] = 0.0
            item["converted_range"] = [
                (raw - angular["zero_ticks"]) * angular["degrees_per_tick"]
                for raw in (angular_limits["min"], angular_limits["max"])
            ]
            item["positive_direction"] = angular.get("positive_direction", "verify physically")
            if angular["error"]:
                warnings.append(f"Angular calibration: {angular['error']}")
        item["state"] = "warning" if warnings else "consistent"
        report["joints"][joint] = item
    report["state"] = "warning" if report["warnings"] or any(
        item["warnings"] for item in report["joints"].values()
    ) else "consistent"
    report["validation_key"] = hashlib.sha256(json.dumps({
        "side": side, "arm_id": report["arm_id"], "port": report["port"],
        "loaded": saved,
        "joints": {
            joint: {key: item.get(key) for key in (
                "id", "model", "mode", "resolution", "live", "operating_mode",
                "angular_calibration", "working_range",
            )}
            for joint, item in report["joints"].items()
        },
    }, sort_keys=True).encode()).hexdigest()
    return report


def capture_validation(samples, body):
    """Capture measured evidence; never infer physical truth from the 3D model."""
    if not isinstance(body, dict) or body.get("side") not in ("leader", "follower", "both"):
        raise ValueError("select leader, follower, or both")
    if body.get("confirmed") is not True:
        raise ValueError("confirm the physical pose was independently checked")
    label = body.get("label")
    if not isinstance(label, str) or not label.strip() or len(label) > 80:
        raise ValueError("provide a pose label of at most 80 characters")
    sides = ("leader", "follower") if body["side"] == "both" else (body["side"],)
    evidence = {}
    for side in sides:
        sample = samples.get(side)
        if not sample or not 0 <= time.time() - sample["sampled_at"] < 1.5:
            raise ValueError(f"{side} sample missing or stale")
        inspection = sample.get("calibration")
        if not inspection:
            raise ValueError(f"{side} calibration inspector unavailable; update both packages")
        evidence[side] = {
            "sampled_at": sample["sampled_at"], "values": dict(sample["values"]),
            "raw": dict(sample["raw"]), "calibration": inspection["validation_key"],
            "file_sha256": inspection["file"].get("sha256"),
            "consistent": inspection["state"] == "consistent",
        }
    result = {"label": label.strip(), "side": body["side"], "captured_at": time.time(), "evidence": evidence}
    if body["side"] == "both":
        if abs(samples["leader"]["sampled_at"] - samples["follower"]["sampled_at"]) > 0.3:
            raise ValueError("matched samples are too far apart in time")
        result["deltas"] = {}
        for joint in JOINTS:
            modes = [samples[side]["calibration"]["joints"][joint]["mode"] for side in sides]
            values = [samples[side]["values"][joint] for side in sides]
            if modes[0] != modes[1] or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                raise ValueError("matched capture requires finite values in matching units")
            result["deltas"][joint] = values[0] - values[1]
        return result
    joint = body.get("joint")
    if joint not in JOINTS:
        raise ValueError("select a joint for independent validation")
    expected, tolerance = body.get("expected"), body.get("tolerance")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (expected, tolerance)):
        raise ValueError("expected value and tolerance must be finite numbers")
    unit = samples[body["side"]]["calibration"]["joints"][joint]["unit"]
    if unit not in ("deg", "%") or not 0 < tolerance <= 100:
        raise ValueError("validation requires degrees/percent and tolerance in (0,100]")
    if not (-360 <= expected <= 360) or (unit == "%" and not 0 <= expected <= 100):
        raise ValueError("expected value outside supported range")
    observed = samples[body["side"]]["values"][joint]
    if type(observed) not in (int, float) or not math.isfinite(observed):
        raise ValueError("normalized joint value unavailable")
    error = observed - expected
    result.update(
        joint=joint, expected=expected, observed=observed, unit=unit,
        tolerance=tolerance, error=error, passed=abs(error) <= tolerance,
    )
    return result


def validation_analysis(captures, samples):
    """Fit observed vs independently measured values, grouped by calibration."""
    results = []
    for side in ("leader", "follower"):
        current = ((samples.get(side) or {}).get("calibration") or {}).get("validation_key")
        for joint in JOINTS:
            points = [
                point for point in captures
                if point["side"] == side and point.get("joint") == joint
                and point["evidence"][side]["calibration"] == current
            ]
            if not points:
                continue
            result = {
                "side": side, "joint": joint, "points": len(points),
                "state": "insufficient evidence", "notes": [],
            }
            if (
                len({point["expected"] for point in points}) >= 3
                and max(point["expected"] for point in points) - min(point["expected"] for point in points)
                > 2 * max(point["tolerance"] for point in points)
            ):
                x = [point["expected"] for point in points]
                y = [point["observed"] for point in points]
                mx, my = sum(x) / len(x), sum(y) / len(y)
                slope = sum((a - mx) * (b - my) for a, b in zip(x, y)) / sum((a - mx) ** 2 for a in x)
                offset = my - slope * mx
                result.update(slope=slope, offset=offset)
                if slope < 0:
                    result["notes"].append("Likely reversed direction.")
                elif abs(slope - 1) > 0.1:
                    result["notes"].append("Possible scale/unit mismatch.")
                if abs(offset) > min(point["tolerance"] for point in points):
                    result["notes"].append("Possible reference/offset error.")
                if not all(point["passed"] for point in points):
                    result["notes"].append("One or more measured poses exceed tolerance.")
                if not all(point["evidence"][side]["consistent"] for point in points):
                    result["notes"].append("Calibration consistency warnings remain.")
                result["state"] = "review" if result["notes"] else "checked at captured poses"
            else:
                result["notes"].append("Use three distinct poses spanning more than twice the measurement tolerance.")
            result["notes"].append("Not a safety certification or full-range validation.")
            results.append(result)
    return results
