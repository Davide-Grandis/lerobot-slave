"""Explicit encoder-coordinate rebasing without guessing angular zero."""

import argparse
from copy import deepcopy
import json
import os
from pathlib import Path
import secrets
import stat
import threading
import time

from cf_robot.angular_install import AngularInstall, _atomic_write, _digest
from cf_robot.calibration_inspection import calibration_record
from cf_robot.hardware import make_leader


class EncoderRebase(AngularInstall):
    def __init__(self, side="leader"):
        super().__init__(side)

    def preview(
        self, device, joint_id, old_start, old_end, new_start, expected_offset, *,
        provisional_max=None,
    ):
        self.plan = None
        snapshot = self._snapshot(device, joint_id)
        name = snapshot["name"]
        resolution = device.bus.model_resolution_table["sts3215"]
        if resolution != 4096 or any(type(value) is not int for value in (
            old_start, new_start, expected_offset,
        )):
            raise ValueError("require integer coordinates on the 4096-count STS3215 encoder")
        if not 0 <= old_start < resolution:
            raise ValueError("invalid measured range start")
        if snapshot["live"][name]["homing_offset"] != expected_offset:
            raise ValueError("unexpected live offset: already rebased or coordinate system changed; inspect before retrying")
        shift = (new_start - old_start + resolution // 2) % resolution - resolution // 2
        if old_end is None:
            if type(provisional_max) is not int:
                raise ValueError("provide a measured old end or an explicit provisional maximum")
            high = provisional_max
        else:
            if type(old_end) is not int or not 0 <= old_end < resolution or provisional_max is not None:
                raise ValueError("provide either a measured old end or a provisional maximum, not both")
            high = new_start + (old_end - old_start) % resolution
        # Equivalent offsets one revolution apart preserve the encoder coordinates.
        offset = (expected_offset - shift + resolution // 2) % resolution - resolution // 2
        if not 0 <= new_start < high < resolution or high - new_start <= 100:
            raise ValueError("rebased range does not fit safely within one encoder revolution")
        if not -2047 <= offset <= 2047:
            raise ValueError("rebasing would exceed the supported homing-offset register range")
        projected = (snapshot["raw"] + shift) % resolution
        if not new_start + 20 <= projected <= high - 20:
            raise ValueError("support the joint and move it away from the endpoint with torque OFF; need 20-tick margin")
        if snapshot["lock"] not in (0, 1):
            raise ValueError("unexpected EEPROM lock state")
        proposed = {**snapshot["live"][name], "homing_offset": offset,
                    "range_min": new_start, "range_max": high}
        sdk, profile = deepcopy(snapshot["sdk"]), deepcopy(snapshot["profile"])
        sdk[name].update(homing_offset=offset, range_min=new_start, range_max=high)
        profile["joints"][name] = {
            "id": joint_id, "model": "sts3215", "homing_offset": offset,
            "degrees_per_tick": 360 / resolution, "encoder_resolution": resolution,
            "pending_zero": True, "positive_direction": "increasing encoder ticks",
            "pending_range_max": old_end is None,
        }
        self.plan = {
            **snapshot, "joint_id": joint_id, "shift": shift, "projected": projected,
            "proposed": proposed, "proposed_sdk": sdk, "proposed_profile": profile,
            "token": secrets.token_hex(16), "created_at": time.monotonic(),
        }
        return {
            "joint_id": joint_id, "token": self.plan["token"], "expires_in_s": 60,
            "before": snapshot["live"][name], "after": proposed,
            "raw": snapshot["raw"], "projected_raw": projected, "tick_shift": shift,
            "pending_physical_zero": True,
            "maximum_is_provisional": old_end is None,
            "warning": "Writes selected HOMING OFFSET and MIN/MAX EEPROM plus files. Torque stays OFF; no goals or other-joint writes. Degree/goal control stays blocked while zero or range verification is pending.",
        }

    def apply(self, device, token, stop):
        plan, self.plan = self.plan, None
        if not plan or not secrets.compare_digest(plan["token"], token):
            raise ValueError("no matching unused encoder-rebase preview")
        if time.monotonic() - plan["created_at"] > 60:
            raise ValueError("encoder-rebase preview expired")
        current = self._snapshot(device, plan["joint_id"])
        if any(current[key] != plan[key] for key in ("sdk_path", "profile_path", "live", "lock")) or any(
            _digest(current[key]) != _digest(plan[key]) for key in ("sdk_bytes", "profile_bytes")
        ):
            raise ValueError("settings/files changed since preview; do not write")
        distance = abs((current["raw"] - plan["raw"] + 2048) % 4096 - 2048)
        projected = (current["raw"] + plan["shift"]) % 4096
        low, high = plan["proposed"]["range_min"], plan["proposed"]["range_max"]
        if distance > 20 or not low + 20 <= projected <= high - 20:
            raise ValueError("joint moved or is too close to an endpoint; preview again")
        if stop.is_set():
            raise ValueError("rebase cancelled before writing")
        backups = []
        for key in ("sdk", "profile"):
            if plan[f"{key}_bytes"] is not None:
                path = Path(f"{plan[f'{key}_path']}.before-rebase-{plan['token']}.bak")
                with path.open("xb") as output:
                    output.write(plan[f"{key}_bytes"])
                    output.flush()
                    os.fsync(output.fileno())
                backups.append(str(path))
        live_path = Path(f"{plan['sdk_path']}.before-rebase-{plan['token']}.live.json")
        with live_path.open("x", encoding="utf-8") as output:
            json.dump({"joint_id": plan["joint_id"], "live": current["live"],
                       "lock": current["lock"], "raw": current["raw"]}, output, indent=2)
            output.flush()
            os.fsync(output.fileno())
        backups.append(str(live_path))
        bus, name, writes = device.bus, plan["name"], []

        def write(register, value):
            if stop.is_set():
                raise RuntimeError("rebase interrupted")
            if int(bus.read("Torque_Enable", name, normalize=False, num_retry=2)):
                raise RuntimeError("torque unexpectedly ON; refusing EEPROM write")
            writes.append(register)
            bus.write(register, name, value, normalize=False, num_retry=0)

        self.write_started = True
        try:
            if current["lock"]:
                write("Lock", 0)
            write("Homing_Offset", plan["proposed"]["homing_offset"])
            fields = [("Min_Position_Limit", low), ("Max_Position_Limit", high)]
            if low >= current["live"][name]["range_max"]:
                fields.reverse()
            for register, value in fields:
                write(register, value)
            after = {key: calibration_record(value) for key, value in bus.read_calibration().items()}
            if after[name] != plan["proposed"] or {
                key: value for key, value in after.items() if key != name
            } != {key: value for key, value in current["live"].items() if key != name}:
                raise RuntimeError("selected/other-joint calibration readback verification failed")
            raw = int(bus.read("Present_Position", name, normalize=False, num_retry=2))
            if abs(raw - projected) > 20 or not low + 20 <= raw <= high - 20:
                raise RuntimeError("projected position was not verified after coordinate change")
            if current["lock"]:
                write("Lock", current["lock"])
            if int(bus.read("Lock", name, normalize=False, num_retry=2)) != current["lock"]:
                raise RuntimeError("EEPROM lock verification failed")
            if stop.is_set() or int(bus.read("Torque_Enable", name, normalize=False, num_retry=2)):
                raise RuntimeError("cancelled or torque changed before file update")
            _atomic_write(plan["profile_path"], (json.dumps(plan["proposed_profile"], indent=2) + "\n").encode(), 0o600)
            if stop.is_set():
                raise RuntimeError("interrupted between file updates")
            _atomic_write(plan["sdk_path"], (json.dumps(plan["proposed_sdk"], indent=2) + "\n").encode(),
                          stat.S_IMODE(plan["sdk_path"].stat().st_mode))
            if (
                json.loads(plan["sdk_path"].read_bytes()) != plan["proposed_sdk"]
                or json.loads(plan["profile_path"].read_bytes()) != plan["proposed_profile"]
            ):
                raise RuntimeError("file readback verification failed")
            return {
                "joint_id": plan["joint_id"], "after": after[name], "raw": raw,
                "torque_enabled": False, "writes": writes, "backups": backups,
                "pending_physical_zero": True,
                "maximum_is_provisional": plan["proposed_profile"]["joints"][name]["pending_range_max"],
            }
        except Exception as exc:  # noqa: BLE001 - never retry uncertain EEPROM outcomes.
            raise RuntimeError(
                f"Encoder rebase incomplete: {exc}. EEPROM/files may be partially changed. "
                f"No automatic retry/rollback. Keep torque OFF. Writes: {writes}. Backups: {backups}"
            ) from exc


def main():
    parser = argparse.ArgumentParser(description="Preview/confirm leader encoder rebasing; physical zero stays unset")
    parser.add_argument("--port", required=True)
    parser.add_argument("--arm-id", default="so101_leader")
    parser.add_argument("--joint", type=int, choices=range(1, 6), required=True)
    parser.add_argument("--old-start", type=int, required=True)
    parser.add_argument("--old-end", type=int)
    parser.add_argument("--provisional-max", type=int, help="unverified NEW maximum; degree/goal control stays blocked")
    parser.add_argument("--new-start", type=int, required=True)
    parser.add_argument("--expected-offset", type=int, required=True)
    args = parser.parse_args()
    device, connected = None, False
    manager = EncoderRebase()
    try:
        device = make_leader(args.port, args.arm_id)
        connected = True
        device.bus.connect()
        preview = manager.preview(
            device, args.joint, args.old_start, args.old_end, args.new_start, args.expected_offset,
            provisional_max=args.provisional_max,
        )
        print(json.dumps(preview, indent=2), flush=True)
        if input(f"Type REBASE LEADER J{args.joint} to confirm (Ctrl-C cancels): ").strip() != f"REBASE LEADER J{args.joint}":
            print("Cancelled; no EEPROM writes.")
            return
        result = manager.apply(device, preview["token"], threading.Event())
        print(json.dumps(result, indent=2))
        print("Encoder range verified. Torque stays OFF. Use raw ticks to measure physical zero next.")
        if result["maximum_is_provisional"]:
            print("MAX is provisional, not verified physical travel. Degree/goal control remains blocked.")
    except KeyboardInterrupt:
        print("\nInterrupted. If writes started, inspect hardware/files before any further action.")
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Encoder rebase stopped: {exc}\n")
    finally:
        if connected and device is not None:
            try:
                device.bus.disconnect(disable_torque=False)
            except Exception as exc:  # noqa: BLE001 - preserve the update outcome.
                print(f"Read-only disconnect failed: {exc}")


if __name__ == "__main__":
    main()
