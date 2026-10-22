"""Explicit local leader hardware-limit update with measured angular references."""

import argparse
import json
import sys
import threading

from cf_robot.angular_install import AngularInstall
from cf_robot.hardware import make_leader


def main():
    parser = argparse.ArgumentParser(
        description="Preview/confirm one leader joint's EEPROM MIN/MAX; no homing or goal changes",
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--arm-id", default="so101_leader")
    parser.add_argument("--joint", type=int, choices=range(1, 6), default=5)
    parser.add_argument("--min", dest="minimum", type=int, required=True)
    parser.add_argument("--max", dest="maximum", type=int, required=True)
    parser.add_argument(
        "--zero-ticks", type=float,
        help="measured zero; alone this uses 360/encoder-resolution degrees per tick",
    )
    parser.add_argument("--reference-ticks", type=int, help="encoder ticks at a second measured angle")
    parser.add_argument("--reference-degrees", type=float, help="nonzero angle of the second reference")
    args = parser.parse_args()
    print(
        "Stop ARMS MONITOR/teleoperation first. Secure/support the leader.\n"
        "The selected joint must already have torque OFF. No automatic release."
    )
    device = None
    manager = AngularInstall("leader")
    connected = False
    try:
        device = make_leader(args.port, args.arm_id)
        connected = True
        device.bus.connect()
        preview = manager.preview_limits(
            device, args.joint, args.minimum, args.maximum,
            zero_ticks=args.zero_ticks, reference_ticks=args.reference_ticks,
            reference_degrees=args.reference_degrees,
        )
        preview["instruction"] = f"Type APPLY J{args.joint} LIMITS in this terminal"
        print(json.dumps(preview, indent=2))
        confirmation = input(
            f"Write ONLY J{args.joint} Min/Max EEPROM and matching files?\n"
            f"Type APPLY J{args.joint} LIMITS (Ctrl-C cancels): ",
        )
        if confirmation.strip() != f"APPLY J{args.joint} LIMITS":
            print("Cancelled; no hardware limits were written.")
            return
        result = manager.apply(device, {
            "joint_id": args.joint, "token": preview["token"],
            "confirmation": f"APPLY ANGULAR AND LIMITS J{args.joint}",
        }, threading.Event())
        print(json.dumps(result, indent=2))
        print("Hardware limits verified and saved. Torque remains OFF; restart monitoring.")
    except KeyboardInterrupt:
        if manager.write_started:
            print(
                "\nInterrupted after update began. EEPROM/files may be partially changed.\n"
                "Keep torque OFF and inspect readback/backups before use.\n"
                "No rollback was attempted."
            )
        else:
            print("\nCancelled before writing; no EEPROM limits were changed.")
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"Leader limit update stopped: {exc}\n")
    finally:
        if connected and device is not None:
            try:
                device.bus.disconnect(disable_torque=False)
            except Exception as exc:  # noqa: BLE001 - preserve the original update outcome.
                print(f"Read-only disconnect failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
