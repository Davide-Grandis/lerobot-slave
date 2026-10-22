# Follower Encoder Rebase Over WSS

Encoder rebasing uses the authenticated `/monitor` connection, with the same
exclusive hardware ownership as pose monitoring. It does not need SSH.
The Pi must first be updated to a version supporting `rebase_preview` and
`rebase_apply`; installing the Mac code alone does not enable these commands.

Unlike angular-reference installation, rebasing explicitly changes the selected
servo's homing offset as well as its MIN/MAX EEPROM registers. Support the arm
first. Torque must already be OFF. The projected position must be at least
20 ticks inside both proposed boundaries. No position goals or torque changes
are issued by the rebase operation.

## Preview And Confirm

After the normal `{"version":1,"type":"start"}` monitor request, send:

```json
{
  "version": 1,
  "type": "command",
  "command_id": 1,
  "joint_id": 2,
  "mode": "rebase_preview",
  "old_start": 3803,
  "new_start": 500,
  "provisional_max": 4095,
  "expected_offset": -1902
}
```

`old_start` is the independently measured physical minimum in the current
encoder coordinate system, not the current resting position or a factory
MIN register. `expected_offset` must match the live homing offset; this guards
against applying the same coordinate change twice. A measured `old_end` may
replace `provisional_max`, but the two fields cannot be supplied together.

For these J2 measurements the tick shift is +793, and the equivalent supported
homing offset is 1401. Offsets are reduced modulo 4096 into the supported signed
register range. For example, an old reading of 787 projects to 1580. These
are coordinate changes, not commanded physical motion.

Review the returned `rebase_preview`, then send its one-use token with
`mode: "rebase_apply"` and `confirmation: "REBASE FOLLOWER J2"` using a new
command ID. Confirmation must arrive within 60 seconds with unchanged files,
registers and a stationary joint. Another command invalidates the preview.

Original files and live settings are backed up before writing. The operation
verifies selected settings, projected position, EEPROM lock and unchanged other
joints before saving matching SDK/profile files. It then stops the monitor.
An uncertain write is never automatically retried or rolled back.

Reconnect to inspect the new raw ticks. A provisional maximum of 4095 is only
a temporary ceiling, not verified physical travel. `pending_zero` and
`pending_range_max` remain set; normalized readings and position goals stay
blocked until the physical maximum and zero are measured and registered.
Keep torque OFF during those manual measurements and never force mechanical
stops.
