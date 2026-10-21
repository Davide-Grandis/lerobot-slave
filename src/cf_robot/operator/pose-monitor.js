import { JOINTS, createArmViewer } from "./arm-viewer.js";
import { createCalibrationPanel } from "./calibration.js";

const $ = (id) => document.getElementById(id);
const display = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : "--";

export function createPoseMonitor({ request, paired, controlStatus, onChange }) {
  let status = { running: false };
  let online = false;
  let busy = false;
  let lastHeartbeat = 0;
  let restorePreview = null;
  let restoreExpires = 0;
  let restoreCommandId = null;
  const calibration = createCalibrationPanel({ request, paired, getPose: () => status, controlStatus });
  const models = Object.fromEntries(["leader", "follower"].map((side) => {
    const values = JOINTS.map((joint, index) => {
      const item = document.createElement("div");
      const label = document.createElement("span");
      label.textContent = `J${index + 1} / ${joint.label}`;
      const value = document.createElement("strong");
      value.textContent = "--";
      item.append(label, value);
      $(`pose-${side}-joints`).append(item);
      return value;
    });
    return [side, {
      viewer: createArmViewer($(`pose-${side}-canvas`), { solid: true }),
      values,
    }];
  }));
  const rows = JOINTS.map((joint, index) => {
    const row = document.createElement("tr");
    const name = document.createElement("th");
    name.scope = "row";
    name.textContent = `J${index + 1} / ${joint.key.replace(".pos", "")}`;
    row.append(name);
    const cells = Array.from({ length: 10 }, () => {
      const cell = document.createElement("td");
      cell.textContent = "--";
      row.append(cell);
      return cell;
    });
    $("pose-readings").append(row);
    return cells;
  });

  function updateControls() {
    const control = controlStatus();
    const startReason = !paired() ? "Pair the local controller above to enable monitoring."
      : !online ? "Waiting for the local controller. Check that cf-robot-web is running."
        : busy ? "Request in progress."
          : status.running ? "Monitor is already running."
            : calibration.running ? "Finish calibration maintenance first."
            : control?.estop_latched ? "Re-arm the software stop before starting."
              : control?.running ? "Disconnect teleoperation before starting monitoring."
                : "";
    $("pose-start").disabled = Boolean(startReason);
    $("pose-start").title = startReason;
    $("pose-start-reason").textContent = startReason;
    $("pose-start-reason").hidden = !startReason;
    $("pose-stop").disabled = !paired() || !status.running || status.state === "stopping";
    const fresh = status.follower && Date.now() / 1000 - status.follower.sampled_at < 1.5;
    const position = ["degrees", "ticks", "percent"].includes($("pose-command-mode").value);
    const consistent = status.follower?.calibration?.state === "consistent";
    $("pose-send").disabled = !online || busy || !status.running || status.state !== "ready"
      || status.command_pending || !fresh || (position && !consistent);
    const restoreReady = online && !busy && status.running && status.state === "ready"
      && !status.command_pending && fresh && status.follower?.torque?.["gripper.pos"] === false;
    $("pose-restore-preview").disabled = !paired() || !restoreReady;
    $("pose-restore-apply").disabled = !paired() || !restoreReady
      || !restorePreview || Date.now() >= restoreExpires;
  }

  function render() {
    $("pose-state").textContent = online ? status.state.toUpperCase() : "OFFLINE";
    $("pose-tab-state").textContent = status.running ? ` / ${online ? status.state.toUpperCase() : "OFFLINE"}` : "";
    const follower = status.follower;
    const leader = status.leader;
    for (const [side, model] of Object.entries(models)) {
      const sample = status[side];
      const valid = JOINTS.every(({ key }, index) => Number.isFinite(sample?.values?.[key])
        && (!sample.calibration || sample.calibration.joints[key]?.unit === (index === 5 ? "%" : "deg")));
      const fresh = valid && online && status.running
        && Date.now() / 1000 - sample.sampled_at <= 1.5;
      const modelState = !valid ? "waiting" : fresh ? "live" : "stale";
      $(`pose-${side}-model`).dataset.state = modelState;
      $(`pose-${side}-state`).textContent = !valid ? "NO SAMPLE" : fresh ? "LIVE" : "LAST SAMPLE";
      $(`pose-${side}-empty`).hidden = valid;
      model.viewer.setPoses(
        valid && side === "follower" ? sample.values : null,
        valid && side === "leader" ? sample.values : null,
      );
      model.values.forEach((element, index) => {
        const unit = sample?.calibration?.joints?.[JOINTS[index].key]?.unit;
        element.textContent = valid
          ? `${sample.values[JOINTS[index].key].toFixed(1)} ${unit || (index === 5 ? "/ 100" : "deg")}`
          : "--";
      });
    }
    JOINTS.forEach((joint, index) => {
      const key = joint.key;
      const limits = (sample) => {
        if (!sample?.limits?.[key]) return "--";
        const hardware = `${sample.limits[key].min}..${sample.limits[key].max}`;
        const working = sample.calibration?.joints?.[key]?.working_range;
        return working && (working.min !== sample.limits[key].min || working.max !== sample.limits[key].max)
          ? `${working.min}..${working.max} work / HW ${hardware}` : hardware;
      };
      const torque = (sample) => typeof sample?.torque?.[key] === "boolean"
        ? sample.torque[key] ? "V" : "X" : "--";
      const leaderMode = leader?.calibration?.joints[key]?.mode;
      const followerMode = follower?.calibration?.joints[key]?.mode;
      const compatible = !leaderMode || !followerMode || leaderMode === followerMode;
      const delta = compatible && Number.isFinite(leader?.values?.[key]) && Number.isFinite(follower?.values?.[key])
        ? leader.values[key] - follower.values[key] : null;
      const actualUnits = [...new Set([leader, follower].filter(Boolean).map((sample) =>
        sample.calibration ? sample.calibration.joints[key]?.unit || "unknown" : index === 5 ? "%" : "deg"))];
      const values = [
        actualUnits.length ? actualUnits.join(" / ") : index === 5 ? "%" : "deg",
        display(leader?.values?.[key]), display(follower?.values?.[key]), display(delta),
        display(leader?.raw?.[key], 0), limits(leader),
        display(follower?.raw?.[key], 0), limits(follower), torque(leader), torque(follower),
      ];
      rows[index].forEach((cell, column) => { cell.textContent = values[column]; });
      rows[index][3].classList.toggle("pose-mismatch", delta !== null && Math.abs(delta) > 5);
    });
    const age = (sample) => sample
      ? `${Math.max(0, Math.round(Date.now() - sample.sampled_at * 1000))} ms old` : "NO SAMPLE";
    $("pose-sample-age").textContent = `LEADER / ${age(leader)}   FOLLOWER / ${age(follower)}`;
    const command = status.command;
    if (command?.type === "command_result" && command.command_id !== restoreCommandId
      && ["calibration_preview", "calibration_restore"].includes(command.mode)) {
      restoreCommandId = command.command_id;
      restorePreview = command.calibration_preview || null;
      restoreExpires = Date.now() + (restorePreview?.expires_in_s || 0) * 1000;
      $("pose-restore-result").textContent = JSON.stringify(command, null, 2);
    }
    if (command) {
      $("pose-command-result").textContent = `#${command.command_id} / J${command.joint_id ?? "?"}: ${command.state || command.type}`
        + (command.message ? ` - ${command.message}` : "")
        + (command.type === "command_progress" ? ` (${display(command.current)} -> ${display(command.target)} ${command.unit || ""})` : "")
        + (typeof command.torque_enabled === "boolean" ? ` / torque ${command.torque_enabled ? "ON" : "off"}` : "");
    } else {
      $("pose-command-result").textContent = "No commands sent.";
    }
    if (status.error) $("pose-message").textContent = status.error;
    calibration.render();
    updateControls();
    onChange();
  }

  async function action(path, body) {
    restorePreview = null;
    busy = true;
    updateControls();
    try {
      await request(path, { method: "POST", ...(body ? { body: JSON.stringify(body) } : {}) });
      status = await request("/api/pose/status");
      online = true;
      render();
      return true;
    } catch (error) {
      $("pose-message").textContent = error.message;
      return false;
    } finally {
      busy = false;
      updateControls();
    }
  }

  $("pose-start").addEventListener("click", async () => {
    if (await action("/api/pose/start", { sides: $("pose-sides").value })) {
      $("pose-message").textContent = "Monitor started. Commands affect only the follower.";
    }
  });
  $("pose-stop").addEventListener("click", () => action("/api/pose/stop"));
  $("pose-restore-preview").addEventListener("click", async () => {
    if ($("pose-restore-preview").disabled) return;
    $("pose-restore-confirmation").value = "";
    $("pose-restore-supported").checked = false;
    await action("/api/pose/command", { joint_id: 6, mode: "calibration_preview" });
  });
  $("pose-restore-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    if ($("pose-restore-apply").disabled || !event.target.reportValidity()) return;
    const proposed = restorePreview.proposed;
    if (!window.confirm(`Write ONLY follower J6: offset ${proposed.homing_offset}, limits ${proposed.range_min}..${proposed.range_max}? Torque stays OFF; J1-J5 and the saved file are unchanged.`)) return;
    const body = {
      joint_id: 6, mode: "calibration_restore", token: restorePreview.token,
      confirmation: $("pose-restore-confirmation").value,
    };
    $("pose-restore-confirmation").value = "";
    $("pose-restore-supported").checked = false;
    await action("/api/pose/command", body);
  });
  const mode = $("pose-command-mode");
  const target = $("pose-value");
  function updateCommand() {
    const gripper = $("pose-joint").value === "6";
    mode.querySelector('[value="degrees"]').disabled = gripper;
    mode.querySelector('[value="percent"]').disabled = !gripper;
    if ((gripper && mode.value === "degrees") || (!gripper && mode.value === "percent")) {
      mode.value = gripper ? "percent" : "degrees";
    }
    const position = ["degrees", "ticks", "percent"].includes(mode.value);
    $("pose-value-label").hidden = !position;
    target.disabled = !position;
    target.required = position;
    target.step = mode.value === "ticks" ? "1" : "any";
    target.removeAttribute("min");
    target.removeAttribute("max");
    if (mode.value === "ticks" || mode.value === "percent") {
      target.min = "0";
      target.max = mode.value === "ticks" ? "4095" : "100";
    }
    target.value = "";
    $("pose-command-confirm").checked = false;
  }
  $("pose-joint").addEventListener("change", updateCommand);
  mode.addEventListener("change", updateCommand);
  $("pose-command-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    if ($("pose-send").disabled || !$("pose-command-form").reportValidity()) return;
    const body = { joint_id: Number($("pose-joint").value), mode: mode.value };
    if (["enable", "disable"].includes(mode.value)) {
      body.mode = "torque";
      body.enabled = mode.value === "enable";
    } else {
      body.value = Number(target.value);
    }
    $("pose-command-confirm").checked = false;
    await action("/api/pose/command", body);
  });

  const tabs = [$("teleop-tab"), $("pose-tab")];
  function selectTab(selected) {
    tabs.forEach((tab) => {
      const active = tab === selected;
      tab.setAttribute("aria-selected", String(active));
      tab.tabIndex = active ? 0 : -1;
      $(tab.getAttribute("aria-controls")).hidden = !active;
    });
    window.dispatchEvent(new Event("resize"));
  }
  tabs.forEach((tab, index) => {
    tab.addEventListener("click", () => selectTab(tab));
    tab.addEventListener("keydown", (event) => {
      if (!["ArrowLeft", "ArrowRight", "Home", "End"].includes(event.key)) return;
      event.preventDefault();
      const selected = tabs[event.key === "Home" ? 0 : event.key === "End" ? 1 : 1 - index];
      selectTab(selected);
      selected.focus();
    });
  });
  async function poll() {
    if (paired()) {
      try {
        if (status.running && Date.now() - lastHeartbeat > 800) {
          await request("/api/pose/heartbeat", { method: "POST" });
          lastHeartbeat = Date.now();
        }
        status = await request("/api/pose/status");
        online = true;
        render();
      } catch (error) {
        online = false;
        $("pose-message").textContent = `Controller unavailable: ${error.message}. The heartbeat watchdog stops the monitor; inspect the arm before restarting.`;
        render();
      }
    } else {
      online = false;
      updateControls();
    }
    window.setTimeout(poll, 200);
  }
  updateCommand();
  poll();
  return { get running() { return status.running || calibration.running; } };
}
