import { JOINTS } from "./arm-viewer.js";

const $ = (id) => document.getElementById(id);
const display = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : "--";

export function createPoseMonitor({ request, paired, controlStatus, onChange }) {
  let status = { running: false };
  let online = false;
  let busy = false;
  let lastHeartbeat = 0;
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
    $("pose-start").disabled = !paired() || !online || busy || status.running
      || Boolean(control?.running || control?.estop_latched);
    $("pose-stop").disabled = !paired() || !status.running || status.state === "stopping";
    const fresh = status.follower && Date.now() / 1000 - status.follower.sampled_at < 1.5;
    $("pose-send").disabled = !online || busy || !status.running || status.state !== "ready"
      || status.command_pending || !fresh;
  }

  function render() {
    $("pose-state").textContent = online ? status.state.toUpperCase() : "OFFLINE";
    $("pose-tab-state").textContent = status.running ? ` / ${online ? status.state.toUpperCase() : "OFFLINE"}` : "";
    const follower = status.follower;
    const leader = status.leader;
    JOINTS.forEach((joint, index) => {
      const key = joint.key;
      const limits = (sample) => sample?.limits?.[key]
        ? `${sample.limits[key].min}..${sample.limits[key].max}` : "--";
      const torque = (sample) => typeof sample?.torque?.[key] === "boolean"
        ? sample.torque[key] ? "ON" : "off" : "--";
      const delta = Number.isFinite(leader?.values?.[key]) && Number.isFinite(follower?.values?.[key])
        ? leader.values[key] - follower.values[key] : null;
      const values = [
        index === 5 ? "0-100" : "deg",
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
    const torqueSummary = (side) => {
      const torque = status[`${side}_torque`];
      return `${side}: ${torque ? JOINTS.map((joint, index) =>
        `J${index + 1} ${torque.before[joint.key] ? "ON" : "off"} -> ${torque.after[joint.key] ? "ON" : "off"}`).join(", ") : "not read"}`;
    };
    $("pose-torque-summary").textContent = `Torque at monitor start (unchanged) - ${torqueSummary("leader")}; ${torqueSummary("follower")}`;
    const command = status.command;
    if (command) {
      $("pose-command-result").textContent = `#${command.command_id} / J${command.joint_id ?? "?"}: ${command.state || command.type}`
        + (command.message ? ` - ${command.message}` : "")
        + (command.type === "command_progress" ? ` (${display(command.current)} -> ${display(command.target)} ${command.unit || ""})` : "")
        + (typeof command.torque_enabled === "boolean" ? ` / torque ${command.torque_enabled ? "ON" : "off"}` : "");
    } else {
      $("pose-command-result").textContent = "No commands sent.";
    }
    if (status.error) $("pose-message").textContent = status.error;
    updateControls();
    onChange();
  }

  async function action(path, body) {
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
    if (await action("/api/pose/start", {})) {
      $("pose-message").textContent = "Monitor requested. Startup preserves torque; commands affect only the follower.";
    }
  });
  $("pose-stop").addEventListener("click", () => action("/api/pose/stop"));
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
  return { get running() { return status.running; } };
}
