import { JOINTS } from "./arm-viewer.js";

const $ = (id) => document.getElementById(id);
const text = (value) => value ?? "--";
const stamp = (value) => value ? new Date(value * 1000).toISOString() : "--";

export function createCalibrationPanel({ request, paired, getPose, controlStatus }) {
  let calibration = { running: false, state: "idle" };
  let online = false;
  let busy = false;
  let lastHeartbeat = 0;
  let lastEvidence = "";
  const cards = {};
  const lastReports = {};
  for (const side of ["leader", "follower"]) {
    const card = document.createElement("article");
    card.className = "calibration-card";
    $("calibration-inspectors").append(card);
    cards[side] = card;
  }
  JOINTS.forEach((joint, i) => {
    const option = document.createElement("option");
    option.value = joint.key;
    option.textContent = `J${i + 1} / ${joint.label}`;
    $("calibration-capture-joint").append(option);
  });

  function inspect(side, sample) {
    const report = sample?.calibration;
    const key = JSON.stringify(report ?? null);
    if (lastReports[side] === key) return;
    lastReports[side] = key;
    const card = cards[side];
    card.replaceChildren();
    const heading = document.createElement("h3");
    heading.textContent = `${side === "leader" ? "Leader" : "Follower"} / ${report?.state ?? "not inspected"}`;
    card.append(heading);
    const info = document.createElement("p");
    if (!report) {
      info.textContent = "Start monitoring this arm. If readings appear without an inspector, update both controller and follower.";
      card.append(info);
      return;
    }
    info.textContent = [
      `${text(report.arm_type)} / ID ${text(report.arm_id)}`,
      `Port: ${text(report.port)} / LeRobot ${text(report.lerobot_version)}`,
      `File: ${text(report.file.path)}`,
      `Modified: ${stamp(report.file.modified_at)}`,
      `File SHA256: ${text(report.file.sha256)}`,
      `Loaded SHA256: ${text(report.loaded_sha256)}`,
      `File matches loaded: ${text(report.file.matches_loaded)}`,
      `Inspected: ${stamp(report.inspected_at)} (register snapshot at monitor start)`,
      "Physical pose: NOT VALIDATED by register consistency",
      ...report.warnings,
    ].join("\n");
    card.append(info);
    JOINTS.forEach((joint, i) => {
      const item = report.joints[joint.key];
      if (!item) return;
      const details = document.createElement("details");
      const summary = document.createElement("summary");
      summary.textContent = `J${i + 1} ${joint.label} / ${text(item.mode)} / ${item.state}`;
      if (item.warnings.length) summary.className = "pose-mismatch";
      const values = document.createElement("pre");
      const record = (entry) => entry
        ? `ID ${entry.id}; drive ${entry.drive_mode}; offset ${entry.homing_offset}; range ${entry.range_min}..${entry.range_max}`
        : "UNAVAILABLE";
      const angularLimits = item.angular_calibration?.limits || item.working_range || item.servo_limits;
      values.textContent = [
        `Configured ID/model: ${item.id} / ${text(item.model)}`,
        `Detected model number: ${text(item.detected_model_number)}`,
        `Saved: ${record(item.saved)}`,
        `Live:  ${record(item.live)}`,
        "Live drive_mode is SDK metadata, not a verified hardware direction bit.",
        `Operating mode: ${text(item.operating_mode)} (expected 0)`,
        `Encoder resolution: ${text(item.resolution)} ticks/turn`,
        `Actual unit: ${text(item.unit)} / saved drive mode applied: ${item.drive_mode_applied}`,
        `Conversion reference: ${text(item.reference_ticks)} ticks -> ${text(item.reference_value)} ${text(item.unit)}`,
        `Converted endpoints: ${item.converted_range?.join(" .. ") ?? "--"} ${text(item.unit)}`,
        `Positive direction: ${text(item.positive_direction)}; verify physically`,
        ...(item.angular_calibration ? [
          `Angular calibration: ${item.angular_calibration.state} / ${item.angular_calibration.persistence}`,
          `Angular file: ${item.angular_calibration.file}`,
          `Angular SHA256: ${item.angular_calibration.sha256}`,
          `Limits source: ${item.angular_calibration.limits_source || "legacy_software"}`,
          `Calibrated ticks: ${angularLimits?.min ?? "--"}..${angularLimits?.max ?? "--"}`,
          `Degrees/tick: ${item.angular_calibration.degrees_per_tick}`,
          `Bound homing offset: ${item.angular_calibration.homing_offset}`,
        ] : []),
        ...item.warnings,
      ].join("\n");
      details.append(summary, values);
      card.append(details);
    });
  }

  function render() {
    const pose = getPose();
    inspect("leader", pose.leader);
    inspect("follower", pose.follower);
    const warnings = ["leader", "follower"].flatMap((side) => [
      ...(pose[side]?.motion_warnings || []).map((warning) => `${side}: ${warning}`),
      ...Object.entries(pose[side]?.conversion_errors || {}).map(([joint, error]) => `${side} ${joint}: ${error}`),
    ]);
    const control = controlStatus();
    $("calibration-start").disabled = !paired() || !online || busy || pose.running
      || control?.running || control?.estop_latched || calibration.running;
    $("calibration-stop").disabled = !paired() || !calibration.running || busy;
    $("calibration-reply").disabled = !paired() || !online || busy
      || !calibration.running || calibration.state !== "running";
    $("calibration-capture").disabled = !paired() || !pose.running || pose.state !== "ready"
      || pose.command_pending || busy || calibration.running;
    $("calibration-export").disabled = !paired() || (!pose.leader && !pose.follower && !pose.captures?.length);
    $("calibration-state").textContent = calibration.running ? `${calibration.side.toUpperCase()} / ${calibration.state.toUpperCase()}` : calibration.state.toUpperCase();
    $("calibration-output").textContent = calibration.output || "No calibration started.";
    if (calibration.error) $("calibration-message").textContent = calibration.error;
    const captures = pose.captures || [];
    let evidence = captures.length ? [
      `${captures.length} captured poses (export before controller restart)`,
      ...captures.map((c) => c.side === "both"
        ? `${c.label} / matched: ${Object.entries(c.deltas).map(([k, v]) => `${k.replace(".pos", "")} ${v.toFixed(2)}`).join("; ")}`
        : `${c.label} / ${c.side} ${c.joint}: expected ${c.expected}, observed ${c.observed.toFixed(2)}, error ${c.error.toFixed(2)} ${c.unit}, tolerance ${c.tolerance} / ${c.passed ? "within tolerance" : "REVIEW"}`),
      ...(pose.analysis || []).map((a) => `${a.side} ${a.joint}: ${a.points} points / ${a.state}`
        + (a.slope !== undefined ? ` / scale ${a.slope.toFixed(3)}, offset ${a.offset.toFixed(2)}` : "")
        + `\n${a.notes.join(" ")}`),
      "Captures from older calibration fingerprints are excluded from current fits. Full evidence is in the export.",
      ...warnings,
    ].join("\n") : "No physical validation captured. Evidence is kept in this controller process until cleared or restarted; export to retain it.";
    if (!captures.length && warnings.length) evidence += `\n${warnings.join("\n")}`;
    if (evidence !== lastEvidence) {
      $("calibration-evidence").textContent = evidence;
      lastEvidence = evidence;
    }
  }

  async function post(path, body) {
    busy = true;
    render();
    try {
      await request(path, { method: "POST", body: JSON.stringify(body) });
      return true;
    } catch (error) {
      $("calibration-message").textContent = error.message;
      return false;
    } finally {
      busy = false;
      render();
    }
  }
  $("calibration-capture-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    if ($("calibration-capture").disabled || !event.target.reportValidity()) return;
    const body = {
      side: $("calibration-capture-side").value,
      joint: $("calibration-capture-joint").value,
      expected: Number($("calibration-expected").value),
      tolerance: Number($("calibration-tolerance").value),
      label: $("calibration-capture-label").value,
      confirmed: $("calibration-capture-confirm").checked,
    };
    if (await post("/api/pose/capture", body)) $("calibration-message").textContent = "Pose evidence captured. No movement commanded.";
    $("calibration-capture-confirm").checked = false;
  });
  $("calibration-capture-side").addEventListener("change", () => {
    const matched = $("calibration-capture-side").value === "both";
    for (const id of ["calibration-capture-joint", "calibration-expected", "calibration-tolerance"]) {
      $(id).disabled = matched;
    }
  });
  $("calibration-export").addEventListener("click", () => {
    const pose = getPose();
    const blob = new Blob([JSON.stringify({
      version: 1, exported_at: new Date().toISOString(),
      note: "User-measured evidence; not a safety certification.",
      leader: pose.leader, follower: pose.follower, captures: pose.captures, analysis: pose.analysis,
    }, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `arm-calibration-${Date.now()}.json`;
    link.click();
    URL.revokeObjectURL(url);
  });
  $("calibration-clear").addEventListener("click", async () => {
    if (window.confirm("Clear captured physical evidence? Export first if you need to retain it.")) {
      await post("/api/pose/captures/clear", { confirmation: "CLEAR" });
    }
  });
  $("calibration-start-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    if ($("calibration-start").disabled || !event.target.reportValidity()) return;
    const side = $("calibration-set-side").value;
    if (!window.confirm(`Calibrate ONLY the ${side}? Torque may be released and saved calibration overwritten. Confirm physical support, a person at this arm and emergency stop access.`)) return;
    if (await post("/api/calibration/start", { side, confirmation: $("calibration-confirmation").value })) {
      $("calibration-message").textContent = "Follow the selected arm's LeRobot prompts. Restart monitoring after completion.";
      calibration.running = true;
      calibration.side = side;
      calibration.state = "starting";
      $("calibration-confirmation").value = "";
    }
  });
  $("calibration-stop").addEventListener("click", () => post("/api/calibration/stop", {}));
  $("calibration-input-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    if ($("calibration-reply").disabled) return;
    if (await post("/api/calibration/input", { text: $("calibration-input").value })) {
      $("calibration-input").value = "";
    }
  });

  async function poll() {
    if (paired()) {
      try {
        if (calibration.running && Date.now() - lastHeartbeat > 800) {
          await request("/api/calibration/heartbeat", { method: "POST" });
          lastHeartbeat = Date.now();
        }
        calibration = await request("/api/calibration/status");
        online = true;
      } catch {
        online = false;
        if (calibration.running) $("calibration-message").textContent = "Controller unavailable. Calibration watchdog cancels the procedure; inspect the arm.";
      }
    } else {
      online = false;
      calibration = { ...calibration, state: "idle", running: false };
    }
    render();
    window.setTimeout(poll, 300);
  }
  poll();
  return { render, get running() { return calibration.running || busy; } };
}
