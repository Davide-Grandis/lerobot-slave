import { HOME_POSE, JOINTS, alignmentDelta, createArmViewer, forwardKinematics } from "./arm-viewer.js";
import { createPoseMonitor } from "./pose-monitor.js";

const savedCameraRotation = Number(localStorage.getItem("cf-robot-video-rotation"));
const state = {
  controllerBase: localStorage.getItem("cf-robot-controller-origin")
    || (["localhost", "127.0.0.1"].includes(location.hostname) && location.port === "8080"
      ? location.origin
      : "http://127.0.0.1:8080"),
  pairingCode: "",
  paired: false,
  requestController: null,
  agent: null,
  robotId: "so101-main",
  sessionId: null,
  manualPose: null,
  manualBusy: false,
  mode: "manual",
  pollTimer: null,
  videoTimer: null,
  frameUrl: null,
  lastFrameAt: null,
  cameraRotation: [0, 90, 180, 270].includes(savedCameraRotation) ? savedCameraRotation : 0,
  cameraWidth: 0,
  cameraHeight: 0,
};

const $ = (id) => document.getElementById(id);
const elements = {
  pairingPanel: $("pairing-panel"),
  pairingOrigin: $("controller-origin"),
  pairingCode: $("pairing-code"),
  pairingStatus: $("pairing-status"),
  pairController: $("pair-controller"),
  disconnectController: $("disconnect-controller"),
  cameraImage: $("camera-image"),
  cameraFrame: $("camera-frame"),
  cameraRotate: $("camera-rotate"),
  cameraPlaceholder: $("camera-placeholder"),
  cameraMessage: $("camera-message"),
  cameraState: $("camera-state"),
  cameraClock: $("camera-clock"),
  cameraAge: $("camera-age"),
  sessionState: $("session-state"),
  sessionDetail: $("session-detail"),
  leaderConnection: $("leader-connection"),
  alignmentState: $("alignment-state"),
  start: $("start-control"),
  stop: $("stop-control"),
  estop: $("software-stop"),
  leaderConnect: $("leader-connect"),
  leaderDisconnect: $("leader-disconnect"),
  manualMode: $("manual-mode"),
  leaderMode: $("leader-mode"),
  modeChip: $("mode-chip"),
  jogStep: $("jog-step"),
  jogList: $("jog-list"),
  readings: $("joint-readings"),
  canvas: $("arm-canvas"),
  modelEmpty: $("model-empty"),
  toolPosition: $("tool-position"),
  latency: $("latency-value"),
  commandCount: $("command-count"),
  feedbackAge: $("feedback-age"),
  identity: $("operator-identity"),
  rearmDialog: $("rearm-dialog"),
  rearmConfirmation: $("rearm-confirmation"),
};

const viewer = createArmViewer(elements.canvas, { solid: true });
let poseMonitor = null;

function localControllerUrl() {
  let url;
  try {
    url = new URL(elements.pairingOrigin.value.trim());
  } catch {
    throw new Error("Enter a valid local controller address.");
  }
  if (!["http:", "https:"].includes(url.protocol)
      || !["localhost", "127.0.0.1"].includes(url.hostname)
      || url.pathname !== "/" || url.search || url.hash) {
    throw new Error("The controller address must be a loopback URL, for example http://127.0.0.1:8080.");
  }
  return url.origin;
}

function bridgeHeaders() {
  const headers = {};
  if (state.pairingCode) headers["X-CF-Robot-Bridge"] = state.pairingCode;
  return headers;
}

async function agentRequest(path, options = {}) {
  if (!state.paired) throw new Error("Pair the local controller first.");
  const controller = state.requestController;
  const response = await fetch(`${state.controllerBase}${path}`, {
    cache: "no-store",
    ...options,
    headers: {
      ...bridgeHeaders(),
      ...(options.body ? { "Content-Type": "application/json" } : {}),
      ...options.headers,
    },
    signal: controller.signal,
  });
  const payload = response.headers.get("content-type")?.includes("application/json")
    ? await response.json()
    : null;
  if (!state.paired || controller !== state.requestController) {
    throw new DOMException("Controller disconnected.", "AbortError");
  }
  if (!response.ok) throw new Error(payload?.detail || payload?.error || `Controller returned HTTP ${response.status}.`);
  return payload;
}

async function workerRequest(path, options = {}) {
  const response = await fetch(path, {
    cache: "no-store",
    ...options,
    headers: {
      ...(options.body ? { "Content-Type": "application/json" } : {}),
      ...options.headers,
    },
  });
  if (!response.ok) {
    const payload = await response.json().catch(() => null);
    throw new Error(payload?.error || `Operator app returned HTTP ${response.status}.`);
  }
  return response.json();
}

function announce(text, tone = "") {
  elements.pairingStatus.textContent = text;
  elements.pairingStatus.dataset.tone = tone;
}

function setCameraLive(live, message = "") {
  elements.cameraState.classList.toggle("connected", live);
  elements.cameraState.lastElementChild.textContent = live ? "Live" : "Waiting";
  elements.cameraPlaceholder.classList.toggle("hidden", live);
  elements.cameraImage.classList.toggle("visible", live);
  if (message) elements.cameraMessage.textContent = message;
  elements.cameraClock.textContent = live ? "LIVE / 10 HZ" : "NO SIGNAL";
  if (!live) {
    elements.cameraAge.textContent = "—";
  }
}

function updateCameraRotation() {
  const angle = state.cameraRotation;
  elements.cameraFrame.dataset.rotation = String(angle);
  elements.cameraImage.style.setProperty("--camera-rotation", `${angle}deg`);
  elements.cameraRotate.title = `Rotate clockwise by 90 degrees (current orientation: ${angle} degrees)`;
  elements.cameraRotate.setAttribute("aria-label", elements.cameraRotate.title);
  const quarterTurn = angle % 180 !== 0;
  if (state.cameraWidth && state.cameraHeight) {
    elements.cameraFrame.style.aspectRatio = quarterTurn
      ? `${state.cameraHeight} / ${state.cameraWidth}`
      : `${state.cameraWidth} / ${state.cameraHeight}`;
  }
  const { width, height } = elements.cameraFrame.getBoundingClientRect();
  if (!width || !height) return;
  // Swap the contain box for quarter turns so portrait and landscape feeds both fit.
  elements.cameraImage.style.width = `${quarterTurn ? height : width}px`;
  elements.cameraImage.style.height = `${quarterTurn ? width : height}px`;
}

function updateTheme() {
  const dark = localStorage.getItem("cf-robot-theme") !== "light";
  document.documentElement.dataset.theme = dark ? "dark" : "light";
  document.querySelector('meta[name="theme-color"]').content = dark ? "#18211f" : "#e9e6d9";
  $("theme-toggle").textContent = dark ? "☼" : "◐";
}

function makeJogControls() {
  elements.jogList.innerHTML = "";
  elements.readings.innerHTML = "";
  for (const [index, joint] of JOINTS.entries()) {
    const row = document.createElement("div");
    row.className = "jog-row";
    row.innerHTML = `
      <div class="jog-joint"><span class="joint-index">0${index + 1}</span><strong>${joint.label}</strong></div>
      <button class="jog-button" type="button" data-joint="${index}" data-direction="-1" aria-label="Decrease ${joint.label}" disabled>−</button>
      <button class="jog-button" type="button" data-joint="${index}" data-direction="1" aria-label="Increase ${joint.label}" disabled>+</button>
      <span class="jog-value" id="jog-value-${index}">—</span>
    `;
    elements.jogList.append(row);

    const reading = document.createElement("div");
    reading.className = "reading-row";
    reading.innerHTML = `
      <span class="reading-name"><i>0${index + 1}</i>${joint.label}</span>
      <span class="reading-delta" id="reading-delta-${index}">—</span>
      <strong id="reading-value-${index}">—</strong>
    `;
    elements.readings.append(reading);
  }
  elements.jogList.addEventListener("click", jogClick);
}

function poseMatches(pose) {
  return pose && JOINTS.every((joint) => Number.isFinite(pose[joint.key]));
}

function formatValue(joint, value) {
  if (!Number.isFinite(value)) return "—";
  return `${Number(value).toFixed(1)}${joint.unit}`;
}

function renderMeasurements(status) {
  const follower = status.running && poseMatches(status.follower_positions) ? status.follower_positions : null;
  const leader = follower && status.leader_connected && poseMatches(status.leader_positions)
    ? status.leader_positions : null;
  const age = Date.now() / 1000 - status.follower_observed_at;
  const fresh = follower && Number.isFinite(status.follower_observed_at) && age >= 0 && age <= 1.5;
  const delta = alignmentDelta(leader, follower);
  const modelState = !follower ? "home" : fresh ? "live" : "stale";
  if (elements.canvas.dataset.state !== modelState) {
    elements.canvas.dataset.state = modelState;
    elements.modelEmpty.textContent = !follower ? "HOME REFERENCE / NOT LIVE"
      : fresh ? "LIVE FOLLOWER / MEASURED" : "LAST SAMPLE / NOT LIVE";
  }
  viewer.setPoses(follower || HOME_POSE, leader);
  if (poseMatches(follower)) {
    const point = forwardKinematics(follower).at(-1);
    elements.toolPosition.textContent = point.map((value) => `${(value * 1000).toFixed(0)}`).join(" / ") + " mm";
    state.manualPose = state.manualBusy ? state.manualPose : { ...follower };
  } else {
    elements.toolPosition.textContent = "—";
    state.manualPose = null;
  }

  JOINTS.forEach((joint, index) => {
    const value = follower?.[joint.key];
    $(`jog-value-${index}`).textContent = formatValue(joint, value);
    $(`reading-value-${index}`).textContent = formatValue(joint, value);
    const deltaElement = $(`reading-delta-${index}`);
    if (delta && Number.isFinite(delta[joint.key])) {
      deltaElement.textContent = `${delta[joint.key] >= 0 ? "+" : ""}${delta[joint.key].toFixed(1)}${joint.unit}`;
      deltaElement.classList.toggle("matched", Math.abs(delta[joint.key]) <= 5);
      deltaElement.classList.toggle("mismatched", Math.abs(delta[joint.key]) > 5);
    } else {
      deltaElement.textContent = "—";
      deltaElement.classList.remove("matched", "mismatched");
    }
  });
}

function updateButtons(status) {
  const connected = Boolean(state.paired && status.running && status.state === "ready");
  const cameraLive = Boolean(status.camera_live);
  const browserCameraLive = state.lastFrameAt !== null && Date.now() - state.lastFrameAt <= 1500;
  const stopped = Boolean(status.estop_latched);
  elements.start.disabled = !state.paired || !cameraLive || !browserCameraLive || status.running || stopped || Boolean(poseMonitor?.running);
  elements.stop.disabled = !status.running;
  elements.estop.disabled = !state.paired || (status.running || poseMonitor?.running ? stopped : !stopped);
  elements.estop.textContent = stopped ? "↻  RE-ARM / INSPECT" : "■   SOFTWARE STOP";
  elements.leaderConnect.disabled = !connected || status.leader_connected || state.manualBusy;
  elements.leaderDisconnect.disabled = !connected || !status.leader_connected || state.manualBusy;
  elements.manualMode.disabled = !connected || state.mode === "manual";
  elements.leaderMode.disabled = !connected || state.mode === "leader";
  elements.jogStep.disabled = !connected || state.mode !== "manual" || state.manualBusy;
  elements.jogList.querySelectorAll(".jog-button").forEach((button) => {
    button.disabled = !connected || state.mode !== "manual" || state.manualBusy || stopped;
  });
  elements.sessionState.textContent = ["synchronizing", "waiting_feedback"].includes(status.state) ? "WAIT"
    : status.state === "ready" && status.mode === "leader" ? "Ready for teleop"
      : String(status.state || "idle").replaceAll("_", " ").toUpperCase();
  elements.sessionState.dataset.state = status.state || "idle";
  elements.sessionDetail.textContent = status.error || status.teleop_message
    || (status.running
      ? status.mode === "leader"
        ? "Leader streaming is active. Monitor the measured follower pose and camera."
        : "Manual mode is active. Each on-screen jog sends one bounded target."
      : cameraLive
        ? "Camera is live. Start teleoperation through the local controller when the workspace is clear."
        : "Live follower video is required before starting teleoperation.");
  elements.leaderConnection.textContent = status.leader_connected ? "CONNECTED" : "NOT CONNECTED";
  elements.alignmentState.textContent = !status.leader_connected
    ? "NOT CHECKED"
    : status.aligned ? "WITHIN 5 UNITS" : "ALIGN ARMS";
  elements.alignmentState.classList.toggle("safe", Boolean(status.aligned));
  elements.modeChip.textContent = state.mode.toUpperCase();
  elements.commandCount.textContent = String(status.commands_sent ?? 0);
  if (status.follower_observed_at) {
    const age = Math.max(0, (Date.now() / 1000 - status.follower_observed_at) * 1000);
    elements.feedbackAge.textContent = `${Math.round(age)} ms`;
  } else {
    elements.feedbackAge.textContent = "—";
  }
  if (stopped) {
    elements.start.textContent = "Re-arm required";
    elements.start.disabled = true;
  } else {
    elements.start.innerHTML = 'Start teleoperation <span>↗</span>';
  }
  if (status.error) elements.pairingStatus.textContent = status.error;
  renderMeasurements(status);
}

async function pollStatus() {
  if (!state.paired) return;
  const controller = state.requestController;
  const requestedAt = performance.now();
  try {
    const status = await agentRequest("/api/status");
    elements.latency.textContent = `${Math.round(performance.now() - requestedAt)} ms`;
    const resumedManual = state.agent?.state === "waiting_feedback"
      && status.state === "ready" && status.mode === "manual";
    state.agent = status;
    state.mode = status.mode || "manual";
    if (resumedManual && poseMatches(status.follower_positions)) {
      state.manualPose = { ...status.follower_positions };
    }
    updateButtons(status);
    if (!status.running && state.sessionId) {
      await recordSessionEnd(status.state === "error" ? "error" : "stopped");
    }
    if (!state.paired || controller !== state.requestController) return;
    const frameAt = status.camera_received_at ? Number(status.camera_received_at) * 1000 : null;
    elements.cameraAge.textContent = frameAt ? `${Math.max(0, Math.round(Date.now() - frameAt))} ms old` : "—";
    if (!status.camera_live) setCameraLive(false, status.camera_error || "Camera unavailable. Control remains disarmed.");
  } catch (error) {
    if (!state.paired || controller !== state.requestController) return;
    state.agent = null;
    renderMeasurements({});
    setCameraLive(false, "Controller or camera unavailable. Check the local bridge.");
    elements.sessionState.textContent = "OFFLINE";
    elements.sessionDetail.textContent = error.message;
    elements.start.disabled = true;
    elements.stop.disabled = true;
  }
}

async function pollFrame() {
  if (!state.paired) return;
  const controller = state.requestController;
  if ($("teleop-panel").hidden && !state.agent?.running) {
    state.videoTimer = window.setTimeout(pollFrame, 300);
    return;
  }
  try {
    const response = await fetch(`${state.controllerBase}/api/video/frame`, {
      cache: "no-store",
      headers: bridgeHeaders(),
      signal: controller.signal,
    });
    if (!response.ok) throw new Error("camera frame unavailable");
    const blob = await response.blob();
    if (!state.paired || controller !== state.requestController) return;
    const nextUrl = URL.createObjectURL(blob);
    try {
      const frame = new Image();
      frame.src = nextUrl;
      await frame.decode();
      if (!state.paired || controller !== state.requestController) {
        URL.revokeObjectURL(nextUrl);
        return;
      }
      if (state.cameraWidth !== frame.naturalWidth || state.cameraHeight !== frame.naturalHeight) {
        state.cameraWidth = frame.naturalWidth;
        state.cameraHeight = frame.naturalHeight;
        updateCameraRotation();
      }
    } catch (error) {
      URL.revokeObjectURL(nextUrl);
      throw error;
    }
    if (state.frameUrl) URL.revokeObjectURL(state.frameUrl);
    state.frameUrl = nextUrl;
    state.lastFrameAt = Date.now();
    elements.cameraImage.src = nextUrl;
    elements.cameraAge.textContent = "LIVE";
    setCameraLive(true);
  } catch {
    if (!state.paired || controller !== state.requestController) return;
    if (state.lastFrameAt === null || Date.now() - state.lastFrameAt > 1500) {
      setCameraLive(false, "Waiting for follower camera frames…");
    }
  } finally {
    if (state.paired && controller === state.requestController) {
      state.videoTimer = window.setTimeout(pollFrame, 100);
    }
  }
}

async function pollLoop() {
  const controller = state.requestController;
  await pollStatus();
  if (state.paired && controller === state.requestController) {
    state.pollTimer = window.setTimeout(pollLoop, 100);
  }
}

async function heartbeat() {
  if (state.paired && state.agent?.running) {
    const cameraLive = state.lastFrameAt !== null && Date.now() - state.lastFrameAt <= 1500;
    agentRequest("/api/heartbeat", {
      method: "POST",
      body: JSON.stringify({ camera_live: cameraLive }),
    }).catch(() => {});
  }
  window.setTimeout(heartbeat, 1000);
}

async function connectWorkerMetadata() {
  try {
    await workerRequest("/api/me");
    elements.identity.textContent = "OPERATOR";
    const { robots } = await workerRequest("/api/robots");
    const robot = robots?.[0];
    if (robot) {
      state.robotId = robot.id;
    }
  } catch {
    elements.identity.textContent = "OPERATOR";
  }
}

async function pairController() {
  if (state.paired) return;
  const controller = new AbortController();
  state.requestController = controller;
  elements.pairController.disabled = true;
  try {
    state.controllerBase = localControllerUrl();
    state.pairingCode = elements.pairingCode.value.trim();
    if (location.origin !== state.controllerBase && !state.pairingCode) {
      throw new Error("Paste the one-run pairing code from the controller terminal.");
    }
    localStorage.setItem("cf-robot-controller-origin", state.controllerBase);
    state.paired = true;
    await agentRequest("/api/config");
    elements.pairingPanel.classList.add("paired");
    const monitoring = $("teleop-panel").hidden;
    announce(monitoring ? "Controller paired. Ready to monitor." : "Controller paired. Opening camera…", "success");
    if (!state.pollTimer) pollLoop();
    if (!state.videoTimer) pollFrame();
  } catch (error) {
    if (controller !== state.requestController) return;
    state.paired = false;
    controller.abort();
    announce(error.message, "error");
  } finally {
    if (controller === state.requestController) elements.pairController.disabled = false;
  }
}

async function disconnectController() {
  if (!state.paired) return;
  const base = state.controllerBase;
  const headers = bridgeHeaders();
  state.paired = false;
  state.requestController.abort();
  state.requestController = null;
  window.clearTimeout(state.pollTimer);
  window.clearTimeout(state.videoTimer);
  state.pollTimer = null;
  state.videoTimer = null;
  void recordSessionEnd("stopped");
  state.agent = null;
  state.mode = "manual";
  state.pairingCode = "";
  elements.pairingCode.value = "";
  state.lastFrameAt = null;
  state.cameraWidth = 0;
  state.cameraHeight = 0;
  if (state.frameUrl) URL.revokeObjectURL(state.frameUrl);
  state.frameUrl = null;
  elements.cameraImage.removeAttribute("src");
  elements.cameraFrame.style.removeProperty("aspect-ratio");
  elements.pairingPanel.classList.remove("paired");
  elements.pairController.disabled = true;
  updateButtons({});
  setCameraLive(false, "Pair the controller to open its camera feed.");
  announce("Disconnecting controller...");
  const stopRequest = (path) => fetch(`${base}${path}`, {
    method: "POST", headers, signal: AbortSignal.timeout(3000),
  });
  try {
    const response = await stopRequest("/api/disconnect");
    if (response.status === 404) {
      const stops = await Promise.all(
        ["/api/stop", "/api/pose/stop", "/api/calibration/stop"].map(stopRequest),
      );
      if (stops.some((stop) => !stop.ok)) throw new Error("Stop request rejected.");
      announce("Disconnected. Stops requested; update/restart the bridge to release its camera.", "success");
    } else {
      if (!response.ok) throw new Error("Disconnect request rejected.");
      announce("Controller disconnected. Active sessions asked to stop.", "success");
    }
  } catch {
    announce("Browser disconnected. Stop could not be confirmed; inspect the robot. Heartbeat watchdogs remain active.", "error");
  } finally {
    elements.pairController.disabled = false;
  }
}

async function recordSessionStart() {
  if (location.hostname === "localhost" || location.hostname === "127.0.0.1") return;
  state.sessionId = crypto.randomUUID();
  try {
    await workerRequest("/api/sessions", {
      method: "POST",
      body: JSON.stringify({
        id: state.sessionId,
        robot_id: state.robotId,
        mode: state.mode,
      }),
    });
  } catch (error) {
    elements.sessionDetail.textContent = `Session audit could not be recorded: ${error.message}`;
    state.sessionId = null;
  }
}

async function recordSessionEnd(stateName) {
  if (!state.sessionId) return;
  const id = state.sessionId;
  state.sessionId = null;
  try {
    await workerRequest(`/api/sessions/${id}`, {
      method: "PATCH",
      body: JSON.stringify({
        state: stateName,
        command_count: state.agent?.commands_sent ?? 0,
      }),
    });
  } catch (error) {
    console.warn("Could not record session end:", error);
  }
}

async function startControl() {
  try {
    elements.start.disabled = true;
    await agentRequest("/api/start", {
      method: "POST",
      body: JSON.stringify({ camera_live: state.lastFrameAt !== null && Date.now() - state.lastFrameAt <= 1500 }),
    });
    await recordSessionStart();
    announce("Teleoperation requested through the local controller.", "success");
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
    await pollStatus();
  }
}

async function stopControl() {
  try {
    await agentRequest("/api/stop", { method: "POST" });
    await recordSessionEnd("stopped");
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
  }
}

async function emergencyStop() {
  try {
    await agentRequest("/api/estop", { method: "POST" });
    await recordSessionEnd("stopped");
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
  }
}

async function changeMode(mode) {
  if (mode === "leader" && !window.confirm(
    "The follower will synchronize to the leader, one joint at a time. "
    + "Secure both arms, clear the workspace and hold the leader still. Continue?",
  )) return;
  try {
    const result = await agentRequest("/api/mode", {
      method: "POST",
      body: JSON.stringify({ mode, confirm_sync: mode === "leader" }),
    });
    state.mode = result.mode;
    elements.manualMode.classList.toggle("active", state.mode === "manual");
    elements.leaderMode.classList.toggle("active", state.mode === "leader");
    elements.manualMode.setAttribute("aria-selected", String(state.mode === "manual"));
    elements.leaderMode.setAttribute("aria-selected", String(state.mode === "leader"));
    elements.modeChip.textContent = state.mode.toUpperCase();
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
  }
}

async function connectLeader() {
  try {
    elements.leaderConnect.disabled = true;
    elements.leaderConnect.textContent = "Connecting…";
    await agentRequest("/api/leader/connect", { method: "POST" });
    announce("Leader connected. Select Leader arm to confirm startup synchronization.", "success");
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
  } finally {
    await pollStatus();
  }
}

async function disconnectLeader() {
  try {
    elements.leaderDisconnect.disabled = true;
    await agentRequest("/api/leader/disconnect", { method: "POST" });
    state.mode = "manual";
    announce("Leader disconnected; its USB port is released.", "success");
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
  } finally {
    await pollStatus();
  }
}

async function jogClick(event) {
  const button = event.target.closest(".jog-button");
  if (!button || button.disabled || state.manualBusy) return;
  const index = Number(button.dataset.joint);
  const joint = JOINTS[index];
  const base = state.manualPose || state.agent?.follower_positions;
  if (!joint || !poseMatches(base)) return;
  const target = { ...base };
  const direction = Number(button.dataset.direction);
  const step = Number(elements.jogStep.value);
  target[joint.key] = Math.max(joint.min, Math.min(joint.max, Number(target[joint.key]) + direction * step));
  if (target[joint.key] === Number(base[joint.key])) return;
  state.manualBusy = true;
  updateButtons(state.agent || {});
  try {
    await agentRequest("/api/manual", {
      method: "POST",
      body: JSON.stringify({ action: target }),
    });
    state.manualPose = target;
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
  } finally {
    state.manualBusy = false;
    updateButtons(state.agent || {});
  }
}

async function rearm() {
  if (elements.rearmConfirmation.value.trim() !== "RE-ARM") {
    elements.rearmConfirmation.setCustomValidity("Enter RE-ARM exactly.");
    elements.rearmConfirmation.reportValidity();
    return;
  }
  elements.rearmConfirmation.setCustomValidity("");
  try {
    await agentRequest("/api/estop/rearm", {
      method: "POST",
      body: JSON.stringify({ confirmation: "RE-ARM" }),
    });
    elements.rearmDialog.close();
    elements.rearmConfirmation.value = "";
    announce("Software latch cleared. Reconnect only after confirming the area is safe.", "success");
    await pollStatus();
  } catch (error) {
    announce(error.message, "error");
  }
}

elements.pairingOrigin.value = state.controllerBase;
makeJogControls();
renderMeasurements({});
updateCameraRotation();
new ResizeObserver(updateCameraRotation).observe(elements.cameraFrame);
updateTheme();
connectWorkerMetadata();
poseMonitor = createPoseMonitor({
  request: agentRequest,
  paired: () => state.paired,
  controlStatus: () => state.agent,
  onChange: () => updateButtons(state.agent || {}),
});

$("theme-toggle").addEventListener("click", () => {
  const dark = document.documentElement.dataset.theme !== "dark";
  localStorage.setItem("cf-robot-theme", dark ? "dark" : "light");
  updateTheme();
});
$("pair-controller").addEventListener("click", pairController);
elements.disconnectController.addEventListener("click", disconnectController);
elements.cameraRotate.addEventListener("click", () => {
  state.cameraRotation = (state.cameraRotation + 90) % 360;
  localStorage.setItem("cf-robot-video-rotation", String(state.cameraRotation));
  updateCameraRotation();
});
elements.start.addEventListener("click", startControl);
elements.stop.addEventListener("click", stopControl);
elements.manualMode.addEventListener("click", () => changeMode("manual"));
elements.leaderMode.addEventListener("click", () => changeMode("leader"));
elements.leaderConnect.addEventListener("click", connectLeader);
elements.leaderDisconnect.addEventListener("click", disconnectLeader);
$("rearm-button").addEventListener("click", rearm);
elements.rearmConfirmation.addEventListener("input", () => elements.rearmConfirmation.setCustomValidity(""));
elements.estop.addEventListener("click", () => {
  if (state.agent?.estop_latched) elements.rearmDialog.showModal();
  else emergencyStop();
});
elements.pairingCode.addEventListener("change", () => { state.pairingCode = elements.pairingCode.value.trim(); });
window.addEventListener("pagehide", () => {
  if (state.frameUrl) URL.revokeObjectURL(state.frameUrl);
});

heartbeat();
if (["localhost", "127.0.0.1"].includes(location.hostname) && location.port === "8080") {
  state.pairingCode = "";
  pairController();
}
