export const JOINTS = [
  { key: "shoulder_pan.pos", label: "Shoulder pan", unit: "°", min: -110, max: 110 },
  { key: "shoulder_lift.pos", label: "Shoulder lift", unit: "°", min: -100, max: 100 },
  { key: "elbow_flex.pos", label: "Elbow flex", unit: "°", min: -100, max: 90 },
  { key: "wrist_flex.pos", label: "Wrist flex", unit: "°", min: -95, max: 95 },
  { key: "wrist_roll.pos", label: "Wrist roll", unit: "°", min: -160, max: 160 },
  { key: "gripper.pos", label: "Gripper", unit: "%", min: 0, max: 100 },
];

const CHAIN = [
  {
    key: "shoulder_pan.pos",
    origin: [0.0207909, -0.0230745, 0.0948817],
    rpy: [-Math.PI, 0, Math.PI / 2],
  },
  {
    key: "shoulder_lift.pos",
    origin: [-0.0303992, -0.0182778, -0.0542],
    rpy: [-Math.PI / 2, -Math.PI / 2, 0],
  },
  {
    key: "elbow_flex.pos",
    origin: [-0.11257, -0.028, 0],
    rpy: [0, 0, Math.PI / 2],
  },
  {
    key: "wrist_flex.pos",
    origin: [-0.1349, 0.0052, 0],
    rpy: [0, 0, -Math.PI / 2],
  },
  {
    key: "wrist_roll.pos",
    origin: [0, -0.0611, 0.0181],
    rpy: [Math.PI / 2, 0, Math.PI],
  },
  {
    key: "gripper.pos",
    origin: [0.0202, 0.0188, -0.0234],
    rpy: [Math.PI / 2, 0, 0],
  },
];

const COLORS = {
  follower: "#df633e",
  leader: "#5d8580",
  grid: "#d9ddd1",
  axis: "#8d988b",
};

const identity = () => ({
  rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  position: [0, 0, 0],
});

// Joint transforms follow TheRobotStudio/SO-ARM100's Simulation/SO101 URDF
// (BSD-licensed; https://github.com/TheRobotStudio/SO-ARM100). Meshes are omitted.
const multiply3 = (a, b) => a.map((row) =>
  b[0].map((_, column) => row.reduce((sum, value, index) => sum + value * b[index][column], 0)));

const apply3 = (matrix, point) =>
  matrix.map((row) => row.reduce((sum, value, index) => sum + value * point[index], 0));

const add3 = (a, b) => a.map((value, index) => value + b[index]);

function rpyMatrix([roll, pitch, yaw]) {
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const cp = Math.cos(pitch);
  const sp = Math.sin(pitch);
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function axisRotation(axis, angle) {
  const [x, y, z] = axis;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;
  return [
    [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
    [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
  ];
}

function compose(parent, origin, angle) {
  const originRotation = rpyMatrix(origin.rpy);
  const position = add3(parent.position, apply3(parent.rotation, origin.origin));
  const atJoint = multiply3(parent.rotation, originRotation);
  const value = origin.key === "gripper.pos"
    ? (-10 + Number(angle) * 1.1) * Math.PI / 180
    : Number(angle) * Math.PI / 180;
  return {
    rotation: multiply3(atJoint, axisRotation([0, 0, 1], value)),
    position,
  };
}

export function forwardKinematics(positions) {
  let transform = identity();
  const points = [[0, 0, 0]];
  for (const joint of CHAIN) {
    transform = compose(transform, joint, positions?.[joint.key] ?? 0);
    points.push(transform.position);
  }
  return points;
}

export function alignmentDelta(leader, follower) {
  if (!leader || !follower) return null;
  return Object.fromEntries(JOINTS.map(({ key }) => [key, Number(leader[key]) - Number(follower[key])]));
}

function project(point, width, height, scale, yaw, pitch) {
  const [x, y, z] = point;
  const aroundZx = Math.cos(yaw) * x - Math.sin(yaw) * y;
  const aroundZy = Math.sin(yaw) * x + Math.cos(yaw) * y;
  const screenY = Math.cos(pitch) * z + Math.sin(pitch) * aroundZy;
  return {
    x: width / 2 + aroundZx * scale,
    y: height * 0.74 - screenY * scale,
  };
}

function drawGrid(ctx, width, height, scale, yaw, pitch) {
  ctx.save();
  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth = 1;
  for (let step = -0.5; step <= 0.5; step += 0.1) {
    for (const alongX of [true, false]) {
      const a = alongX ? [step, -0.5, 0] : [-0.5, step, 0];
      const b = alongX ? [step, 0.5, 0] : [0.5, step, 0];
      const pa = project(a, width, height, scale, yaw, pitch);
      const pb = project(b, width, height, scale, yaw, pitch);
      ctx.beginPath();
      ctx.moveTo(pa.x, pa.y);
      ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }
  }
  ctx.restore();
}

function drawPose(ctx, points, width, height, scale, yaw, pitch, color, dashed = false) {
  const projected = points.map((point) => project(point, width, height, scale, yaw, pitch));
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = dashed ? 3 : 6;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  if (dashed) ctx.setLineDash([5, 7]);
  ctx.beginPath();
  projected.forEach((point, index) => {
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.setLineDash([]);
  projected.forEach((point, index) => {
    ctx.beginPath();
    ctx.arc(point.x, point.y, index === 0 ? 9 : 5.5, 0, Math.PI * 2);
    ctx.fill();
  });
  ctx.restore();
}

export function createArmViewer(canvas) {
  const context = canvas.getContext("2d");
  const view = { yaw: 0.72, pitch: 0.42, zoom: 1, follower: null, leader: null };
  let dragging = false;
  let lastX = 0;
  let lastY = 0;

  function render() {
    const bounds = canvas.getBoundingClientRect();
    if (!bounds.width || !bounds.height) return;
    const ratio = window.devicePixelRatio || 1;
    const width = bounds.width;
    const height = bounds.height;
    if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
      canvas.width = Math.round(width * ratio);
      canvas.height = Math.round(height * ratio);
    }
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    const scale = Math.min(width, height) * 1.38 * view.zoom;
    drawGrid(context, width, height, scale, view.yaw, view.pitch);
    const follower = view.follower;
    if (view.leader) {
      drawPose(context, forwardKinematics(view.leader), width, height, scale, view.yaw, view.pitch, COLORS.leader, true);
    }
    if (follower) {
      drawPose(context, forwardKinematics(follower), width, height, scale, view.yaw, view.pitch, COLORS.follower);
    }
  }

  canvas.addEventListener("pointerdown", (event) => {
    dragging = true;
    lastX = event.clientX;
    lastY = event.clientY;
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!dragging) return;
    view.yaw += (event.clientX - lastX) * 0.009;
    view.pitch = Math.max(-0.4, Math.min(1.35, view.pitch + (event.clientY - lastY) * 0.006));
    lastX = event.clientX;
    lastY = event.clientY;
    render();
  });
  canvas.addEventListener("pointerup", () => { dragging = false; });
  canvas.addEventListener("pointercancel", () => { dragging = false; });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    view.zoom = Math.max(0.65, Math.min(1.8, view.zoom * (event.deltaY < 0 ? 1.08 : 0.92)));
    render();
  }, { passive: false });
  new ResizeObserver(render).observe(canvas);

  return {
    setPoses(follower, leader) {
      view.follower = follower;
      view.leader = leader;
      render();
    },
    render,
  };
}
