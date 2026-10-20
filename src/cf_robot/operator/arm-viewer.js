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
  leader: "#80aba2",
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

function jointFrames(positions) {
  let transform = identity();
  const frames = [transform];
  for (const joint of CHAIN) {
    transform = compose(transform, joint, positions?.[joint.key] ?? 0);
    frames.push(transform);
  }
  return frames;
}

export function forwardKinematics(positions) {
  return jointFrames(positions).map((frame) => frame.position);
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
    depth: Math.cos(pitch) * aroundZy - Math.sin(pitch) * z,
  };
}

function drawGrid(ctx, width, height, scale, yaw, pitch) {
  ctx.save();
  ctx.strokeStyle = COLORS.grid;
  ctx.globalAlpha = document.documentElement.dataset.theme === "dark" ? 0.16 : 0.6;
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

const scale3 = (vector, amount) => vector.map((value) => value * amount);
const subtract3 = (a, b) => a.map((value, index) => value - b[index]);
const cross3 = ([ax, ay, az], [bx, by, bz]) => [
  ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx,
];
const unit3 = (vector) => scale3(vector, 1 / Math.hypot(...vector));

function shaded(color, brightness) {
  const channels = color.match(/[a-f\d]{2}/gi).map((value) =>
    Math.min(255, Math.round(parseInt(value, 16) * brightness)));
  return `rgb(${channels.join(",")})`;
}

function drawSolidPose(ctx, positions, width, height, scale, yaw, pitch, color) {
  const faces = [];
  const frames = jointFrames(positions);
  const faceIndices = [
    [0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1],
    [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3],
  ];
  function box(center, dimensions, rotation, material) {
    const vertices = Array.from({ length: 8 }, (_, index) => add3(center, apply3(
      rotation, dimensions.map((size, axis) => (index & (1 << axis) ? 0.5 : -0.5) * size),
    )));
    for (const indices of faceIndices) {
      const points = indices.map((index) => vertices[index]);
      const normal = unit3(cross3(subtract3(points[1], points[0]), subtract3(points[2], points[0])));
      const light = Math.abs(normal[0] * 0.3 - normal[1] * 0.5 + normal[2] * 0.8);
      const projected = points.map((point) => project(point, width, height, scale, yaw, pitch));
      faces.push({
        points: projected,
        depth: projected.reduce((sum, point) => sum + point.depth, 0) / 4,
        color: shaded(material, 0.65 + light * 0.5),
      });
    }
  }
  function localBox(frame, offset, size, material) {
    box(add3(frame.position, apply3(frame.rotation, offset)), size, frame.rotation, material);
  }
  const base = identity();
  localBox(base, [0, 0, 0.012], [0.105, 0.09, 0.024], "#40534c");
  localBox(base, [0, 0, 0.043], [0.052, 0.05, 0.05], color);
  for (let index = 1; index < frames.length; index += 1) {
    const a = frames[index - 1].position;
    const b = frames[index].position;
    const length = Math.hypot(...subtract3(b, a));
    if (length > 0.025) {
      const x = unit3(subtract3(b, a));
      const y = unit3(cross3(Math.abs(x[2]) < 0.9 ? [0, 0, 1] : [0, 1, 0], x));
      const z = cross3(x, y);
      const rotation = x.map((_, row) => [x[row], y[row], z[row]]);
      const center = scale3(add3(a, b), 0.5);
      // Paired side plates and servo housings approximate the SO-ARM101 structure.
      for (const offset of [-0.018, 0.018]) {
        box(add3(center, scale3(y, offset)), [length, 0.009, 0.025], rotation, color);
      }
    }
    if (index < frames.length - 1) {
      localBox(frames[index], [0, 0, 0], [0.041, 0.032, 0.04], "#384940");
      localBox(frames[index], [0, 0, 0.023], [0.025, 0.024, 0.008], color);
      localBox(frames[index], [0, 0, 0.029], [0.009, 0.009, 0.005], "#c4cec0");
    }
  }
  const palm = compose(frames.at(-2), CHAIN.at(-1), 0);
  localBox(palm, [0.008, 0, 0], [0.035, 0.044, 0.025], color);
  const opening = Math.max(0, Math.min(100, positions["gripper.pos"])) / 100;
  for (const direction of [-1, 1]) {
    const angle = direction * opening * 0.55;
    const finger = {
      position: add3(palm.position, apply3(palm.rotation, [0.02, direction * 0.014, 0])),
      rotation: multiply3(palm.rotation, axisRotation([0, 0, 1], angle)),
    };
    localBox(finger, [0.025, 0, 0], [0.05, 0.009, 0.014], color);
    localBox(finger, [0.048, -direction * 0.004, 0], [0.014, 0.015, 0.018], "#384940");
  }
  faces.sort((a, b) => b.depth - a.depth);
  ctx.save();
  ctx.lineWidth = 0.7;
  ctx.strokeStyle = "rgba(12,20,18,.5)";
  for (const face of faces) {
    ctx.beginPath();
    face.points.forEach((point, index) => {
      if (index === 0) ctx.moveTo(point.x, point.y);
      else ctx.lineTo(point.x, point.y);
    });
    ctx.closePath();
    ctx.fillStyle = face.color;
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}

export function createArmViewer(canvas, { solid = false } = {}) {
  const context = canvas.getContext("2d");
  const view = { yaw: 0.72, pitch: 0.42, zoom: solid ? 1.4 : 1, follower: null, leader: null };
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
      if (solid) drawSolidPose(context, view.leader, width, height, scale, view.yaw, view.pitch, COLORS.leader);
      else drawPose(context, forwardKinematics(view.leader), width, height, scale, view.yaw, view.pitch, COLORS.leader, true);
    }
    if (follower) {
      if (solid) drawSolidPose(context, follower, width, height, scale, view.yaw, view.pitch, COLORS.follower);
      else drawPose(context, forwardKinematics(follower), width, height, scale, view.yaw, view.pitch, COLORS.follower);
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
