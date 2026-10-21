export const JOINTS = [
  { key: "shoulder_pan.pos", label: "Shoulder pan", unit: "°", min: -110, max: 110 },
  { key: "shoulder_lift.pos", label: "Shoulder lift", unit: "°", min: -100, max: 100 },
  { key: "elbow_flex.pos", label: "Elbow flex", unit: "°", min: -100, max: 90 },
  { key: "wrist_flex.pos", label: "Wrist flex", unit: "°", min: -95, max: 95 },
  { key: "wrist_roll.pos", label: "Wrist roll", unit: "°", min: -160, max: 160 },
  { key: "gripper.pos", label: "Gripper", unit: "%", min: 0, max: 100 },
];

// Illustration-style reference only, never a calibrated hardware go-home target.
export const HOME_POSE = Object.freeze({
  "shoulder_pan.pos": 0,
  "shoulder_lift.pos": -65,
  "elbow_flex.pos": 20,
  "wrist_flex.pos": 80,
  "wrist_roll.pos": 0,
  "gripper.pos": 60,
});

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

function jointMount(parent, origin) {
  const originRotation = rpyMatrix(origin.rpy);
  return {
    position: add3(parent.position, apply3(parent.rotation, origin.origin)),
    rotation: multiply3(parent.rotation, originRotation),
  };
}

function compose(parent, origin, angle) {
  const mount = jointMount(parent, origin);
  const value = origin.key === "gripper.pos"
    ? (-10 + Number(angle) * 1.1) * Math.PI / 180
    : Number(angle) * Math.PI / 180;
  return {
    rotation: multiply3(mount.rotation, axisRotation([0, 0, 1], value)),
    position: mount.position,
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
  const frames = jointFrames(positions);
  const palm = jointMount(frames[5], CHAIN[5]);
  // J6 opens a jaw; it does not bend the wrist or move the fixed tool reference.
  const tool = add3(palm.position, apply3(palm.rotation, [-0.024, -0.072, 0]));
  return [...frames.slice(0, -1).map((frame) => frame.position), tool];
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

export function buildArmModel(positions = HOME_POSE) {
  const frames = jointFrames(positions);
  const parts = [];
  function prism(name, frame, outline, thickness, offset, material = "shell") {
    const count = outline.length;
    const vertices = [-0.5, 0.5].flatMap((side) => outline.map(([x, y]) =>
      add3(frame.position, apply3(frame.rotation, add3(offset, [x, y, side * thickness])))));
    const faces = [
      Array.from({ length: count }, (_, index) => count - 1 - index),
      Array.from({ length: count }, (_, index) => count + index),
      ...outline.map((_, index) => {
        const next = (index + 1) % count;
        return [index, next, next + count, index + count];
      }),
    ];
    parts.push({ name, material, vertices, faces });
  }
  function box(name, frame, offset, [x, y, z], material = "shell") {
    prism(name, frame, [[-x / 2, -y / 2], [x / 2, -y / 2], [x / 2, y / 2], [-x / 2, y / 2]],
      z, offset, material);
  }
  function cylinder(name, frame, offset, radius, depth, material = "shell") {
    const outline = Array.from({ length: 16 }, (_, index) => {
      const angle = index * Math.PI / 8;
      return [Math.cos(angle) * radius, Math.sin(angle) * radius];
    });
    prism(name, frame, outline, depth, offset, material);
  }
  function bracket(name, a, b, axis, width = 0.036) {
    const x = unit3(subtract3(b, a));
    const y = unit3(cross3(axis, x));
    const z = cross3(x, y);
    const frame = { position: a, rotation: x.map((_, row) => [x[row], y[row], z[row]]) };
    const length = Math.hypot(...subtract3(b, a));
    const outline = [
      [-0.014, -width / 4], [-0.007, -width / 2],
      [length - 0.007, -width / 2], [length + 0.014, -width / 4],
      [length + 0.014, width / 4], [length + 0.007, width / 2],
      [0.007, width / 2], [-0.014, width / 4],
    ];
    for (const side of [-1, 1]) {
      prism(`${name}-${side}`, frame, outline, 0.006, [0, 0, side * 0.023]);
    }
    box(`${name}-brace`, frame, [length * 0.5, 0, 0], [0.017, width, 0.042]);
  }
  function servo(name, mount) {
    box(`${name}-motor`, mount, [0.009, -0.008, 0], [0.04, 0.031, 0.036], "servo");
    for (const side of [-1, 1]) {
      cylinder(`${name}-cap-${side}`, mount, [0, 0, side * 0.026], 0.02, 0.006);
      cylinder(`${name}-horn-${side}`, mount, [0, 0, side * 0.030], 0.011, 0.003, "accent");
      cylinder(`${name}-screw-${side}`, mount, [0, 0, side * 0.032], 0.003, 0.003, "metal");
    }
  }
  const axis = (index) => frames[index].rotation.map((row) => row[2]);
  const base = identity();
  box("base-foot", base, [0, -0.025, 0.01], [0.115, 0.1, 0.02]);
  box("base-pedestal", base, [-0.015, -0.023, 0.045], [0.071, 0.064, 0.06]);
  box("base-motor", base, [-0.016, -0.023, 0.076], [0.046, 0.038, 0.032], "servo");
  cylinder("base-turntable", frames[1], [0, 0, 0], 0.038, 0.014);
  cylinder("base-yaw-ring", frames[1], [0, 0, -0.009], 0.029, 0.005, "accent");
  bracket("shoulder-yoke", frames[1].position, frames[2].position, axis(2), 0.045);

  // Only J2-J3 and J3-J4 are long links. J4-J5 is the compact wrist yoke.
  bracket("upper-arm", frames[2].position, frames[3].position, axis(2));
  bracket("forearm", frames[3].position, frames[4].position, axis(3));
  bracket("wrist-yoke", frames[4].position, frames[5].position, axis(4), 0.03);
  for (const index of [1, 2, 3]) {
    servo(`j${index + 1}`, jointMount(frames[index], CHAIN[index]));
  }
  const rollMount = jointMount(frames[4], CHAIN[4]);
  box("roll-motor", rollMount, [0, 0, 0.012], [0.035, 0.038, 0.039], "servo");
  cylinder("roll-bearing", rollMount, [0, 0, -0.009], 0.025, 0.014);
  cylinder("roll-horn", frames[5], [0, 0, -0.019], 0.02, 0.006, "accent");
  box("gripper-mount", frames[5], [0, 0, -0.032], [0.039, 0.045, 0.028]);

  const palm = jointMount(frames[5], CHAIN[5]);
  servo("j6", palm);
  // The SO-ARM101 has one fixed jaw and one jaw driven by the sixth servo.
  prism("jaw-fixed", palm, [
    [-0.014, 0.02], [-0.035, 0.01], [-0.041, -0.052],
    [-0.029, -0.087], [-0.021, -0.087], [-0.026, -0.018], [-0.009, -0.008],
  ], 0.018, [0, 0, -0.008]);
  prism("jaw-moving", frames[6], [
    [-0.013, 0.012], [0.012, 0.012], [0.021, -0.01],
    [0.001, -0.071], [-0.009, -0.085], [-0.016, -0.081], [-0.012, -0.022],
  ], 0.014, [0, 0, 0.012]);
  box("jaw-fixed-pad", palm, [-0.022, -0.071, -0.008], [0.006, 0.024, 0.021], "pad");
  box("jaw-moving-pad", frames[6], [-0.013, -0.069, 0.012], [0.006, 0.024, 0.018], "pad");

  return {
    parts,
    joints: JOINTS.map((joint, index) => ({
      key: joint.key,
      position: frames[index + 1].position,
      axis: axis(index + 1),
    })),
    tool: forwardKinematics(positions).at(-1),
  };
}

function shaded(color, brightness) {
  const channels = color.match(/[a-f\d]{2}/gi).map((value) =>
    Math.min(255, Math.round(parseInt(value, 16) * brightness)));
  return `rgb(${channels.join(",")})`;
}

function drawSolidPose(ctx, model, width, height, scale, yaw, pitch, color) {
  const faces = [];
  const materials = {
    shell: color === COLORS.follower ? "#e9ece4" : "#9cc1b8",
    servo: "#29343b",
    accent: color,
    metal: "#b5bdba",
    pad: "#18262c",
  };
  for (const part of model.parts) {
    for (const indices of part.faces) {
      const points = indices.map((index) => part.vertices[index]);
      const normal = unit3(cross3(subtract3(points[1], points[0]), subtract3(points[2], points[0])));
      const light = Math.abs(normal[0] * 0.3 - normal[1] * 0.5 + normal[2] * 0.8);
      const projected = points.map((point) => project(point, width, height, scale, yaw, pitch));
      faces.push({
        points: projected,
        depth: projected.reduce((sum, point) => sum + point.depth, 0) / 4,
        color: shaded(materials[part.material], 0.64 + light * 0.35),
      });
    }
  }
  faces.sort((a, b) => b.depth - a.depth);
  ctx.save();
  ctx.lineWidth = 0.5;
  ctx.strokeStyle = "rgba(12,20,18,.28)";
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
  const view = { yaw: 0.72, pitch: 0.3, zoom: 1, follower: null, leader: null };
  canvas.setAttribute("aria-label",
    "SO-ARM101: J1 base yaw, J2 shoulder, J3 elbow, J4 wrist bend, J5 wrist roll, J6 gripper opening");
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
    const follower = view.follower && buildArmModel(view.follower);
    const leader = view.leader && buildArmModel(view.leader);
    const models = [follower, leader].filter(Boolean);
    const points = models.flatMap((model) => model.parts.flatMap((part) => part.vertices))
      .map((point) => project(point, 0, 0, 1, view.yaw, view.pitch));
    let scale = Math.min(width, height) * 1.38 * view.zoom;
    let offset = { x: 0, y: 0 };
    if (points.length) {
      const xs = points.map((point) => point.x);
      const ys = points.map((point) => point.y);
      const left = Math.min(...xs);
      const right = Math.max(...xs);
      const top = Math.min(...ys);
      const bottom = Math.max(...ys);
      scale = Math.min((width - 94) / (right - left), (height - 92) / (bottom - top)) * view.zoom;
      offset = {
        x: -(left + right) * scale / 2,
        y: height / 2 + 9 - height * 0.74 - (top + bottom) * scale / 2,
      };
    }
    context.save();
    context.translate(offset.x, offset.y);
    drawGrid(context, width, height, scale, view.yaw, view.pitch);
    if (leader) {
      if (solid) drawSolidPose(context, leader, width, height, scale, view.yaw, view.pitch, COLORS.leader);
      else drawPose(context, forwardKinematics(view.leader), width, height, scale, view.yaw, view.pitch, COLORS.leader, true);
    }
    if (follower) {
      if (solid) drawSolidPose(context, follower, width, height, scale, view.yaw, view.pitch, COLORS.follower);
      else drawPose(context, forwardKinematics(view.follower), width, height, scale, view.yaw, view.pitch, COLORS.follower);
    }
    context.restore();
    const labelled = follower || leader;
    if (labelled) {
      const placed = [];
      const shifts = [[-28, 18], [30, 8], [-28, -14], [-25, -18], [28, -12], [31, 14]];
      context.save();
      context.font = '700 9px "SFMono-Regular", Consolas, monospace';
      context.textAlign = "center";
      context.textBaseline = "middle";
      labelled.joints.forEach((joint, index) => {
        const projected = project(joint.position, width, height, scale, view.yaw, view.pitch);
        const point = { x: projected.x + offset.x, y: projected.y + offset.y };
        const x = Math.max(18, Math.min(width - 18, point.x + shifts[index][0]));
        let y = Math.max(52, Math.min(height - 30, point.y + shifts[index][1]));
        while (placed.some((label) => Math.abs(label.x - x) < 30 && Math.abs(label.y - y) < 16)) {
          y = y + 17 > height - 24 ? 52 : y + 17;
        }
        placed.push({ x, y });
        context.strokeStyle = follower ? COLORS.follower : COLORS.leader;
        context.lineWidth = 0.8;
        context.beginPath();
        context.moveTo(point.x, point.y);
        context.lineTo(x, y);
        context.stroke();
        context.fillStyle = document.documentElement.dataset.theme === "dark" ? "#18211f" : "#f4f2e9";
        context.fillRect(x - 12, y - 7, 24, 14);
        context.fillStyle = follower ? COLORS.follower : "#42796d";
        context.fillText(`J${index + 1}`, x, y);
      });
      context.restore();
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
