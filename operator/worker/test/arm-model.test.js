import test from "node:test";
import assert from "node:assert/strict";

import { buildArmModel, forwardKinematics, HOME_POSE, JOINTS } from "../../../src/cf_robot/operator/arm-viewer.js";

const zero = Object.fromEntries(JOINTS.map(({ key }) => [key, 0]));
const vertices = (model, name) => model.parts.find((part) => part.name === name).vertices;
const distance = (a, b) => Math.hypot(...a.map((value, index) => value - b[index]));
const dot = (a, b) => a.reduce((sum, value, index) => sum + value * b[index], 0);
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-9, `${a} != ${b}`);

test("six mechanical joints have vertical yaw, parallel bends, axial roll and a jaw hinge", () => {
  const model = buildArmModel(HOME_POSE);
  assert.deepEqual(model.joints.map((joint) => joint.key), JOINTS.map(({ key }) => key));
  model.joints.forEach((joint) => near(Math.hypot(...joint.axis), 1));
  near(Math.abs(model.joints[0].axis[2]), 1);
  near(Math.abs(dot(model.joints[1].axis, model.joints[2].axis)), 1);
  near(Math.abs(dot(model.joints[2].axis, model.joints[3].axis)), 1);
  near(dot(model.joints[3].axis, model.joints[4].axis), 0);
  near(dot(model.joints[4].axis, model.joints[5].axis), 0);
});

test("two long links retain their lengths throughout shoulder, elbow and wrist movement", () => {
  const baseline = buildArmModel(zero);
  const lengths = [1, 2].map((index) =>
    distance(baseline.joints[index].position, baseline.joints[index + 1].position));
  assert.ok(lengths[0] > 0.11 && lengths[0] < 0.12);
  assert.ok(lengths[1] > 0.13 && lengths[1] < 0.14);
  for (const pose of [HOME_POSE, zero, Object.fromEntries(JOINTS.map(({ key, max }) => [key, max]))]) {
    const model = buildArmModel(pose);
    lengths.forEach((length, index) =>
      near(distance(model.joints[index + 1].position, model.joints[index + 2].position), length));
    assert.ok(model.parts.flatMap((part) => part.vertices).flat().every(Number.isFinite));
    assert.ok(model.parts.every((part) => part.faces.every((face) => face.every((index) =>
      index >= 0 && index < part.vertices.length))));
  }
});

test("J1 turns the arm about the vertical bearing without moving the stationary base", () => {
  const before = buildArmModel(zero);
  const after = buildArmModel({ ...zero, "shoulder_pan.pos": 60 });
  for (const name of ["base-foot", "base-pedestal", "base-motor"]) {
    assert.deepEqual(vertices(after, name), vertices(before, name));
  }
  assert.notDeepEqual(vertices(after, "upper-arm-1"), vertices(before, "upper-arm-1"));
  before.joints.forEach((joint, index) => near(joint.position[2], after.joints[index].position[2]));
});

test("J2 raises the elbow, while J3 bends the forearm without altering the upper arm", () => {
  const before = buildArmModel(zero);
  const shoulder = buildArmModel({ ...zero, "shoulder_lift.pos": -40 });
  assert.deepEqual(shoulder.joints[1].position, before.joints[1].position);
  assert.notDeepEqual(shoulder.joints[2].position, before.joints[2].position);
  const elbow = buildArmModel({ ...zero, "elbow_flex.pos": 45 });
  assert.deepEqual(vertices(elbow, "upper-arm-1"), vertices(before, "upper-arm-1"));
  assert.notDeepEqual(vertices(elbow, "forearm-1"), vertices(before, "forearm-1"));
});

test("J4 bends only the compact wrist and hand, not an extra long arm link", () => {
  const before = buildArmModel(zero);
  const after = buildArmModel({ ...zero, "wrist_flex.pos": 45 });
  for (const name of ["upper-arm-1", "forearm-1", "j4-motor"]) {
    assert.deepEqual(vertices(after, name), vertices(before, name));
  }
  assert.notDeepEqual(vertices(after, "wrist-yoke-1"), vertices(before, "wrist-yoke-1"));
  assert.notDeepEqual(vertices(after, "jaw-fixed"), vertices(before, "jaw-fixed"));
});

test("J5 rolls the whole gripper without changing the arm or bending the roll axis", () => {
  const before = buildArmModel(zero);
  const after = buildArmModel({ ...zero, "wrist_roll.pos": 90 });
  for (const name of ["upper-arm-1", "forearm-1", "wrist-yoke-1", "roll-motor"]) {
    assert.deepEqual(vertices(after, name), vertices(before, name));
  }
  before.joints.slice(0, 5).forEach((joint, index) =>
    assert.deepEqual(joint.position, after.joints[index].position));
  assert.deepEqual(before.joints[4].axis, after.joints[4].axis);
  assert.notDeepEqual(vertices(after, "jaw-fixed"), vertices(before, "jaw-fixed"));
  assert.notDeepEqual(vertices(after, "jaw-moving"), vertices(before, "jaw-moving"));
});

test("J6 opens only the moving jaw; fixed jaw, wrist and tool reference remain fixed", () => {
  const before = buildArmModel({ ...HOME_POSE, "gripper.pos": 0 });
  const after = buildArmModel({ ...HOME_POSE, "gripper.pos": 100 });
  const changed = before.parts.filter((part) =>
    JSON.stringify(part.vertices) !== JSON.stringify(vertices(after, part.name))).map((part) => part.name);
  assert.deepEqual(changed, ["jaw-moving", "jaw-moving-pad"]);
  assert.deepEqual(after.joints.map((joint) => joint.position), before.joints.map((joint) => joint.position));
  assert.deepEqual(after.tool, before.tool);
  assert.deepEqual(forwardKinematics({ ...HOME_POSE, "gripper.pos": 0 }),
    forwardKinematics({ ...HOME_POSE, "gripper.pos": 100 }));
});
