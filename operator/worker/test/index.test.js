import test from "node:test";
import assert from "node:assert/strict";

import worker from "../src/index.js";
import { alignmentDelta, forwardKinematics, HOME_POSE, JOINTS } from "../../../src/cf_robot/operator/arm-viewer.js";

const request = (path, method = "GET", body) => new Request(`https://operator.example${path}`, {
  method,
  headers: body ? { "Content-Type": "application/json" } : {},
  ...(body ? { body: JSON.stringify(body) } : {}),
});

test("public operator endpoint works without an Access identity", async () => {
  const response = await worker.fetch(request("/api/me"), { DB: {} }, {});
  assert.equal(response.status, 200);
  assert.deepEqual(await response.json(), { name: "PUBLIC OPERATOR", authenticated: false });
});

test("public operator endpoint never consults or exposes an Access identity", async () => {
  const response = await worker.fetch(
    request("/api/me"),
    { DB: {} },
    { access: { getIdentity: async () => { throw new Error("Access must not be consulted"); } } },
  );
  assert.equal(response.status, 200);
  assert.deepEqual(await response.json(), { name: "PUBLIC OPERATOR", authenticated: false });
});

test("static paths are delegated to Workers Static Assets", async () => {
  let called = false;
  const response = await worker.fetch(request("/"), {
    ASSETS: { fetch: async () => { called = true; return new Response("operator shell"); } },
  }, {});
  assert.equal(await response.text(), "operator shell");
  assert.equal(called, true);
});

test("SO-ARM101 kinematics returns a finite base, arm and tool reference", () => {
  const zero = Object.fromEntries(JOINTS.map(({ key }) => [key, 0]));
  const points = forwardKinematics(zero);
  assert.equal(points.length, 7);
  assert.ok(points.flat().every(Number.isFinite));
  const turned = forwardKinematics({ ...zero, "shoulder_pan.pos": 30 });
  assert.notDeepEqual(turned[2], points[2]);
});

test("offline home reference is an immutable complete illustration pose", () => {
  assert.deepEqual(Object.keys(HOME_POSE), JOINTS.map(({ key }) => key));
  assert.ok(JOINTS.every(({ key, min, max }) => HOME_POSE[key] >= min && HOME_POSE[key] <= max));
  assert.ok(Object.isFrozen(HOME_POSE));
  assert.ok(forwardKinematics(HOME_POSE).flat().every(Number.isFinite));
});

test("alignment differences are calculated in the six servo channels", () => {
  const follower = Object.fromEntries(JOINTS.map(({ key }) => [key, 0]));
  const leader = { ...follower, "elbow_flex.pos": 4.5 };
  assert.equal(alignmentDelta(leader, follower)["elbow_flex.pos"], 4.5);
});
