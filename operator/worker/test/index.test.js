import test from "node:test";
import assert from "node:assert/strict";

import worker from "../src/index.js";
import { alignmentDelta, forwardKinematics, JOINTS } from "../../../src/cf_robot/operator/arm-viewer.js";

const request = (path, method = "GET", body) => new Request(`https://operator.example${path}`, {
  method,
  headers: body ? { "Content-Type": "application/json" } : {},
  ...(body ? { body: JSON.stringify(body) } : {}),
});

test("API routes require an Access identity", async () => {
  const response = await worker.fetch(request("/api/me"), { DB: {} }, {});
  assert.equal(response.status, 401);
  assert.equal(await response.text(), "Cloudflare Access authentication required");
});

test("static UI is also denied without an Access identity", async () => {
  const response = await worker.fetch(request("/"), { ASSETS: {} }, {});
  assert.equal(response.status, 401);
});

test("identity endpoint exposes only the authenticated operator", async () => {
  const response = await worker.fetch(
    request("/api/me"),
    { DB: {} },
    { access: { getIdentity: async () => ({ email: "davide@example.com", name: "Davide" }) } },
  );
  assert.equal(response.status, 200);
  assert.deepEqual(await response.json(), { email: "davide@example.com", name: "Davide" });
});

test("static paths are delegated to Workers Static Assets", async () => {
  let called = false;
  const response = await worker.fetch(request("/"), {
    ASSETS: { fetch: async () => { called = true; return new Response("operator shell"); } },
  }, { access: { getIdentity: async () => ({ email: "davide@example.com" }) } });
  assert.equal(await response.text(), "operator shell");
  assert.equal(called, true);
});

test("SO-ARM101 kinematics returns a finite base and six-joint chain", () => {
  const zero = Object.fromEntries(JOINTS.map(({ key }) => [key, 0]));
  const points = forwardKinematics(zero);
  assert.equal(points.length, 7);
  assert.ok(points.flat().every(Number.isFinite));
  const turned = forwardKinematics({ ...zero, "shoulder_pan.pos": 30 });
  assert.notDeepEqual(turned[2], points[2]);
});

test("alignment differences are calculated in the six servo channels", () => {
  const follower = Object.fromEntries(JOINTS.map(({ key }) => [key, 0]));
  const leader = { ...follower, "elbow_flex.pos": 4.5 };
  assert.equal(alignmentDelta(leader, follower)["elbow_flex.pos"], 4.5);
});
