import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { DatabaseSync } from "node:sqlite";

import worker from "../src/index.js";

const SESSION_ID = "11111111-1111-4111-8111-111111111111";
const sessionBody = { id: SESSION_ID, robot_id: "so101-main", mode: "manual" };
const request = (path, method = "GET", body) => new Request(`https://operator.example${path}`, {
  method,
  ...(body !== undefined ? {
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  } : {}),
});

function testDatabase(t) {
  const database = new DatabaseSync(":memory:");
  database.exec(readFileSync(new URL("../migrations/0001_operator.sql", import.meta.url), "utf8"));
  t.after(() => database.close());
  const env = {
    DB: {
      prepare(sql) {
        const statement = database.prepare(sql);
        let parameters = [];
        return {
          bind(...values) { parameters = values; return this; },
          first() { return statement.get(...parameters) ?? null; },
          all() { return { results: statement.all(...parameters) }; },
          run() { return statement.run(...parameters); },
        };
      },
      async batch(statements) {
        return statements.map((statement) => statement.run());
      },
    },
  };
  return { database, env };
}

test("robot display metadata is public and contains no operator records", async (t) => {
  const { env } = testDatabase(t);
  const response = await worker.fetch(request("/api/robots"), env);
  assert.equal(response.status, 200);
  assert.deepEqual(await response.json(), { robots: [
    { id: "so101-main", name: "SO-ARM101 follower", model: "SO-ARM101" },
  ] });
});

test("public sessions start and finish with anonymous audit attribution", async (t) => {
  const { database, env } = testDatabase(t);
  const started = await worker.fetch(request("/api/sessions", "POST", {
    ...sessionBody, operator_email: "spoofed@example.com",
  }), env);
  assert.equal(started.status, 201);
  assert.equal((await started.json()).id, SESSION_ID);
  const finished = await worker.fetch(request(`/api/sessions/${SESSION_ID}`, "PATCH", {
    state: "stopped", command_count: 12,
  }), env);
  assert.equal(finished.status, 200);
  const session = database.prepare("SELECT * FROM control_sessions").get();
  assert.equal(session.operator_email, "anonymous");
  assert.equal(session.state, "stopped");
  assert.equal(session.command_count, 12);
  assert.ok(session.ended_at);
  const audit = database.prepare("SELECT actor_email, event_type FROM audit_events ORDER BY id").all();
  assert.deepEqual(audit.map((row) => ({ ...row })), [
    { actor_email: "anonymous", event_type: "session_started" },
    { actor_email: "anonymous", event_type: "session_stopped" },
  ]);
});

test("anonymous requests cannot change historical identified sessions", async (t) => {
  const { database, env } = testDatabase(t);
  database.prepare(`INSERT INTO control_sessions
    (id, robot_id, operator_email, mode, state, started_at)
    VALUES (?, 'so101-main', 'original@example.com', 'manual', 'running', ?)`)
    .run(SESSION_ID, new Date().toISOString());
  const response = await worker.fetch(request(`/api/sessions/${SESSION_ID}`, "PATCH", {
    state: "error", command_count: 100,
  }), env);
  assert.equal(response.status, 404);
  assert.equal(database.prepare("SELECT state FROM control_sessions").get().state, "running");
  assert.equal(database.prepare("SELECT count(*) AS count FROM audit_events").get().count, 0);
});

test("public session routes retain validation and expose no session listing", async (t) => {
  const { database, env } = testDatabase(t);
  for (const [path, method, body, status] of [
    ["/api/sessions", "POST", null, 400],
    ["/api/sessions", "POST", { ...sessionBody, id: "bad" }, 400],
    ["/api/sessions", "POST", { ...sessionBody, mode: "other" }, 400],
    ["/api/sessions", "POST", { ...sessionBody, robot_id: "unknown" }, 404],
    [`/api/sessions/${SESSION_ID}`, "PATCH", { state: "running" }, 400],
    [`/api/sessions/${SESSION_ID}`, "PATCH", { state: "stopped" }, 404],
    ["/api/sessions", "GET", undefined, 404],
    [`/api/sessions/${SESSION_ID}`, "GET", undefined, 404],
  ]) {
    const response = await worker.fetch(request(path, method, body), env);
    assert.equal(response.status, status, `${method} ${path}: ${JSON.stringify(body)}`);
  }
  assert.equal(database.prepare("SELECT count(*) AS count FROM control_sessions").get().count, 0);
});
