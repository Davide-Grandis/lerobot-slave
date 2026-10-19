const SESSION_ID_PATTERN = /^[0-9a-f-]{36}$/i;
const VALID_MODES = new Set(["manual", "leader"]);
const TERMINAL_STATES = new Set(["stopped", "error"]);

function json(data, status = 200) {
  return Response.json(data, {
    status,
    headers: { "Cache-Control": "no-store" },
  });
}

async function identityFrom(context) {
  if (context.access?.getIdentity) {
    return context.access.getIdentity();
  }
  return null;
}

function requestJson(request) {
  return request.json().catch(() => null);
}

async function handleApi(request, env, identity, url) {
  if (request.method === "GET" && url.pathname === "/api/me") {
    return json({ email: identity.email, name: identity.name ?? identity.email });
  }

  if (request.method === "GET" && url.pathname === "/api/robots") {
    const { results } = await env.DB.prepare(
      "SELECT id, name, model FROM robots ORDER BY name",
    ).all();
    return json({ robots: results });
  }

  if (request.method === "POST" && url.pathname === "/api/sessions") {
    const body = await requestJson(request);
    if (
      !body
      || !SESSION_ID_PATTERN.test(body.id ?? "")
      || typeof body.robot_id !== "string"
      || !VALID_MODES.has(body.mode)
    ) {
      return json({ error: "Invalid session payload" }, 400);
    }
    const robot = await env.DB.prepare("SELECT id FROM robots WHERE id = ?")
      .bind(body.robot_id)
      .first();
    if (!robot) {
      return json({ error: "Unknown robot" }, 404);
    }
    const startedAt = new Date().toISOString();
    await env.DB.batch([
      env.DB.prepare(
        `INSERT INTO control_sessions
          (id, robot_id, operator_email, mode, state, started_at)
         VALUES (?, ?, ?, ?, 'running', ?)`,
      ).bind(body.id, body.robot_id, identity.email, body.mode, startedAt),
      env.DB.prepare(
        `INSERT INTO audit_events (session_id, robot_id, actor_email, event_type)
         VALUES (?, ?, ?, 'session_started')`,
      ).bind(body.id, body.robot_id, identity.email),
    ]);
    return json({ id: body.id, state: "running", started_at: startedAt }, 201);
  }

  const sessionMatch = url.pathname.match(/^\/api\/sessions\/([0-9a-f-]{36})$/i);
  if (request.method === "PATCH" && sessionMatch) {
    const body = await requestJson(request);
    if (!body || !TERMINAL_STATES.has(body.state)) {
      return json({ error: "Session state must be stopped or error" }, 400);
    }
    const session = await env.DB.prepare(
      "SELECT id, robot_id FROM control_sessions WHERE id = ? AND operator_email = ?",
    ).bind(sessionMatch[1], identity.email).first();
    if (!session) {
      return json({ error: "Session not found" }, 404);
    }
    const eventType = body.state === "error" ? "session_error" : "session_stopped";
    await env.DB.batch([
      env.DB.prepare(
        `UPDATE control_sessions
         SET state = ?, ended_at = ?, command_count = ?
         WHERE id = ? AND operator_email = ?`,
      ).bind(
        body.state,
        new Date().toISOString(),
        Number.isSafeInteger(body.command_count) && body.command_count >= 0
          ? body.command_count
          : 0,
        session.id,
        identity.email,
      ),
      env.DB.prepare(
        `INSERT INTO audit_events (session_id, robot_id, actor_email, event_type)
         VALUES (?, ?, ?, ?)`,
      ).bind(session.id, session.robot_id, identity.email, eventType),
    ]);
    return json({ id: session.id, state: body.state });
  }

  return json({ error: "Not found" }, 404);
}

export default {
  async fetch(request, env, context) {
    const url = new URL(request.url);
    const identity = await identityFrom(context);
    if (!identity?.email) {
      return new Response("Cloudflare Access authentication required", {
        status: 401,
        headers: { "Cache-Control": "no-store" },
      });
    }
    if (url.pathname.startsWith("/api/")) {
      try {
        return await handleApi(request, env, identity, url);
      } catch (error) {
        console.error(JSON.stringify({
          event: "operator_api_error",
          path: url.pathname,
          message: String(error),
        }));
        return json({ error: "Operator API unavailable" }, 500);
      }
    }
    return env.ASSETS.fetch(request);
  },
};
