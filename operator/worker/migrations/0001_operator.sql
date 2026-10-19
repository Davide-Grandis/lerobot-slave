CREATE TABLE robots (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    model TEXT NOT NULL,
    created_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE TABLE control_sessions (
    id TEXT PRIMARY KEY,
    robot_id TEXT NOT NULL REFERENCES robots(id),
    operator_email TEXT NOT NULL,
    mode TEXT NOT NULL CHECK (mode IN ('manual', 'leader')),
    state TEXT NOT NULL CHECK (state IN ('running', 'stopped', 'error')),
    started_at TEXT NOT NULL,
    ended_at TEXT,
    command_count INTEGER NOT NULL DEFAULT 0 CHECK (command_count >= 0)
);

CREATE INDEX control_sessions_robot_started
    ON control_sessions(robot_id, started_at DESC);

CREATE TABLE audit_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT REFERENCES control_sessions(id),
    robot_id TEXT NOT NULL REFERENCES robots(id),
    actor_email TEXT NOT NULL,
    event_type TEXT NOT NULL CHECK (
        event_type IN ('session_started', 'session_stopped', 'session_error', 'software_stop')
    ),
    created_at TEXT NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE INDEX audit_events_robot_created
    ON audit_events(robot_id, created_at DESC);

INSERT INTO robots (id, name, model)
VALUES ('so101-main', 'SO-ARM101 follower', 'SO-ARM101')
ON CONFLICT(id) DO NOTHING;
