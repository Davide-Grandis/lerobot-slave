"""Access credentials are supplied only by the laptop process environment."""

import os
from urllib.parse import urlsplit


def access_headers(url):
    parsed = urlsplit(url)
    if parsed.scheme not in ("ws", "wss") or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError("expected a ws:// or wss:// URL without embedded credentials")
    if parsed.scheme == "ws" and parsed.hostname not in ("localhost", "127.0.0.1"):
        raise ValueError("remote connections must use wss://")
    key = os.environ.get("CF_ROBOT_ORIGIN_KEY")
    if not key or len(key) < 32:
        raise ValueError("set CF_ROBOT_ORIGIN_KEY to the Pi's 32+ character origin key")
    if parsed.scheme == "ws":
        return {"X-CF-Robot-Key": key}
    client_id = os.environ.get("CF_ACCESS_CLIENT_ID")
    secret = os.environ.get("CF_ACCESS_CLIENT_SECRET")
    if not client_id or not secret:
        raise ValueError("set CF_ACCESS_CLIENT_ID and CF_ACCESS_CLIENT_SECRET")
    return {
        "CF-Access-Client-Id": client_id,
        "CF-Access-Client-Secret": secret,
        "X-CF-Robot-Key": key,
    }
