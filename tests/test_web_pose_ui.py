"""Opt-in Chrome smoke test: CF_ROBOT_BROWSER_TESTS=1; never connects to hardware."""

import asyncio
import base64
import functools
import http.server
import json
import os
import subprocess
import tempfile
import threading
import unittest
from pathlib import Path

try:
    from websockets.asyncio.client import connect
except ImportError:
    connect = None


MOCK_BRIDGE = """
const originalFetch = window.fetch.bind(window);
window.testCommands = [];
const keys = ['shoulder_pan.pos','shoulder_lift.pos','elbow_flex.pos',
              'wrist_flex.pos','wrist_roll.pos','gripper.pos'];
const sample = side => ({
  type:'pose_sample', side, sampled_at:Date.now()/1000,
  values:Object.fromEntries(keys.map((k,i)=>[k,i*2])),
  raw:Object.fromEntries(keys.map(k=>[k,2048])),
  torque:Object.fromEntries(keys.map(k=>[k,true])),
  limits:Object.fromEntries(keys.map(k=>[k,{min:100,max:3900}]))
});
let running = false, command = null;
window.fetch = async (url, options={}) => {
  const path = new URL(url,location.href).pathname;
  const reply = (payload,status=200) => new Response(JSON.stringify(payload),
    {status,headers:{'Content-Type':'application/json'}});
  if (!path.startsWith('/api/')) return originalFetch(url,options);
  if (path === '/api/config') {
    window.pairedAddress = new URL(url).origin;
    return reply({api_version:1});
  }
  if (path === '/api/status') return reply({running:false,state:'idle',camera_live:false});
  if (path === '/api/video/frame') return reply({},503);
  if (path === '/api/pose/start') { running=true; return reply({running}); }
  if (path === '/api/pose/stop') { running=false; return reply({running}); }
  if (path === '/api/pose/command') {
    const body = JSON.parse(options.body);
    window.testCommands.push(body);
    command={type:'command_result',command_id:window.testCommands.length,
             joint_id:body.joint_id,state:'succeeded'};
    return reply({command_id:command.command_id});
  }
  if (path === '/api/pose/heartbeat') return reply({ok:true});
  if (path === '/api/pose/status') return reply({
    running,state:running?'ready':'idle',command_pending:false,
    leader:running?sample('leader'):null,follower:running?sample('follower'):null,command
  });
  return reply({},401);
};
"""


@unittest.skipUnless(
    os.environ.get("CF_ROBOT_BROWSER_TESTS") == "1" and connect is not None,
    "set CF_ROBOT_BROWSER_TESTS=1 and CHROME_BIN to run the browser smoke test",
)
class PoseBrowserTests(unittest.IsolatedAsyncioTestCase):
    async def test_tabs_readings_commands_stop_and_mobile_layout(self):
        root = Path(__file__).resolve().parents[1] / "src/cf_robot/operator"
        server = http.server.ThreadingHTTPServer(
            ("127.0.0.1", 0),
            functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(root)),
        )
        threading.Thread(target=server.serve_forever, daemon=True).start()
        chrome = None
        try:
            with tempfile.TemporaryDirectory(prefix="cf-pose-ui-") as profile:
                chrome = await asyncio.create_subprocess_exec(
                    os.environ.get("CHROME_BIN", "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"),
                    "--headless=new", "--no-first-run", "--no-default-browser-check",
                    "--disable-background-networking", "--disable-sync",
                    "--remote-debugging-port=0", f"--user-data-dir={profile}", "about:blank",
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
                port_file = Path(profile) / "DevToolsActivePort"
                async with asyncio.timeout(15):
                    while not port_file.exists():
                        await asyncio.sleep(0.1)
                port, endpoint = port_file.read_text().splitlines()
                async with connect(f"ws://127.0.0.1:{port}{endpoint}") as socket:
                    self.sequence = 0
                    self.events = []

                    async def send(method, params=None, session=None):
                        self.sequence += 1
                        payload = {"id": self.sequence, "method": method, "params": params or {}}
                        if session:
                            payload["sessionId"] = session
                        await socket.send(json.dumps(payload))
                        async with asyncio.timeout(15):
                            while True:
                                item = json.loads(await socket.recv())
                                if item.get("id") == self.sequence:
                                    self.assertNotIn("error", item)
                                    return item.get("result", {})
                                self.events.append(item)

                    target = (await send("Target.createTarget", {"url": "about:blank"}))["targetId"]
                    session = (await send("Target.attachToTarget", {
                        "targetId": target, "flatten": True,
                    }))["sessionId"]

                    async def cdp(method, params=None):
                        return await send(method, params, session)

                    async def evaluate(expression):
                        result = await cdp("Runtime.evaluate", {
                            "expression": expression, "returnByValue": True, "awaitPromise": True,
                        })
                        self.assertNotIn("exceptionDetails", result)
                        return result["result"].get("value")

                    async def wait_for(expression):
                        async with asyncio.timeout(5):
                            while not await evaluate(expression):
                                await asyncio.sleep(0.05)

                    await cdp("Page.enable")
                    await cdp("Runtime.enable")
                    await cdp("Emulation.setDeviceMetricsOverride", {
                        "width": 1440, "height": 1400, "deviceScaleFactor": 1, "mobile": False,
                    })
                    await cdp("Page.addScriptToEvaluateOnNewDocument", {"source": MOCK_BRIDGE})
                    origin = f"http://127.0.0.1:{server.server_port}"
                    await cdp("Page.navigate", {"url": origin})
                    await wait_for("document.querySelectorAll('#pose-readings tr').length === 6")
                    self.assertEqual(await evaluate(
                        "getComputedStyle(document.body).backgroundColor",
                    ), "rgb(24, 33, 31)")
                    self.assertEqual(await evaluate(
                        "getComputedStyle(document.body).backgroundImage",
                    ), "none")
                    await evaluate("document.getElementById('theme-toggle').click()")
                    self.assertEqual(await evaluate(
                        "document.documentElement.dataset.theme",
                    ), "light")
                    await evaluate("document.getElementById('theme-toggle').click()")
                    await evaluate("""
                      document.getElementById('pose-tab').click();
                      document.getElementById('controller-origin').value=location.origin;
                      document.getElementById('pair-controller').click();
                    """)
                    await wait_for("!document.getElementById('pose-start').disabled")
                    self.assertEqual(await evaluate("window.pairedAddress"), origin)
                    self.assertTrue(await evaluate("""
                      document.getElementById('teleop-panel').hidden &&
                      !document.getElementById('pose-panel').hidden
                    """))
                    await evaluate("document.getElementById('pose-start').click()")
                    await wait_for("document.getElementById('pose-state').textContent === 'READY'")
                    self.assertTrue(await evaluate("!document.getElementById('software-stop').disabled"))
                    self.assertEqual(await evaluate(
                        "document.querySelector('#pose-readings tr td:nth-child(3)').textContent",
                    ), "0.00")
                    await evaluate("""
                      document.getElementById('pose-joint').value='6';
                      document.getElementById('pose-joint').dispatchEvent(new Event('change'));
                      document.getElementById('pose-value').value='40';
                      document.getElementById('pose-command-confirm').checked=true;
                      document.getElementById('pose-send').click();
                    """)
                    await wait_for("window.testCommands.length === 1")
                    self.assertEqual(await evaluate("window.testCommands"), [
                        {"joint_id": 6, "mode": "percent", "value": 40},
                    ])
                    await wait_for("document.getElementById('pose-command-result').textContent.includes('succeeded')")
                    for width, height in ((1440, 1400), (390, 844)):
                        await cdp("Emulation.setDeviceMetricsOverride", {
                            "width": width, "height": height, "deviceScaleFactor": 1,
                            "mobile": width < 500,
                        })
                        self.assertTrue(await evaluate(
                            "document.documentElement.scrollWidth <= innerWidth",
                        ), await evaluate("""({
                          width:innerWidth, scroll:document.documentElement.scrollWidth,
                          overflow:[...document.querySelectorAll('body *')].filter(e =>
                            e.getBoundingClientRect().right > innerWidth &&
                            !e.closest('.pose-table-scroll')).map(e => ({
                              tag:e.tagName, id:e.id, class:e.className,
                              right:e.getBoundingClientRect().right
                            })).slice(0,20)
                        })"""))
                        if os.environ.get("CF_ROBOT_SCREENSHOT_DIR"):
                            directory = Path(os.environ["CF_ROBOT_SCREENSHOT_DIR"])
                            self.assertTrue(directory.is_dir())
                            await evaluate("document.getElementById('pose-panel').scrollIntoView()")
                            image = await cdp("Page.captureScreenshot", {"format": "png"})
                            (directory / f"pose-{width}.png").write_bytes(base64.b64decode(image["data"]))
                    await evaluate("document.getElementById('pose-stop').click()")
                    await wait_for("document.getElementById('pose-state').textContent === 'IDLE'")
                    self.assertFalse([event for event in self.events
                                      if event.get("method") == "Runtime.exceptionThrown"])
                chrome.terminate()
                await asyncio.wait_for(chrome.wait(), timeout=10)
                chrome = None
        finally:
            if chrome is not None:
                chrome.terminate()
                await asyncio.wait_for(chrome.wait(), timeout=10)
            await asyncio.to_thread(server.shutdown)
            server.server_close()


if __name__ == "__main__":
    unittest.main()
