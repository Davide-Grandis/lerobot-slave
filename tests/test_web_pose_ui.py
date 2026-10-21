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
window.testModeRequests = [];
window.testControl = {running:false,state:'idle',camera_live:false};
window.testPoseOffline = false;
window.testLeaderOffset = 0;
window.testFollowerOffset = 0;
window.testMissingLeader = false;
window.testSampleAge = 0;
window.testVideoRequests = 0;
window.testJ6Torque = true;
const keys = ['shoulder_pan.pos','shoulder_lift.pos','elbow_flex.pos',
              'wrist_flex.pos','wrist_roll.pos','gripper.pos'];
const sample = side => ({
  type:'pose_sample', side, sampled_at:Date.now()/1000-window.testSampleAge,
  values:Object.fromEntries(keys.map((k,i)=>[
    k,i*2+(side==='leader'?window.testLeaderOffset:window.testFollowerOffset)
  ])),
  raw:Object.fromEntries(keys.map(k=>[k,2048])),
  torque:Object.fromEntries(keys.map((k,i)=>[k,i===5?window.testJ6Torque:true])),
  limits:Object.fromEntries(keys.map(k=>[k,{min:100,max:3900}]))
  ,calibration: {
    state:'consistent',side,arm_id:side,port:'/dev/fake',lerobot_version:'test',
    inspected_at:1,loaded_sha256:'test',validation_key:'test',warnings:[],
    file:{path:'/test/calibration.json',matches_loaded:true},
    joints:Object.fromEntries(keys.map((k,i)=>[k,{
      id:i+1,mode:i===5?'range_0_100':'degrees',unit:i===5?'%':'deg',
      state:'consistent',warnings:[],saved:null,live:null
    }]))
  }
});
let running = false, command = null, sides = 'both', captures = [];
window.fetch = async (url, options={}) => {
  const path = new URL(url,location.href).pathname;
  const reply = (payload,status=200) => new Response(JSON.stringify(payload),
    {status,headers:{'Content-Type':'application/json'}});
  if (!path.startsWith('/api/')) return originalFetch(url,options);
  if (path === '/api/config') {
    window.pairedAddress = new URL(url).origin;
    return reply({api_version:1});
  }
  if (path === '/api/status') return reply(window.testControl);
  if (path === '/api/mode') {
    const body=JSON.parse(options.body);
    window.testModeRequests.push(body);
    window.testControl={...window.testControl,mode:body.mode,state:'synchronizing',
      teleop_message:'WAIT - synchronizing follower'};
    return reply(window.testControl);
  }
  if (path === '/api/heartbeat') return reply({ok:true});
  if (path === '/api/video/frame') { window.testVideoRequests++; return reply({},503); }
  if (path === '/api/pose/start') {
    sides=JSON.parse(options.body).sides || 'both'; running=true; return reply({running});
  }
  if (path === '/api/pose/stop') { running=false; return reply({running}); }
  if (path === '/api/pose/command') {
    const body = JSON.parse(options.body);
    window.testCommands.push(body);
    command={type:'command_result',command_id:window.testCommands.length,
             joint_id:body.joint_id,mode:body.mode,state:'succeeded'};
    if (body.mode === 'torque' && body.joint_id === 6) window.testJ6Torque=body.enabled;
    if (body.mode === 'calibration_preview') {
      command.state='preview';
      command.calibration_preview={
        token:'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa',expires_in_s:60,
        current:{homing_offset:-208,range_min:0,range_max:4095},
        proposed:{homing_offset:-873,range_min:2038,range_max:3488},
        current_raw:2100,projected_raw:2765
      };
    }
    if (body.mode === 'calibration_restore') {
      command.calibration_restore={torque_enabled:false};
    }
    return reply({command_id:command.command_id});
  }
  if (path === '/api/pose/heartbeat') return reply({ok:true});
  if (path === '/api/calibration/status') return reply({running:false,state:'idle'});
  if (path === '/api/pose/capture') {
    const body=JSON.parse(options.body);
    captures.push({...body,observed:body.expected,error:0,passed:true,unit:'deg'});
    return reply({ok:true});
  }
  if (path === '/api/pose/status' && window.testPoseOffline) return reply({},503);
  if (path === '/api/pose/status') return reply({
    running,state:running?'ready':'idle',command_pending:false,
    leader:running&&sides!=='follower'&&!window.testMissingLeader?sample('leader'):null,
    follower:running&&sides!=='leader'?sample('follower'):null,command,captures,analysis:[]
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
                    self.assertTrue(await evaluate("""
                      document.querySelector('main').firstElementChild.matches('.intro') &&
                      document.querySelector('main').children[1].id === 'pairing-panel' &&
                      document.querySelector('h1').textContent === 'SO-ARM101 Console' &&
                      parseFloat(getComputedStyle(document.querySelector('h1')).fontSize) <= 48 &&
                      document.querySelector('.intro-copy') === null &&
                      document.querySelector('.device-tag') === null &&
                      document.getElementById('pairing-panel').getBoundingClientRect().bottom <
                        document.querySelector('.utility-tabs').getBoundingClientRect().top &&
                      document.querySelector('.camera-panel').offsetParent === null &&
                      document.querySelector('.session-panel').offsetParent === null &&
                      document.getElementById('pose-tab').getAttribute('aria-selected') === 'true'
                    """))
                    self.assertTrue(await evaluate("document.getElementById('pose-start').disabled"))
                    self.assertIn("Pair the local controller", await evaluate(
                        "document.getElementById('pose-start-reason').textContent",
                    ))
                    self.assertTrue(await evaluate("""[
                      'Compare live leader action values with follower servo readback.',
                      'Startup preserves current torque. Use explicit follower commands to change it.',
                      'Arms may sag when torque is disabled. Support both arms first.'
                    ].every(text => !document.body.textContent.includes(text))"""))
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
                    self.assertEqual(await evaluate("window.testVideoRequests"), 0)
                    self.assertTrue(await evaluate("document.getElementById('pose-start-reason').hidden"))
                    for flag, reason in (
                        ("running", "Disconnect teleoperation"),
                        ("estop_latched", "Re-arm the software stop"),
                    ):
                        await evaluate(f"window.testControl.{flag}=true")
                        await wait_for(
                            "document.getElementById('pose-start').disabled && "
                            f"document.getElementById('pose-start-reason').textContent.includes('{reason}')",
                        )
                        await evaluate(f"window.testControl.{flag}=false")
                        await wait_for("!document.getElementById('pose-start').disabled")
                    await evaluate("window.testPoseOffline=true")
                    await wait_for(
                        "document.getElementById('pose-start').disabled && "
                        "document.getElementById('pose-start-reason').textContent.includes('cf-robot-web')",
                    )
                    await evaluate("window.testPoseOffline=false")
                    await wait_for("!document.getElementById('pose-start').disabled")
                    self.assertTrue(await evaluate("""
                      document.getElementById('teleop-panel').hidden &&
                      !document.getElementById('pose-panel').hidden
                    """))
                    await evaluate("document.getElementById('pose-start').click()")
                    await wait_for("document.getElementById('pose-state').textContent === 'READY'")
                    await wait_for("document.getElementById('pose-leader-model').dataset.state === 'live'")
                    self.assertIn("File matches loaded: true", await evaluate(
                        "document.getElementById('calibration-inspectors').textContent",
                    ))
                    await evaluate("""
                      document.getElementById('calibration-capture-label').value='Measured reference';
                      document.getElementById('calibration-expected').value='0';
                      document.getElementById('calibration-capture-confirm').checked=true;
                      document.getElementById('calibration-capture').click();
                    """)
                    await wait_for("document.getElementById('calibration-evidence').textContent.includes('within tolerance')")
                    self.assertTrue(await evaluate("""
                      document.getElementById('pose-follower-model').dataset.state === 'live' &&
                      document.getElementById('pairing-panel').getBoundingClientRect().bottom <
                        document.getElementById('pose-diagnostics').getBoundingClientRect().top &&
                      document.getElementById('pose-diagnostics').getBoundingClientRect().bottom <=
                        document.querySelector('.pose-arm-grid').getBoundingClientRect().top
                    """))
                    self.assertTrue(await evaluate("!document.getElementById('software-stop').disabled"))
                    self.assertEqual(await evaluate(
                        "document.querySelector('#pose-readings tr td:nth-child(3)').textContent",
                    ), "0.00")
                    await evaluate("""
                      window.leaderImage = document.getElementById('pose-leader-canvas').toDataURL();
                      window.followerImage = document.getElementById('pose-follower-canvas').toDataURL();
                      window.testLeaderOffset = 20;
                    """)
                    await wait_for("document.getElementById('pose-leader-canvas').toDataURL() !== window.leaderImage")
                    self.assertTrue(await evaluate(
                        "document.getElementById('pose-follower-canvas').toDataURL() === window.followerImage",
                    ))
                    self.assertIn("20.0", await evaluate(
                        "document.querySelector('#pose-leader-joints strong').textContent",
                    ))
                    await evaluate("""
                      window.leaderImage = document.getElementById('pose-leader-canvas').toDataURL();
                      window.testFollowerOffset = 10;
                    """)
                    await wait_for("document.getElementById('pose-follower-canvas').toDataURL() !== window.followerImage")
                    self.assertTrue(await evaluate(
                        "document.getElementById('pose-leader-canvas').toDataURL() === window.leaderImage",
                    ))
                    await evaluate("window.testSampleAge = 5")
                    await wait_for("document.getElementById('pose-leader-model').dataset.state === 'stale'")
                    self.assertEqual(await evaluate(
                        "document.getElementById('pose-follower-state').textContent",
                    ), "LAST SAMPLE")
                    await evaluate("window.testSampleAge = 0; window.testMissingLeader = true")
                    await wait_for("document.getElementById('pose-leader-model').dataset.state === 'waiting'")
                    self.assertFalse(await evaluate("document.getElementById('pose-leader-empty').hidden"))
                    self.assertEqual(await evaluate(
                        "document.getElementById('pose-follower-model').dataset.state",
                    ), "live")
                    await evaluate("window.testMissingLeader = false")
                    await wait_for("document.getElementById('pose-leader-model').dataset.state === 'live'")
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
                    self.assertTrue(await evaluate("document.getElementById('pose-restore-preview').disabled"))
                    await evaluate("""
                      document.getElementById('pose-command-mode').value='disable';
                      document.getElementById('pose-command-mode').dispatchEvent(new Event('change'));
                      document.getElementById('pose-command-confirm').checked=true;
                      document.getElementById('pose-send').click();
                    """)
                    await wait_for("!document.getElementById('pose-restore-preview').disabled")
                    self.assertTrue(await evaluate("document.getElementById('pose-restore-apply').disabled"))
                    await evaluate("document.getElementById('pose-restore-preview').click()")
                    await wait_for("!document.getElementById('pose-restore-apply').disabled")
                    self.assertIn("2765", await evaluate(
                        "document.getElementById('pose-restore-result').textContent",
                    ))
                    await evaluate("""
                      window.confirm=()=>true;
                      document.getElementById('pose-restore-confirmation').value='RESTORE J6';
                      document.getElementById('pose-restore-supported').checked=true;
                      document.getElementById('pose-restore-apply').click();
                    """)
                    await wait_for("window.testCommands.length === 4")
                    self.assertEqual(await evaluate("window.testCommands[3]"), {
                        "joint_id": 6, "mode": "calibration_restore",
                        "token": "a" * 32, "confirmation": "RESTORE J6",
                    })
                    self.assertFalse(await evaluate("window.testJ6Torque"))
                    for width, height in ((1440, 1400), (390, 844)):
                        await cdp("Emulation.setDeviceMetricsOverride", {
                            "width": width, "height": height, "deviceScaleFactor": 1,
                            "mobile": width < 500,
                        })
                        self.assertTrue(await evaluate("""(() => {
                          const leader = document.getElementById('pose-leader-model').getBoundingClientRect();
                          const follower = document.getElementById('pose-follower-model').getBoundingClientRect();
                          return Math.abs(leader.width - follower.width) < 1 &&
                            Math.abs(leader.height - follower.height) < 1 &&
                            (innerWidth > 690 ? leader.top === follower.top : follower.top > leader.bottom);
                        })()"""))
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
                            await evaluate("document.querySelector('.pose-arm-grid').scrollIntoView()")
                            image = await cdp("Page.captureScreenshot", {"format": "png"})
                            (directory / f"pose-{width}.png").write_bytes(base64.b64decode(image["data"]))
                    await evaluate("document.getElementById('pose-stop').click()")
                    await wait_for("document.getElementById('pose-state').textContent === 'IDLE'")
                    await evaluate("""
                      document.getElementById('pose-sides').value='leader';
                      document.getElementById('pose-start').click();
                    """)
                    await wait_for("document.getElementById('pose-leader-model').dataset.state === 'live'")
                    self.assertEqual(await evaluate(
                        "document.getElementById('pose-follower-model').dataset.state",
                    ), "waiting")
                    await evaluate("document.getElementById('pose-stop').click()")
                    await wait_for("document.getElementById('pose-state').textContent === 'IDLE'")
                    await evaluate("document.getElementById('teleop-tab').click()")
                    self.assertTrue(await evaluate("""
                      document.querySelector('.camera-panel').offsetParent !== null &&
                      document.querySelector('.session-panel').offsetParent !== null &&
                      document.getElementById('pose-panel').hidden
                    """))
                    await evaluate("""
                      window.testControl={...window.testControl,running:true,state:'ready',
                        mode:'manual',leader_connected:true};
                      window.confirm=message=>{window.testModeConfirmation=message;return true;};
                    """)
                    await wait_for("!document.getElementById('leader-mode').disabled")
                    await evaluate("document.getElementById('leader-mode').click()")
                    await wait_for("document.getElementById('session-state').textContent === 'WAIT'")
                    self.assertEqual(await evaluate("window.testModeRequests"), [
                        {"mode": "leader", "confirm_sync": True},
                    ])
                    self.assertIn("one joint at a time", await evaluate("window.testModeConfirmation"))
                    self.assertIn("WAIT", await evaluate("document.getElementById('session-detail').textContent"))
                    self.assertFalse(await evaluate("document.getElementById('stop-control').disabled"))
                    await evaluate("""
                      window.testControl={...window.testControl,state:'ready',
                        teleop_message:'Ready for teleop'};
                    """)
                    await wait_for("document.getElementById('session-state').textContent === 'Ready for teleop'")
                    self.assertEqual(await evaluate(
                        "document.getElementById('session-detail').textContent",
                    ), "Ready for teleop")
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
