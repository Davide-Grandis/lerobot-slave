import contextlib
import io
import os
import unittest
from unittest.mock import patch

from cf_robot.pairing import ADJECTIVES, NOUNS, generate_pairing_code

try:
    import httpx

    from cf_robot import web
except ImportError:
    web = None


class PairingCodeTests(unittest.TestCase):
    def test_word_lists_are_unique_lowercase_words(self):
        for words in (ADJECTIVES, NOUNS):
            self.assertEqual(len(words), 128)
            self.assertEqual(len(set(words)), len(words))
            for word in words:
                self.assertRegex(word, r"^[a-z]+$")

    def test_uses_secure_choice_for_each_word(self):
        with patch("cf_robot.pairing.secrets.choice", side_effect=["quiet", "otter"]) as choice:
            self.assertEqual(generate_pairing_code(), "quiet-otter")
        self.assertEqual(choice.call_args_list, [
            unittest.mock.call(ADJECTIVES), unittest.mock.call(NOUNS),
        ])

    def test_generated_codes_have_two_words(self):
        for _ in range(100):
            code = generate_pairing_code()
            self.assertRegex(code, r"^[a-z]+-[a-z]+$")
            adjective, noun = code.split("-")
            self.assertIn(adjective, ADJECTIVES)
            self.assertIn(noun, NOUNS)


@unittest.skipIf(web is None, "install the web bridge dependencies and httpx")
class PairingBridgeTests(unittest.IsolatedAsyncioTestCase):
    async def test_generated_and_overridden_codes_authenticate(self):
        origin = "https://cf-robot.davideslab.eu"
        for explicit, environment, expected in (
            (None, "", "quiet-otter"),
            (None, "custom-environment-token", "custom-environment-token"),
            ("explicit-token", "custom-environment-token", "explicit-token"),
        ):
            with self.subTest(explicit=explicit, environment=environment), patch.dict(
                os.environ, {
                    "CF_ROBOT_BRIDGE_TOKEN": environment,
                    "CF_ROBOT_ORIGIN_KEY": "t" * 64,
                },
            ), patch("cf_robot.web.generate_pairing_code", return_value="quiet-otter") as generate:
                app = web.create_app(
                    "ws://127.0.0.1:8765", None, "leader",
                    operator_origin=origin, bridge_token=explicit,
                )
                self.assertEqual(generate.call_count, int(not explicit and not environment))
                async with httpx.AsyncClient(
                    transport=httpx.ASGITransport(app=app),
                    base_url="http://127.0.0.1:8080",
                ) as client:
                    for supplied in ("", "wrong-code", expected):
                        response = await client.get("/api/config", headers={
                            "Origin": origin, "X-CF-Robot-Bridge": supplied,
                        })
                        self.assertEqual(response.status_code, 200 if supplied == expected else 403)
                    response = await client.get("/api/config", headers={
                        "Origin": "https://untrusted.example", "X-CF-Robot-Bridge": expected,
                    })
                    self.assertEqual(response.status_code, 403)

    async def test_cli_prints_same_code_passed_to_app(self):
        for environment, expected in (("", "quiet-otter"), ("custom-token", "custom-token")):
            with self.subTest(environment=environment), patch.dict(
                os.environ, {"CF_ROBOT_BRIDGE_TOKEN": environment},
            ), patch("sys.argv", [
                "cf-robot-web", "--url", "https://robot.example",
                "--arm-id", "leader", "--operator-origin", "https://console.example",
            ]), patch(
                "cf_robot.web.generate_pairing_code", return_value="quiet-otter",
            ) as generate, patch("cf_robot.web.create_app") as create, patch(
                "cf_robot.web.uvicorn.run",
            ), contextlib.redirect_stdout(io.StringIO()) as output:
                web.main()
                self.assertEqual(create.call_args.kwargs["bridge_token"], expected)
                self.assertEqual(output.getvalue().splitlines()[-1], expected)
                self.assertEqual(generate.call_count, int(not environment))
