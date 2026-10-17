import os
import unittest
from unittest.mock import patch

from cf_robot.auth import access_headers


class AuthTests(unittest.TestCase):
    def test_remote_requires_https_and_credentials(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(ValueError):
                access_headers("wss://robot.example.com/control")
            with self.assertRaises(ValueError):
                access_headers("ws://robot.example.com/control")
            with self.assertRaises(ValueError):
                access_headers("ws://127.0.0.1:8765/control")

    def test_credentials_only_in_headers(self):
        key = "k" * 64
        with patch.dict(os.environ, {"CF_ACCESS_CLIENT_ID": "id", "CF_ACCESS_CLIENT_SECRET": "secret",
                                     "CF_ROBOT_ORIGIN_KEY": key}):
            self.assertEqual(
                access_headers("wss://robot.example.com/control"),
                {"CF-Access-Client-Id": "id", "CF-Access-Client-Secret": "secret", "X-CF-Robot-Key": key},
            )
            self.assertEqual(access_headers("ws://127.0.0.1:8765/control"), {"X-CF-Robot-Key": key})
            with self.assertRaises(ValueError):
                access_headers("wss://id:secret@robot.example.com/control")


if __name__ == "__main__":
    unittest.main()
