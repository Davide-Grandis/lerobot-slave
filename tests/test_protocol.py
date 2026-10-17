from decimal import Decimal
import unittest

from cf_robot.protocol import JOINTS, ProtocolError, action_message, validate_action, validate_command


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.action = {joint: 1.0 for joint in JOINTS}

    def test_round_trip(self):
        message = action_message(1, self.action)
        self.assertEqual(validate_command(message, 0, now=message["sent_at"]), (1, self.action))

    def test_rejects_missing_or_extra_joints(self):
        for action in ({}, {**self.action, "other.pos": 3}):
            with self.subTest(action=action), self.assertRaises(ProtocolError):
                validate_action(action)

    def test_rejects_nonfinite_and_boolean(self):
        for value in (float("nan"), float("inf"), True, "1", 361):
            with self.subTest(value=value), self.assertRaises(ProtocolError):
                validate_action({**self.action, JOINTS[0]: value})

    def test_converts_numeric_scalar_to_wire_float(self):
        action = validate_action({**self.action, JOINTS[0]: Decimal("1.25")})
        self.assertEqual(action[JOINTS[0]], 1.25)

    def test_rejects_replay_stale_and_future(self):
        message = action_message(2, self.action)
        with self.assertRaises(ProtocolError):
            validate_command(message, 2, now=message["sent_at"])
        with self.assertRaises(ProtocolError):
            validate_command(message, 0, now=message["sent_at"] + 0.6)
        with self.assertRaises(ProtocolError):
            validate_command(message, 0, now=message["sent_at"] - 0.3)

    def test_rejects_other_protocol_versions(self):
        message = action_message(1, self.action)
        message["version"] = 2
        with self.assertRaises(ProtocolError):
            validate_command(message, 0, now=message["sent_at"])


if __name__ == "__main__":
    unittest.main()
