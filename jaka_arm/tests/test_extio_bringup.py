# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the extended-IO slave recovery helper.

The payload shape is the whole point of this module: the controller silently
ignores `intid` and only honours `extio_mode_req: {submod, tarmode}` sent with
the App session token, so these tests pin it.
"""
import unittest
from unittest.mock import patch

from jaka_arm import extio_bringup as eb

IP = "192.168.10.90"
SUB = 54052


class FakeRC:
    def __init__(self, running_after=0):
        self.calls = 0
        self.running_after = running_after

    def is_extio_running(self):
        self.calls += 1
        return (0, 1 if self.calls > self.running_after else 0)


class FakeHTTP:
    """Records every POST and answers with a canned success envelope."""

    def __init__(self):
        self.calls = []

    def __call__(self, ip, path, payload, token=None):
        self.calls.append((path, payload, token))
        if path == "/user/login":
            return {"data": {"loginData": {"token": "T", "errinfo": {"errcode": 0}}}}
        if path == "/grpcRequest/handleRpc":
            return {"data": {"errcode": 0}}
        return {"data": {"errcode": 0, "errormsg": ""}}


class TestStartPayload(unittest.TestCase):
    def test_start_uses_tarmode_and_the_token(self):
        http = FakeHTTP()
        with patch.object(eb, "_post", http):
            code, _ = eb.set_slave_mode(IP, "T", SUB, run=True)
        self.assertEqual(code, 0)
        path, payload, token = http.calls[-1]
        self.assertEqual(path, "/grpcRequest/")
        self.assertEqual(payload["reqtype"], "MSGID_SETEXTIOMODE")
        self.assertEqual(payload["extio_mode_req"], {"submod": SUB, "tarmode": 1})
        self.assertEqual(token, "T")

    def test_stop_sends_tarmode_zero(self):
        http = FakeHTTP()
        with patch.object(eb, "_post", http):
            eb.set_slave_mode(IP, "T", SUB, run=False)
        self.assertEqual(http.calls[-1][1]["extio_mode_req"]["tarmode"], 0)


class TestEnsureRunning(unittest.TestCase):
    def test_already_running_touches_no_network(self):
        http = FakeHTTP()
        rc = FakeRC(running_after=0)
        rc.calls = 1  # first probe says "running"
        with patch.object(eb, "_post", http):
            ok, reason = eb.ensure_running(rc, IP, SUB, "u", "p")
        self.assertTrue(ok)
        self.assertEqual(reason, "")
        self.assertEqual(http.calls, [])

    def test_missing_password_is_reported_not_guessed(self):
        ok, reason = eb.ensure_running(FakeRC(running_after=1), IP, SUB, "u", "")
        self.assertFalse(ok)
        self.assertIn("web_password", reason)

    def test_start_sequence_captures_starts_and_hands_back(self):
        http = FakeHTTP()
        rc = FakeRC(running_after=1)  # first probe down, then up after start
        with patch.object(eb, "_post", http), patch.object(eb.time, "sleep", lambda *_: None):
            ok, _ = eb.ensure_running(rc, IP, SUB, "u", "p")
        self.assertTrue(ok)
        paths = [c[0] + ":" + (c[1].get("rpcKey") or c[1].get("reqtype") or "")
                 for c in http.calls]
        self.assertEqual(paths, [
            "/user/login:",
            "/grpcRequest/handleRpc:CaptureCtrl",
            "/grpcRequest/:MSGID_SETEXTIOMODE",
            "/grpcRequest/handleRpc:TransCtrl",
            "/user/logout:",
        ])


if __name__ == "__main__":
    unittest.main()
