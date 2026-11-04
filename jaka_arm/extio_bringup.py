# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Start the controller's extended-IO Modbus slave from inside the provider.

The SDK cannot start that slave: ``MSGID_SETEXTIOMODE`` sent without the App
session token is a silent no-op, and the App itself posts
``extio_mode_req: {submod, tarmode}`` rather than the bare ``intid``. These
helpers mirror the App's own calls, so the provider recovers the slave without
a browser or a human.

Preconditions the controller enforces, and this module respects:

* the arm must be **powered** (the gripper draws its supply from it) and
  **disarmed** — the App refuses to start the slave while the arm is enabled;
* the control source must be **local** while the request is posted, so the
  helper captures it and hands it back to Remote afterwards.

Standard library only: the provider keeps no browser dependency.
"""
from __future__ import annotations

import json
import ssl
import time
import urllib.error
import urllib.request

_CTX = ssl._create_unverified_context()  # the controller serves a self-signed cert
_LOGIN_TIMEOUT_S = 8.0


def _post(ip: str, path: str, payload: dict, token: str | None = None) -> dict:
    headers = {"Content-Type": "application/json", "robotIp": ip, "jointNums": "6"}
    if token:
        headers["Authorization"] = token
    request = urllib.request.Request(
        f"https://{ip}/webapp/api{path}",
        data=json.dumps(payload).encode(),
        method="POST",
        headers=headers,
    )
    try:
        with urllib.request.urlopen(request, timeout=_LOGIN_TIMEOUT_S, context=_CTX) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as exc:  # 500 still carries a JSON reason
        body = exc.read().decode(errors="replace")
        try:
            return json.loads(body)
        except json.JSONDecodeError:
            raise RuntimeError(f"HTTP {exc.code}: {body[:200]}") from exc


def login(ip: str, user: str, password: str) -> str:
    reply = _post(ip, "/user/login", {
        "username": user, "password": password, "ip": ip,
        "ismanual": True, "robot_index": 0, "page": "login",
    })
    data = reply.get("data", {}).get("loginData", {})
    err = data.get("errinfo") or {}
    if err.get("errcode"):
        raise RuntimeError(f"{err.get('errcode')}: {err.get('errormsg')}")
    token = data.get("token")
    if not token:
        raise RuntimeError(f"no token in login reply: {reply}")
    return token


def logout(ip: str, token: str) -> None:
    try:
        _post(ip, "/user/logout", {"ip": ip, "robot_index": 0}, token)
    except Exception:  # noqa: BLE001 - best effort, never mask the real error
        pass


def _handle_rpc(ip: str, token: str, key: str, **fields) -> int:
    reply = _post(ip, "/grpcRequest/handleRpc",
                  {"rpcKey": key, "robot_index": 0, "ip": ip, **fields}, token)
    return int(reply.get("data", {}).get("errcode", -1))


def capture_control(ip: str, token: str) -> int:
    """Take the control source into the App so config/start requests are honoured."""
    return _handle_rpc(ip, token, "CaptureCtrl")


def transfer_control(ip: str, token: str) -> int:
    """Hand the control source back so SDK writes are allowed again."""
    return _handle_rpc(ip, token, "TransCtrl", mode=3)


def set_slave_mode(ip: str, token: str, submodule: int, run: bool) -> tuple[int, str]:
    """Start (tarmode=1) or stop (tarmode=0) one extended-IO slave."""
    reply = _post(ip, "/grpcRequest/", {
        "reqtype": "MSGID_SETEXTIOMODE",
        "robot_index": 0,
        "ip": ip,
        "extio_mode_req": {"submod": int(submodule), "tarmode": 1 if run else 0},
    }, token)
    data = reply.get("data", {})
    return int(data.get("errcode", -1)), str(data.get("errormsg") or "")


def running(rc) -> bool:
    ret = rc.is_extio_running()
    return ret[0] == 0 and bool(ret[1])


def ensure_running(rc, ip: str, submodule: int, user: str, password: str,
                   wait_s: float = 10.0) -> tuple[bool, str]:
    """Bring the slave up when it is down. Returns (ok, reason)."""
    if running(rc):
        return True, ""
    if not password:
        return False, ("web_password is not configured, so the controller's "
                       "Modbus slave cannot be started from here")
    try:
        token = login(ip, user, password)
    except Exception as exc:  # noqa: BLE001
        return False, f"App login failed: {exc}"
    try:
        code = capture_control(ip, token)
        if code != 0:
            return False, f"CaptureCtrl failed with {code} (is the App logged in?)"
        code, msg = set_slave_mode(ip, token, submodule, run=True)
        if code != 0:
            return False, f"MSGID_SETEXTIOMODE failed: {code} {msg}"
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if running(rc):
                return True, ""
            time.sleep(0.5)
        return False, (f"slave {submodule} did not come up within {wait_s:.0f} s "
                       "(is the arm powered and disarmed?)")
    finally:
        transfer_control(ip, token)
        logout(ip, token)
