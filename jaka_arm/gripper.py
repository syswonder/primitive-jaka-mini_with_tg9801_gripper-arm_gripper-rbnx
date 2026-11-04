# SPDX-License-Identifier: MIT
# -*- coding: utf-8 -*-
# Adapted from AIIT-Roboarm (arm/jaka_gripper.py), MIT License,
# Copyright (c) 2025 Xu Jinyang & Hou Yunlong. See NOTICE.
"""
JakaGripper：经 JAKA 扩展 IO（Coboπ Modbus 从站）控制 HKV TG-9801 夹爪。

为什么不是 RS485 信号表（2026-11-03 实机结论，固件 3.3.15 + SDK 2.3.3beta2）：
  · `get_rs485_signal_info()` 在 legacy 登录下必 SIGSEGV、grpc 登录下恒返 8 个空槽；
  · `add_tio_rs_signal` 返 0 但不落表，`update_tio_rs_signal` 恒返 260；
  · 换固件、换通道号/信号类型、切自动模式、改调用顺序都无效。
改走控制器自带的「扩展 IO」：Coboπ 里建 TIO Modbus RTU 从站轮询夹爪寄存器，
SDK 侧只做模拟量读写：

    读  get_analog_input(IO_EXTEND, ai_idx, submodule_id)
    写  set_analog_output(IO_EXTEND, ao_idx, value, submodule_id)

通道映射与 Coboπ 的从站配置一一对应（见 EXTIO_AI / EXTIO_AO）。

寄存器语义（2026-11-03 实物确认）：
  · 位置 0x0008 是**闭合量**：1000 = 全闭、0 = 全开（与常见「1000=全开」相反）；
  · 动作 0x000B：1 = 夹取（位置走到 1000）、0 = 松开（走到 0）。

前置条件（缺一不可）：
  · Coboπ 里从站在运行（`is_extio_running()` 为 1），否则读写都返 10003；
  · 控制器控制源在「远程/Remote」，否则写操作返 10007（gRPC Code 7）。

只保留扩展 IO 能覆盖的通道：位置/电流/状态（读）与速度/动作（写）。
其余寄存器（三维力、目标力、力位参数、出厂信息等）没有配置通道，读返回 None、
写返回 -1 并打印一次告警——需要时在 Coboπ 里加通道并扩展这里的映射。

连接模式：
  · robot=None（默认）：独立模式，自建 jkrc.RC 并 login，close() 时 logout；
  · robot=<RC实例>：共享模式，复用外部已 login 的连接，close() 不 logout。
"""
import time

import jkrc

# IOType：0 = 控制柜面板 IO，1 = 工具 IO，2 = 扩展 IO
IO_EXTEND = 2

# 夹爪 Modbus 寄存器地址（TG-9801 手册，与 hkv_tg9801/modbus.py 一致）
REG = {
    "force_fx": 0x0000,        # 三维力 Fx(有符号int16)
    "force_fy": 0x0001,        # Fy
    "force_fz": 0x0002,        # Fz
    "state": 0x0006,           # 夹持状态 0空闲/1运动/2夹稳/3打滑
    "hardness": 0x0007,        # 硬度
    "position": 0x0008,        # 位置（闭合量：1000 全闭、0 全开）
    "current": 0x0009,         # 电流(mA)
    "adaptive_speed": 0x000A,  # 自适应速度
    "action": 0x000B,          # 动作 0松开/1夹取
    "protocol": 0x000C,        # 协议 1自定义/2Modbus
    "device_id": 0x000D,       # 从站地址
    "control_mode": 0x000E,    # 控制模式 0自适应/1力位
    "force_pos_speed": 0x000F, # 力位速度
    "force_pos_target": 0x0010,# 力位目标位置
    "fingertip_calibration": 0x0011,  # 指尖力强制校准(维护用)
    "target_force": 0x0012,    # 目标力
    "factory_number": 0x0013,  # 出厂编号(输入寄存器, 6个ASCII)
    "grip_count": 0x0019,      # 夹取次数(u32, 小端序)
    "follow_speed": 0x001B,    # 跟随速度
    "fingertip_zero": 0x001D,  # 指尖力硬件清零(维护用)
    "baudrate": 0x0021,        # 波特率(u32, 小端序)
    "error_flags": 0x0023,     # 错误标志
    "open_position": 0x002D,   # 张开位置
    "close_position": 0x002E,  # 闭合位置
    "force_pos_state": 0x002F, # 力位状态
    "eeprom_enabled": 0x00EE,  # EEPROM开关
}

# 扩展 IO 通道映射（Coboπ → 设置/硬件与通信/Modbus 设置 里那个从站）
# 只列当前从站实际配了的通道；改配置时同步改这里。
EXTIO_AI = {
    "position": 0,        # 0x0008 位置（闭合量）
    "current": 1,         # 0x0009 电流 mA
    "state": 2,           # 0x0006 夹持状态
    "force_fx": 3,        # 0x0000 力 Fx（有符号，按 uint16 读后转）
    "force_fy": 4,        # 0x0001 力 Fy
    "force_fz": 5,        # 0x0002 力 Fz
    "hardness": 6,        # 0x0007 硬度
    "error_flags": 7,     # 0x0023 错误标志
    "force_pos_state": 8, # 0x002F 力位状态
    "grip_count_lo": 9,   # 0x0019 夹取次数低 16 位
    "grip_count_hi": 10,  # 0x001A 夹取次数高 16 位
}
EXTIO_AO = {
    "adaptive_speed": 0,   # 0x000A 自适应速度
    "action": 1,           # 0x000B 动作 1=夹取 0=松开
    "control_mode": 2,     # 0x000E 控制模式 0=自适应 1=力位
    "force_pos_speed": 3,  # 0x000F 力位速度
    "force_pos_target": 4, # 0x0010 力位目标位置
    "target_force": 5,     # 0x0012 目标力
    "follow_speed": 6,     # 0x001B 跟随速度
    "open_position": 7,    # 0x002D 张开位置
    "close_position": 8,   # 0x002E 闭合位置
}

# 寄存器地址 → 扩展 IO 通道，由上面两张表反推
_ADDR_TO_AI = {REG[name]: idx for name, idx in EXTIO_AI.items() if name in REG}
_ADDR_TO_AO = {REG[name]: idx for name, idx in EXTIO_AO.items() if name in REG}


class JakaGripper:
    def __init__(self, robot=None, ip="10.5.5.100", submodule_id=54052):
        self.submodule_id = int(submodule_id)
        self._shared_robot = robot is not None
        if self._shared_robot:
            self.robot = robot
        else:
            self.robot = jkrc.RC(ip)
            ret = self.robot.login()
            if ret[0] != 0:
                raise ConnectionError(f"login 失败: {ret}")
        self._warned = set()

    def close(self) -> None:
        if not self._shared_robot:
            self.robot.logout()

    # ---------- 扩展 IO 可用性 ----------
    def extio_running(self) -> bool:
        ret = self.robot.is_extio_running()
        return ret[0] == 0 and bool(ret[1])

    def _warn_once(self, key: str, message: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            print(message, flush=True)

    def power_on(self) -> None:
        """机械臂上电，并等夹爪扩展 IO 就绪（有电流读数）。"""
        self.robot.power_on()
        time.sleep(2)
        # 从站没跑时读到的仍是上一次轮询的缓存值，所以先看运行标志再等电流
        if not self.extio_running():
            self._warn_once(
                "extio",
                "[gripper] 扩展 IO 从站未运行：请在 Coboπ 的 Modbus 设置里启动从站",
            )
        for _ in range(8):
            cur = self._read_reg(REG["current"])
            if cur:
                return
            time.sleep(0.2)
        print("Warning: 夹爪无电流读数，检查从站是否运行、RS485 接线与供电", flush=True)

    def power_off(self) -> None:
        """机械臂下电。"""
        self.robot.power_off()

    # ---------- 底层：模拟量读写 ----------
    def _read_ai(self, idx: int):
        ret = self.robot.get_analog_input(IO_EXTEND, int(idx), self.submodule_id)
        if ret[0] != 0 or len(ret) < 2:
            return None
        return int(round(float(ret[1])))

    def _write_ao(self, idx: int, value) -> int:
        ret = self.robot.set_analog_output(
            IO_EXTEND, int(idx), float(value), self.submodule_id)
        return int(ret[0])

    # ---------- 读寄存器（只覆盖已配置的 AI 通道） ----------
    def _read_reg(self, addr: int):
        idx = _ADDR_TO_AI.get(addr)
        if idx is None:
            self._warn_once(
                f"na-{addr}",
                f"[gripper] 寄存器 0x{addr:04X} 没有扩展 IO 通道，读返回 None",
            )
            return None
        return self._read_ai(idx)

    def _write_reg(self, addr: int, value: int) -> int:
        idx = _ADDR_TO_AO.get(addr)
        if idx is None:
            self._warn_once(
                f"wr-{addr}",
                f"[gripper] 寄存器 0x{addr:04X} 没有可写的扩展 IO 通道，写被忽略",
            )
            return -1
        return self._write_ao(idx, value)

    # ---------- 运动接口 ----------
    def _set_speed(self, speed: int) -> int:
        return self._write_ao(EXTIO_AO["adaptive_speed"], int(speed))

    def _set_action(self, action: int) -> int:
        return self._write_ao(EXTIO_AO["action"], int(action))

    def grip(self, speed: int = 1000) -> int:
        """夹取：先写速度，再写动作=1（位置走到 1000）。"""
        code = self._set_speed(speed)
        if code != 0:
            return code
        return self._set_action(1)

    def release(self, speed: int = 1000) -> int:
        """松开：先写速度，再写动作=0（位置走到 0）。"""
        code = self._set_speed(speed)
        if code != 0:
            return code
        return self._set_action(0)

    def force_position_control(self, speed: int, position: int,
                               force: int = None) -> int:
        """力位控制：切模式=1 → 写速度/目标位置/目标力 → 触发夹取。"""
        code = self._write_reg(REG["control_mode"], 1)
        if code != 0:
            return code
        code = self._write_reg(REG["force_pos_speed"], int(speed))
        if code != 0:
            return code
        code = self._write_reg(REG["force_pos_target"], int(position))
        if code != 0:
            return code
        if force is not None:
            code = self._write_reg(REG["target_force"], int(force))
            if code != 0:
                return code
        return self._set_action(1)

    def follow_control(self, speed: int, position: int) -> int:
        return self._unsupported("follow_control")

    # ---------- 维护指令 ----------
    def calibrate_fingertip(self, value: int = 1) -> int:
        return self._unsupported("calibrate_fingertip")

    def clear_fingertip_zero(self, value: int = 1) -> int:
        return self._unsupported("clear_fingertip_zero")

    # ---------- 运行参数接口 ----------
    def set_adaptive_speed(self, speed: int) -> int:
        """设自适应夹取速度(200~1500)：写 AO speed 通道。"""
        return self._set_speed(speed)

    def set_target_force(self, force: int) -> int:
        """目标力(0~100%)：写 AO target_force 通道。"""
        return self._write_reg(REG["target_force"], int(force))

    def set_control_mode(self, mode: int) -> int:
        """控制模式 0=自适应 1=力位。"""
        return self._write_reg(REG["control_mode"], int(mode))

    def set_force_position_speed(self, speed: int) -> int:
        return self._write_reg(REG["force_pos_speed"], int(speed))

    def set_force_position_target(self, position: int) -> int:
        return self._write_reg(REG["force_pos_target"], int(position))

    def _unsupported(self, name: str) -> int:
        self._warn_once(
            f"unsupported-{name}",
            f"[gripper] {name} 需要额外的扩展 IO 通道（当前从站未配置），返回 -1",
        )
        return -1

    # ---------- 读接口 ----------
    def _read_force(self, addr: int):
        """三维力寄存器按有符号 16 位解释。"""
        raw = self._read_reg(addr)
        if raw is None:
            return None
        return raw - 0x10000 if raw >= 0x8000 else raw

    def read_force_fx(self): return self._read_force(REG["force_fx"])
    def read_force_fy(self): return self._read_force(REG["force_fy"])
    def read_force_fz(self): return self._read_force(REG["force_fz"])

    def read_state(self):            return self._read_reg(REG["state"])
    def read_hardness(self):         return self._read_reg(REG["hardness"])
    def read_position(self):         return self._read_reg(REG["position"])
    def read_current(self):          return self._read_reg(REG["current"])
    def read_adaptive_speed(self):   return self._read_reg(REG["adaptive_speed"])
    def read_action(self):           return self._read_reg(REG["action"])

    def read_protocol(self):         return self._read_reg(REG["protocol"])
    def read_device_id(self):        return self._read_reg(REG["device_id"])
    def read_control_mode(self):     return self._read_reg(REG["control_mode"])
    def read_force_position_speed(self):  return self._read_reg(REG["force_pos_speed"])
    def read_force_position_target(self): return self._read_reg(REG["force_pos_target"])
    def read_target_force(self):     return self._read_reg(REG["target_force"])
    def read_open_position(self):    return self._read_reg(REG["open_position"])
    def read_close_position(self):   return self._read_reg(REG["close_position"])
    def read_force_position_state(self): return self._read_reg(REG["force_pos_state"])
    def read_eeprom_enabled(self):   return self._read_reg(REG["eeprom_enabled"])
    def read_error_flags(self):      return self._read_reg(REG["error_flags"])

    def read_grip_count(self):
        lo = self._read_ai(EXTIO_AI["grip_count_lo"])
        hi = self._read_ai(EXTIO_AI["grip_count_hi"])
        if lo is None or hi is None:
            return None
        return (int(hi) << 16) | int(lo)

    def read_baudrate(self):
        return None

    def read_factory_number(self):
        return None

    # ---------- 便利接口 ----------
    def opening_m(self, stroke_m: float = 0.098):
        """开口宽度（米）。位置是闭合量，故 opening = (1000 − pos)/1000 × 行程。"""
        pos = self.read_position()
        if pos is None:
            return None
        if pos <= 1000:
            return max(0.0, min(1.0, (1000 - pos) / 1000.0)) * stroke_m
        return None
