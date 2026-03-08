#!/usr/bin/env python3
"""
Joystick 按键测试工具
用途：帮助确认手柄各按键的 id 编号，以及对应的 FSM 指令是否正确。
用法：python3 scripts/test_joystick.py [设备路径，默认 /dev/input/js0]
"""

import sys
import os
import struct
import fcntl
import time
from collections import defaultdict

# ── 颜色输出 ──────────────────────────────────────────────────────────────────
R  = "\033[31m"
G  = "\033[32m"
Y  = "\033[33m"
B  = "\033[34m"
C  = "\033[36m"
W  = "\033[37m"
BO = "\033[1m"
RS = "\033[0m"

# ── FSM 期望的映射（与 fsm_qmini.hpp / readJoystick 保持一致）────────────────
# 格式：button id → (Gamepad 枚举名, FSM 功能说明)
BUTTON_MAP = {
    0:  ("A",       "站起 GetUp"),
    1:  ("B",       "趴下 GetDown"),
    3:  ("X",       "Passive 模式"),
    4:  ("Y",       "RL 运动模式"),
    6:  ("LB/L1",   "组合键修饰符"),
    7:  ("RB/R1",   "组合键修饰符"),
    10: ("LStick",  "SELECT"),
    11: ("RStick",  "START"),
}

# 轴 id → 说明
AXIS_MAP = {
    0: ("LX",    "左摇杆 X → 横向移动 y"),
    1: ("LY",    "左摇杆 Y → 前后移动 x（推上为正）"),
    2: ("RX",    "右摇杆 X → Yaw 旋转"),
    3: ("RY",    "右摇杆 Y → （未使用）"),
    4: ("L2",    "左扳机"),
    5: ("R2",    "右扳机"),
    6: ("DPadX", "方向键 X：右>0 左<0"),
    7: ("DPadY", "方向键 Y：下>0 上<0 → RB+上=RL模式"),
}

# 组合键逻辑（与 readJoystick 一致）
COMBO_BUTTON = {
    # (modifier_id, face_id) → 枚举名
    (6, 0): "LB_A",
    (6, 1): "LB_B",
    (6, 3): "LB_X",
    (6, 4): "LB_Y",
    (7, 0): "RB_A",
    (7, 1): "RB_B",
    (7, 3): "RB_X",
    (7, 4): "RB_Y",
}

DPAD_COMBOS = {
    # (modifier_id, axis_id, sign) → 枚举名
    (7, 7, -1): ("RB_DPadUp",    "RL 运动模式（备选）"),
    (7, 7, +1): ("RB_DPadDown",  "技能2"),
    (7, 6, +1): ("RB_DPadRight", "技能"),
    (7, 6, -1): ("RB_DPadLeft",  "技能"),
    (6, 7, -1): ("LB_DPadUp",    "技能"),
    (6, 7, +1): ("LB_DPadDown",  "技能"),
    (6, 6, +1): ("LB_DPadRight", "技能"),
    (6, 6, -1): ("LB_DPadLeft",  "技能"),
}

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x80
JS_EVENT_FMT    = "IhBB"   # time(u32), value(s16), type(u8), number(u8)
JS_EVENT_SIZE   = struct.calcsize(JS_EVENT_FMT)

def get_js_name(fd):
    try:
        buf = b'\x00' * 128
        import ctypes
        buf = ctypes.create_string_buffer(128)
        JSIOCGNAME = 0x80006a13 | (128 << 16)
        fcntl.ioctl(fd, JSIOCGNAME, buf)
        return buf.value.decode(errors='replace').strip('\x00')
    except Exception:
        return "未知设备"

def fmt_axis(val_raw):
    v = val_raw / 32767.0
    bar_len = 20
    pos = int((v + 1.0) / 2.0 * bar_len)
    bar = "[" + "#" * pos + "-" * (bar_len - pos) + "]"
    return f"{v:+.3f} {bar}"

def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/input/js0"

    if not os.path.exists(dev):
        print(f"{R}错误：找不到设备 {dev}{RS}")
        print("可用设备：")
        for f in sorted(os.listdir("/dev/input")):
            if f.startswith("js"):
                print(f"  /dev/input/{f}")
        sys.exit(1)

    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    name = get_js_name(fd)

    print(f"\n{BO}{C}╔══════════════════════════════════════════════════╗{RS}")
    print(f"{BO}{C}║        Qmini 手柄按键测试工具                    ║{RS}")
    print(f"{BO}{C}╚══════════════════════════════════════════════════╝{RS}")
    print(f"  设备：{B}{dev}{RS}  ({Y}{name}{RS})")
    print(f"\n{BO}期望的按键映射（基于 readJoystick 代码）：{RS}")
    print(f"  {G}A(id=0){RS}=站起  {R}B(id=1){RS}=趴下  {Y}X(id=3){RS}=Passive  {C}Y(id=4){RS}=RL模式")
    print(f"  {W}L1(id=6){RS}=组合修饰  {W}R1(id=7){RS}=组合修饰  R1+↑=RL模式备选\n")
    print(f"{Y}按下手柄按键，观察下方输出。Ctrl+C 退出。{RS}\n")
    print("-" * 60)

    btns = defaultdict(bool)
    axes = defaultdict(float)
    event_count = 0

    try:
        while True:
            try:
                data = os.read(fd, JS_EVENT_SIZE)
            except BlockingIOError:
                time.sleep(0.01)
                continue

            if len(data) < JS_EVENT_SIZE:
                continue

            ts, value, etype, number = struct.unpack(JS_EVENT_FMT, data)

            # 跳过初始化事件
            if etype & JS_EVENT_INIT:
                continue

            event_count += 1

            if etype & JS_EVENT_BUTTON:
                pressed = value != 0
                btns[number] = pressed
                state_str = f"{G}按下{RS}" if pressed else f"{R}松开{RS}"

                # 查基础映射
                info = BUTTON_MAP.get(number)
                if info:
                    enum_name, fsm_desc = info
                    print(f"  BUTTON  id={BO}{number:2d}{RS}  {state_str}  →  "
                          f"Gamepad::{C}{enum_name}{RS}  [{fsm_desc}]")
                else:
                    print(f"  BUTTON  id={BO}{number:2d}{RS}  {state_str}  →  "
                          f"{Y}(未映射，当前代码中无此按键){RS}")

                # 检测组合键
                if pressed:
                    for (mod, face), combo in COMBO_BUTTON.items():
                        if number == face and btns[mod]:
                            print(f"    {Y}↳ 组合：L1/R1(id={mod})+id={face} → Gamepad::{combo}{RS}")
                        if number == mod and btns[face]:
                            print(f"    {Y}↳ 组合：L1/R1(id={mod})+id={face} → Gamepad::{combo}{RS}")

            elif etype & JS_EVENT_AXIS:
                v = value / 32767.0
                axes[number] = v
                info = AXIS_MAP.get(number, (f"Axis{number}", "未知轴"))
                aname, adesc = info

                # 只在超过死区时打印
                if abs(v) > 0.15 or number in (6, 7):
                    # DPad 轴检测组合键
                    combos_hit = []
                    if number in (6, 7) and abs(v) > 0.5:
                        sign = +1 if v > 0 else -1
                        for mod in (6, 7):
                            key = (mod, number, sign)
                            if key in DPAD_COMBOS and btns[mod]:
                                cn, cdesc = DPAD_COMBOS[key]
                                combos_hit.append(f"Gamepad::{C}{cn}{RS}[{cdesc}]")

                    combo_str = "  " + " + ".join(combos_hit) if combos_hit else ""
                    print(f"  AXIS    id={BO}{number:2d}{RS}  {B}{aname:8s}{RS}  "
                          f"{fmt_axis(value)}  {W}{adesc}{RS}{combo_str}")

    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)

    print(f"\n{G}共收到 {event_count} 个事件。测试结束。{RS}\n")

    # ── 诊断建议 ──────────────────────────────────────────────────────────────
    print(f"{BO}诊断建议：{RS}")
    print(f"  若上方 id 与期望不符，请修改 {Y}rl_real_qmini.cpp{RS} 中 readJoystick() 的按键 id：")
    print(f"  {W}setBtn(<实际id>, Input::Gamepad::A/B/X/Y/LB/RB ...){RS}")
    print()

if __name__ == "__main__":
    main()
