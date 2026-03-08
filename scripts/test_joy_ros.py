#!/usr/bin/env python3
"""
ROS2 joy 话题按键测试工具
用途：打印每次按键时 buttons[] 的哪个 index 变为 1，
      用来确认 rl_sim.cpp JoyCallback 里的 buttons[N] 映射是否正确。
用法：python3 scripts/test_joy_ros.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# rl_sim.cpp 中当前的映射（Xbox Wireless Controller 布局）
EXPECTED = {
    0: "A   → GetUp",
    1: "B   → GetDown",
    3: "X   → Passive / NavigationMode",
    4: "Y   → RL模式",
    6: "LB  → 组合修饰",
    7: "RB  → 组合修饰",
}

class JoyTester(Node):
    def __init__(self):
        super().__init__('joy_tester')
        self.prev_buttons = []
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        print("\033[1;36m╔══════════════════════════════════════════════════╗\033[0m")
        print("\033[1;36m║     ROS2 Joy 按键测试（rl_sim 仿真手柄映射）      ║\033[0m")
        print("\033[1;36m╚══════════════════════════════════════════════════╝\033[0m")
        print("\033[1m当前 rl_sim.cpp 期望映射（F710 布局）：\033[0m")
        for idx, desc in EXPECTED.items():
            print(f"  buttons[{idx}] = {desc}")
        print("\n\033[33m请依次按下手柄各按键，观察哪个 index 变为 1...\033[0m\n")
        print("-" * 55)

    def cb(self, msg):
        btns = list(msg.buttons)
        if btns == self.prev_buttons:
            return

        # 找到发生变化的 index
        changed = []
        for i in range(max(len(btns), len(self.prev_buttons))):
            prev = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
            curr = btns[i] if i < len(btns) else 0
            if curr != prev:
                changed.append((i, prev, curr))

        for idx, prev, curr in changed:
            state = "\033[32m按下\033[0m" if curr else "\033[31m松开\033[0m"
            expected = EXPECTED.get(idx, "(rl_sim 中未使用)")
            match = "✅" if idx in EXPECTED and curr else ("" if not curr else "⚠️ 未在rl_sim中映射")
            print(f"  buttons[{idx:2d}]  {state}  →  期望: {expected}  {match}")

        self.prev_buttons = btns

def main():
    rclpy.init()
    node = JoyTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\n测试结束。")

if __name__ == '__main__':
    main()
