#!/usr/bin/env python3
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(1)

def test_wheel(name, m1, m2, m3, m4):
    cmd = f"$pwm:{m1},{m2},{m3},{m4}#"
    print(f"\n=== Testing {name}: PWM({m1}, {m2}, {m3}, {m4}) ===")
    ser.write(cmd.encode())
    time.sleep(2)  # 转2秒
    
    # 停止
    ser.write(b"$pwm:0,0,0,0#")
    time.sleep(1)

input("按 Enter 开始测试...")

# 逐个测试每个轮子正转
test_wheel("M1 正转", 1500, 0, 0, 0)
test_wheel("M2 正转", 0, 1500, 0, 0)
test_wheel("M3 正转", 0, 0, 1500, 0)
test_wheel("M4 正转", 0, 0, 0, 1500)

# 逐个测试每个轮子反转
test_wheel("M1 反转", -1500, 0, 0, 0)
test_wheel("M2 反转", 0, -1500, 0, 0)
test_wheel("M3 反转", 0, 0, -1500, 0)
test_wheel("M4 反转", 0, 0, 0, -1500)

print("\n测试完成！")
ser.close()
