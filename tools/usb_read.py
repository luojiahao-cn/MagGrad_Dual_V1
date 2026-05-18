#!/usr/bin/env python3
"""USB CDC 串口读取和频率分析工具 (使用 pyserial)"""

import serial
import serial.tools.list_ports
import time
import re
import sys

def find_port():
    """查找 USB CDC 端口"""
    ports = list(serial.tools.list_ports.comports())
    print("可用串口:")
    for p in ports:
        print(f"  {p.device}: {p.description}")

    # 优先选择 USB 相关的端口
    for p in ports:
        if 'usb' in p.device.lower() or 'modem' in p.device.lower():
            return p.device

    if ports:
        return ports[0].device
    return None

def main():
    port = find_port()
    if not port:
        print("未找到可用串口")
        sys.exit(1)

    print(f"\n连接端口: {port} @ 115200")

    try:
        ser = serial.Serial(port, 115200, timeout=1)
        ser.flushInput()

        print("读取 5 秒数据...\n")

        lines = []
        start = time.time()
        last_print = start

        while time.time() - start < 5:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                lines.append(data)

            while ser.in_waiting:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    lines.append(data)

            # 每秒打印进度
            if time.time() - last_print >= 1:
                print(f"  已接收 {len(lines)} 行...")
                last_print = time.time()

        ser.close()

    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)

    # 分析数据
    print("\n" + "=" * 60)
    print("分析结果")
    print("=" * 60)

    if not lines:
        print("未接收到数据")
        return

    print(f"总行数: {len(lines)}")

    # 支持的传感器格式:
    # AK:   AK,bus,mask,hx,hy,hz,status,err,dor
    # TMAG: TMAG,ch,addr,x,y,z,status,flags
    # ICM:  ICM,gx,gy,gz,ax,ay,az,temp

    patterns = {
        'AK': re.compile(r'^AK,(\d+),0x([0-9a-fA-F]+),(-?\d+),(-?\d+),(-?\d+),\d+,\d+,\d+$'),
        'TMAG': re.compile(r'^TMAG,(\d+),0x([0-9a-fA-F]+),(-?\d+),(-?\d+),(-?\d+),\d+,\d+$'),
        'ICM': re.compile(r'^ICM,-?\d+,-?\d+,-?\d+,-?\d+,-?\d+,-?\d+,-?\d+$'),
    }

    # 分类数据
    sensor_lines = {k: [] for k in patterns}
    sensor_counts = {k: 0 for k in patterns}

    for line in lines:
        for name, pattern in patterns.items():
            if pattern.match(line):
                sensor_lines[name].append(line)
                sensor_counts[name] += 1
                break

    # 输出各传感器统计
    for name, count in sensor_counts.items():
        if count > 0:
            print(f"{name} 数据行: {count}")

    # AK 传感器详细分析
    if sensor_lines['AK']:
        print(f"\n--- AK 传感器 ---")
        print(f"前 3 行:")
        for line in sensor_lines['AK'][:3]:
            print(f"  {line}")

        ak_sensors = {}
        for line in sensor_lines['AK']:
            m = patterns['AK'].match(line)
            if m:
                key = f"Bus{m.group(1)}_Mask{m.group(2)}"
                ak_sensors[key] = ak_sensors.get(key, 0) + 1

        print(f"\n各 AK 传感器读取次数:")
        for k, v in sorted(ak_sensors.items()):
            print(f"  {k}: {v}")

    # TMAG 传感器详细分析
    if sensor_lines['TMAG']:
        print(f"\n--- TMAG 传感器 ---")
        print(f"前 3 行:")
        for line in sensor_lines['TMAG'][:3]:
            print(f"  {line}")

        tmag_sensors = {}
        for line in sensor_lines['TMAG']:
            m = patterns['TMAG'].match(line)
            if m:
                key = f"CH{m.group(1)}_{m.group(2)}"
                tmag_sensors[key] = tmag_sensors.get(key, 0) + 1

        print(f"\n各 TMAG 传感器读取次数:")
        for k, v in sorted(tmag_sensors.items()):
            print(f"  {k}: {v}")

    # ICM 传感器详细分析
    if sensor_lines['ICM']:
        print(f"\n--- ICM 传感器 ---")
        print(f"前 3 行:")
        for line in sensor_lines['ICM'][:3]:
            print(f"  {line}")
        print(f"ICM 读取次数: {sensor_counts['ICM']}")

    # 计算帧率
    elapsed = time.time() - start
    print(f"\n--- 帧率统计 ---")
    print(f"总读取: {len(lines)} 行")
    print(f"耗时: {elapsed:.2f} 秒")

    # 按传感器计算帧率
    for name, lines_list in sensor_lines.items():
        if lines_list:
            sensor_freq = len(lines_list) / elapsed
            print(f"\n--- {name} ---")
            print(f"读取频率: {sensor_freq:.1f} 读数/秒")

            # 估算帧率: 总读数 / 每帧传感器数 / 耗时
            unique_sensors = set()
            pattern = patterns[name]
            for line in lines_list:
                m = pattern.match(line)
                if m:
                    if name == 'AK':
                        unique_sensors.add(f"Bus{m.group(1)}_Mask{m.group(2)}")
                    elif name == 'TMAG':
                        unique_sensors.add(f"CH{m.group(1)}_{m.group(2)}")
                    elif name == 'ICM':
                        unique_sensors.add(name)  # 只有一个 ICM

            num_sensors = len(unique_sensors)
            if num_sensors > 0:
                frames = len(lines_list) / num_sensors
                frame_rate = frames / elapsed
                print(f"传感器数: {num_sensors}")
                print(f"帧率: {frame_rate:.2f} Hz")

    print("=" * 60)

if __name__ == '__main__':
    main()
