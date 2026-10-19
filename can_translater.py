      
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN报文实时解析工具 - 博雷顿自动驾驶协议
解析: 挡位 | 手刹 | 油门踏板 | 刹车踏板 | 驾驶模式
支持: candump vcan0 | python3 can_data_handle.py
支持: candump -t A vcan0 | python3 can_data_handle.py (带时间戳)
"""

import sys
import re
from datetime import datetime
from typing import Dict, Tuple, Optional


# ==================== 信号定义类 ====================
class SignalDef:
    def __init__(self, name, desc, start_byte, start_bit, bit_length,
                 resolution, offset, unit, value_map=None):
        self.name = name
        self.desc = desc
        self.start_byte = start_byte
        self.start_bit = start_bit
        self.bit_length = bit_length
        self.resolution = resolution
        self.offset = offset
        self.unit = unit
        self.value_map = value_map or {}

    def parse(self, data: bytes) -> Tuple[float, str]:
        start_pos = self.start_byte * 8 + self.start_bit
        value = 0
        for i in range(self.bit_length):
            byte_idx = (start_pos + i) // 8
            bit_idx = (start_pos + i) % 8
            if byte_idx < len(data):
                if data[byte_idx] & (1 << bit_idx):
                    value |= (1 << i)

        phys_value = value * self.resolution + self.offset

        if value in self.value_map:
            return phys_value, self.value_map[value]
        elif self.unit:
            return phys_value, f"{phys_value:.1f}{self.unit}"
        else:
            return phys_value, f"{phys_value}"


# ==================== 报文定义 ====================
CAN_MESSAGES = {
    # ========== 下行: ADU → VCU ==========
    '181E1F39': {
        'name': 'ADU2VCU1',
        'desc': 'ADU→VCU 驱动控制',
        'signals': [
            SignalDef('DriverMode', '驾驶模式', 7, 0, 4, 1, 0, '', {
                0: '人工驾驶', 1: '待机', 2: '远程驾驶', 3: '无人驾驶'
            }),
            SignalDef('TargetGearAuto', '目标档位(自动档)', 0, 6, 2, 1, 0, '', {
                0: 'N档', 1: 'D档', 2: 'R档'
            }),
            SignalDef('TargetGearManual', '目标档位(手动档)', 1, 0, 4, 1, -2, '', {
                -2: '倒2档', -1: '倒1档', 0: '空档', 1: '1档', 2: '2档',
                3: '3档', 4: '4档', 5: '5档', 6: '6档', 7: '7档',
                8: '8档', 9: '9档', 10: '10档', 11: '11档', 12: '12档'
            }),
            SignalDef('TargetAccPedal', '目标油门', 5, 0, 8, 0.4, 0, '%'),
            SignalDef('TargetBrkPedal', '目标刹车', 6, 0, 8, 0.4, 0, '%'),
            SignalDef('ADUModeSts', 'ADU状态', 0, 0, 3, 1, 0, '', {
                0: '正常', 1: '轻微故障', 2: '严重故障', 3: '致命故障'
            }),
        ]
    },

    '182E1F39': {
        'name': 'ADU2VCU2',
        'desc': 'ADU→VCU 转向驻车',
        'signals': [
            SignalDef('ParkBrakeCmd', '驻车制动指令', 3, 0, 8, 0.4, 0, '%', {
                0: '解除驻车', 251: '最大驻车'
            }),
            SignalDef('EmcyBarkeCmd', '紧急制动指令', 4, 0, 1, 1, 0, '', {
                0: '解除', 1: '紧急制动'
            }),
        ]
    },

    # ========== 上行: VCU → ADU ==========
    '18D0FF00': {
        'name': 'VCU2ADU1',
        'desc': 'VCU→ADU 整车反馈',
        'signals': [
            SignalDef('DriveModeStus', '驾驶模式', 0, 2, 2, 1, 0, '', {
                0: '人工驾驶', 1: '待机', 2: '远程驾驶', 3: '无人驾驶'
            }),
            SignalDef('ShiftLever', '换挡手柄', 1, 0, 4, 1, 0, '', {
                0: 'N档', 1: 'R档', 2: 'D档', 3: '无效'
            }),
            SignalDef('CurrentGear', '当前档位', 1, 4, 4, 1, -2, '', {
                -2: '倒2档', -1: '倒1档', 0: '空档', 1: '1档', 2: '2档',
                3: '3档', 4: '4档', 5: '5档', 6: '6档', 7: '7档',
                8: '8档', 9: '9档', 10: '10档', 11: '11档', 12: '12档'
            }),
            SignalDef('BrakePedalPosition', '刹车踏板', 2, 0, 8, 0.4, 0, '%'),
            SignalDef('AccPedalPosition', '油门踏板', 3, 0, 8, 0.4, 0, '%'),
            SignalDef('ReadyStus', 'Ready状态', 0, 0, 2, 1, 0, '', {
                0: '初始化', 1: 'Ready', 2: '低压状态', 3: '高压未Ready'
            }),
            SignalDef('VehicleSpeed', '车速', 5, 0, 16, 1/256, 0, 'km/h'),
        ]
    },

    '18D1FF00': {
        'name': 'VCU2ADU2',
        'desc': 'VCU→ADU 驻车反馈',
        'signals': [
            SignalDef('ParkBrakeSts', '驻车状态', 5, 8, 2, 1, 0, '', {
                0: '无效', 1: '驻车', 2: '解除驻车', 3: '错误'
            }),
            SignalDef('EmcyBarkeSts', '紧急制动状态', 7, 0, 2, 1, 0, '', {
                0: '解除', 1: '启动'
            }),
        ]
    },

    '18D6FF00': {
        'name': 'VCU2ADU7',
        'desc': 'VCU→ADU 虚拟油门刹车',
        'signals': [
            SignalDef('AccPedalPosition', '虚拟油门', 3, 0, 8, 0.4, 0, '%'),
            SignalDef('BppPedalPosition', '虚拟刹车', 4, 0, 8, 0.4, 0, '%'),
        ]
    },
}


def parse_can_line(line: str) -> Optional[Tuple[str, bytes, str]]:
    """
    解析candump输出的一行
    支持带时间戳和不带时间戳两种格式
    返回: (can_id, data_bytes, timestamp) 或 None
    """
    line = line.strip()
    if not line:
        return None

    timestamp = ""
    can_part = line

    # 检测是否带时间戳: 格式 (2026-07-16 10:52:32.773997)
    time_pattern = r'^\([0-9]{4}-[0-9]{2}-[0-9]{2}\s+[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]+\)\s*'
    match_time = re.match(time_pattern, line)
    if match_time:
        timestamp = match_time.group().strip()
        can_part = line[match_time.end():]

    # 解析CAN数据部分: vcan0 181E1F39 [8] 00 05 71 18 7D 00 00 D3
    pattern = r'^(\S+)\s+([0-9A-Fa-f]{8})\s+\[([0-9]{1,2})\]\s+((?:[0-9A-Fa-f]{2}\s*)+)$'
    match = re.match(pattern, can_part.strip())
    if not match:
        return None

    can_id = match.group(2).upper()
    data_str = re.sub(r'\s+', ' ', match.group(4).strip())

    try:
        data_bytes = bytes.fromhex(data_str)
        return can_id, data_bytes, timestamp
    except ValueError:
        return None


def parse_message(can_id: str, data: bytes) -> Dict:
    result = {
        'can_id': can_id,
        'data': ' '.join(f'{b:02X}' for b in data),
        'name': '未知报文',
        'desc': '',
        'signals': {}
    }

    if can_id in CAN_MESSAGES:
        msg_def = CAN_MESSAGES[can_id]
        result['name'] = msg_def['name']
        result['desc'] = msg_def['desc']

        for sig in msg_def['signals']:
            try:
                _, display = sig.parse(data)
                result['signals'][sig.name] = {
                    'desc': sig.desc,
                    'display': display
                }
            except Exception:
                result['signals'][sig.name] = {
                    'desc': sig.desc,
                    'display': '解析错误'
                }

    return result


def format_compact(result: Dict, timestamp: str = "") -> str:
    """单条报文显示，支持时间戳"""
    if not result['signals']:
        msg = f"[{result['can_id']}] 未知报文"
        return f"{timestamp} {msg}" if timestamp else msg

    parts = []
    for name, info in result['signals'].items():
        skip = ['生命信号', '档位模式', 'ADU状态', '换挡手柄', '档位模式状态', '整车故障等级']
        if info['desc'] in skip:
            continue
        parts.append(f"{info['desc']}:{info['display']}")

    if parts:
        msg = f"{result['can_id']} | {' | '.join(parts)}"
    else:
        msg = f"[{result['can_id']}] 其他信号"

    return f"{timestamp} {msg}" if timestamp else msg


def main():
    is_pipe = not sys.stdin.isatty()

    if is_pipe:
        # 管道模式：实时解析
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue

            parsed_line = parse_can_line(line)
            if parsed_line:
                can_id, data, timestamp = parsed_line
                result = parse_message(can_id, data)
                print(format_compact(result, timestamp), flush=True)
            else:
                print(f"无法解析: {line[:80]}", flush=True)

    else:
        # 交互模式：手动粘贴
        print("=" * 60)
        print("CAN报文解析工具 - 博雷顿自动驾驶协议")
        print("解析: 挡位 | 手刹 | 油门 | 刹车 | 驾驶模式")
        print("=" * 60)
        print("请粘贴CAN数据 (Ctrl+D结束):")

        lines = []
        try:
            while True:
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue
                if line.lower() in ('quit', 'exit'):
                    return
                lines.append(line)
        except KeyboardInterrupt:
            print("\n退出")
            return

        if not lines:
            print("没有数据")
            return

        print("\n解析结果:")
        print("-" * 60)

        for line in lines:
            parsed_line = parse_can_line(line)
            if parsed_line:
                can_id, data, timestamp = parsed_line
                result = parse_message(can_id, data)
                print(format_compact(result, timestamp))
            else:
                print(f"无法解析: {line[:80]}")


if __name__ == "__main__":
    main()

    
