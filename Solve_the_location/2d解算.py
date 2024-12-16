import serial
import struct
import numpy as np
from scipy.optimize import curve_fit
import time
import matplotlib.pyplot as plt
from matplotlib import font_manager

FRAME_HEADER = b'\xff\xaa'
FRAME_TAIL = b'\x00\x00\x80\x7F\x00\x00\x00\x0A'
ANCHOR_NUM=8
fmt = f"2s2s{ANCHOR_NUM}f{ANCHOR_NUM}f3f8B8s"
expected_length = struct.calcsize(fmt)
# 基站坐标
ANCHOR_POSITIONS = {
    'd0': (0.0, 0.0),  # 基站0的坐标 (x0, y0)
    'd1': (0.0, 0.6),  # 基站1的坐标 (x1, y1)
    'd2': (4.2, 1.2),  # 基站2的坐标 (x2, y2)
    'd3': (4.2, 0.0)   # 基站3的坐标 (x3, y3)
}

def parse_frame(data):
    """从接收到的数据中解析有效帧，并提取多个浮点数值和基站状态"""
    start_index = data.find(FRAME_HEADER)
    end_index = data.find(FRAME_TAIL, start_index)
    if start_index != -1 and end_index != -1:
        frame = data[start_index:end_index + len(FRAME_TAIL)]
        # print(f"完整帧内容: {frame.hex()}")  # 打印完整帧的十六进制
        unpacked = struct.unpack(fmt, frame)# 提取所有浮点数字节和基站状态
        frame = {
            "header": unpacked[0],
            "command": unpacked[1],
            "original_distances": unpacked[2:2 + ANCHOR_NUM],
            "calibrated_distances": unpacked[2 + ANCHOR_NUM:2 + 2 * ANCHOR_NUM],
            "position": {
                "x": unpacked[2 + 2 * ANCHOR_NUM],
                "y": unpacked[2 + 2 * ANCHOR_NUM + 1],
                "z": unpacked[2 + 2 * ANCHOR_NUM + 2],
            },
            "is_active": unpacked[2 + 2 * ANCHOR_NUM + 3:2 + 2 * ANCHOR_NUM + 11],
            "tail": unpacked[-1],
        }

        # 使用有效的基站状态
        distances_with_ids = [
            (i, dist) for i, (dist, active) in enumerate(
                zip(frame['calibrated_distances'], frame['is_active'])
            ) if active == 1
        ]
    
        print("活动基站和距离值:")
        for anchor_id, distance in distances_with_ids:
            print(f"d{anchor_id}距离: {distance:.3f}m (基站坐标: {ANCHOR_POSITIONS[f'd{anchor_id}']})")
        
        return distances_with_ids
    return None

def calculate_position_with_valid_anchors(distances_with_ids):
    """使用有效基站数据计算位置"""
    if len(distances_with_ids) < 3:
        print("有效基站数量不足，无法计算位置")
        return None
        
    try:
        # 构建方程组只使用有效基站的数据
        A = []
        b = []
        first_anchor = True
        x1, y1 = None, None
        d1 = None
        
        for anchor_id, distance in distances_with_ids:
            x, y = ANCHOR_POSITIONS[f'd{anchor_id}']
            if first_anchor:
                x1, y1 = x, y
                d1 = distance
                first_anchor = False
                continue          
            A.append([2*(x-x1), 2*(y-y1)])
            b.append([d1**2 - distance**2 - (x1**2 + y1**2) + (x**2 + y**2)])
            d1=distance
            x1=x
            y1=y
        
        # 转换为numpy数组
        A = np.array(A)
        b = np.array(b)
        
        # 求解方程组
        solution = np.linalg.lstsq(A, b, rcond=None)[0]
        x = solution[0][0]
        y = solution[1][0]
        
        return x, y
    except Exception as e:
        print(f"位置计算错误: {e}")
        return None

def plot_position(current_position, anchors, active_anchors):
    """绘制当前标签位置和基站位置，标注活动基站"""
    plt.clf()
    plt.figure(1)
    
    # 设置网格
    plt.grid(True, which='major', linestyle='-', linewidth='0.5')
    plt.minorticks_on()
    plt.grid(True, which='minor', linestyle=':', linewidth='0.5', alpha=0.5)
    
    # 设置刻度
    x_ticks = np.arange(0, 4.8, 0.6)
    y_ticks = np.arange(0, 1.8, 0.6)
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    
    # 绘制基站位置
    for i, (anchor_name, pos) in enumerate(anchors.items()):
        # 根据基站状态选择颜色：蓝色表示活动，灰色表示非活动
        color = 'blue' if active_anchors[i] == 1 else 'gray'
        plt.plot(pos[0], pos[1], 's', color=color, markersize=10)
        plt.annotate(anchor_name, (pos[0], pos[1]), xytext=(5, 5), 
                    textcoords='offset points', fontsize=12,
                    color=color)
    
    # 绘制当前位置
    if current_position is not None:
        x, y = current_position
        plt.plot(x, y, 'ro', markersize=12)
        plt.annotate(f'({x:.2f}, {y:.2f})',
                    (x, y), 
                    xytext=(10, 10), 
                    textcoords='offset points',
                    bbox=dict(boxstyle="round,pad=0.3", fc="yellow", ec="b", alpha=0.3))
    
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Current Tag Position (Grid: 0.6m)')
    plt.xlim(-0.3, 4.5)
    plt.ylim(-0.3, 1.5)
    plt.axis('equal')

def main():
    """主函数：读取数据并显示当前位置"""
    serial_port = "COM15"
    baud_rate = 115200
    update_interval = 0.1
    last_update_time = time.time()
    
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
        print("串口连接成功")
        
        plt.ion()
        plt.figure(figsize=(12, 8))
        
        while True:
            # 读取一帧数据
            if ser.in_waiting >= expected_length:
                data = ser.read(expected_length)
                distances_with_ids= parse_frame(data)
                # print("distances_with_ids内容是：",distances_with_ids)
                position = calculate_position_with_valid_anchors(distances_with_ids)
                if position is not None:
                    x, y = position
                    print(f"计算得到的位置: X = {x:.3f}m, Y = {y:.3f}m")
                    
                    current_time = time.time()
                    if current_time - last_update_time >= update_interval:
                        active_anchors = [1 if i in [anchor_id for anchor_id, _ in distances_with_ids] else 0 for i in range(ANCHOR_NUM)]
                        plot_position((x, y), ANCHOR_POSITIONS,  active_anchors)
                        plt.pause(0.01)
                        last_update_time = current_time
            
            if plt.get_fignums():
                if plt.waitforbuttonpress(timeout=0.01):
                    break
            else:
                break
            
    except Exception as e:
        print(f"发生错误: {e}")
    
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\n串口已关闭")
        
        plt.ioff()
        plt.close('all')

if __name__ == "__main__":
    main()