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
    # 'd0': (0.0, 0.0 ,1),  # 基站0的坐标 (x0, y0,z0)
    # 'd1': (0.0, 0.6,2),  # 基站1的坐标 (x1, y1,z1)
    # 'd2': (4.2, 1.2,3),  # 基站2的坐标 (x2, y2,z2)
    # 'd3': (4.2, 0.0,4)   # 基站3的坐标 (x3, y3,z3)
    'd0': (0.0, 0.0 ,0.0),  # 基站0的坐标 (x0, y0,z0)
    'd1': (0.6, 0.6,0.8),  # 基站1的坐标 (x1, y1,z1)
    'd2': (2.4, 0.6,0.4),  # 基站2的坐标 (x2, y2,z2)
    'd3': (3.0, 0.0,0.0)   # 基站3的坐标 (x3, y3,z3)
}

class PositionFilter:
    def __init__(self, window_size=10, weight_type='linear'):
        self.positions = []
        self.window_size = window_size
        self.weight_type = weight_type
        
    def get_weights(self):
        """获取权重系数"""
        n = len(self.positions)
        if n == 0:
            return []
            
        if self.weight_type == 'uniform':  # 均匀权重
            return np.ones(n) / n
            
        elif self.weight_type == 'linear':  # 线性递增权重
            weights = np.arange(1, n + 1)
            return weights / np.sum(weights)
            
        elif self.weight_type == 'exp':     # 指数递增权重
            weights = np.exp(np.arange(n))
            return weights / np.sum(weights)
    
    def update(self, new_position):
        """更新位置估计"""
        if new_position is None:
            return None
        
        # 添加新位置
        self.positions.append(new_position)
        
        # 保持窗口大小
        if len(self.positions) > self.window_size:
            self.positions.pop(0)
            
        # 获取权重
        weights = self.get_weights()
        
        # 加权平均
        x_avg = np.average([p[0] for p in self.positions], weights=weights)
        y_avg = np.average([p[1] for p in self.positions], weights=weights)
        z_avg = np.average([p[2] for p in self.positions], weights=weights)
        
        # 计算标准差（可用于质量评估）
        x_std = np.std([p[0] for p in self.positions])
        y_std = np.std([p[1] for p in self.positions])
        z_std = np.std([p[2] for p in self.positions])
        
        # 可以根据标准差进行异常检测
        is_stable = all(std < 0.1 for std in [x_std, y_std, z_std])
        
        return x_avg, y_avg, z_avg

    def reset(self):
        """重置滤波器"""
        self.positions = []

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
    """使用四个基站的数据计算三维位置"""
    if len(distances_with_ids) < 4:
        print("有效基站数量不足，无法计算三维位置")
        return None
        
    try:
        # 获取基站坐标和距离
        anchor_coords = []
        distances = []
        
        for anchor_id, distance in distances_with_ids:
            anchor_coords.append(ANCHOR_POSITIONS[f'd{anchor_id}'])
            distances.append(distance)
            
        A = []
        b = []
        
        # 使用第一个基站作为参考
        x1, y1, z1 = anchor_coords[0]
        d1 = distances[0]
        
        for i in range(1, len(anchor_coords)):
            xi, yi, zi = anchor_coords[i]
            di = distances[i]
            
            # 构建系数矩阵A
            A.append([
                2*(x1-xi),
                2*(y1-yi),
                2*(z1-zi)
            ])
            
            # 构建常数向量b - 注意这里不要使用嵌套列表
            b.append(
                (di*di - d1*d1 - xi*xi - yi*yi - zi*zi + x1*x1 + y1*y1 + z1*z1)
            )
        
        # 转换为numpy数组并确保正确的形状
        A = np.array(A, dtype=np.float64)  # 明确指定数据类型
        b = np.array(b, dtype=np.float64)  # 明确指定数据类型
        
        # 使用最小二乘求解
        solution = np.linalg.lstsq(A, b, rcond=None)[0]
        
        # 确保结果是标量
        x = float(solution[0])
        y = float(solution[1])
        z = float(solution[2])
        
        return x, y, z
        
    except Exception as e:
        print(f"位置计算错误: {str(e)}")  # 使用str()确保错误信息可以正确打印
        return None

def plot_position_3d(current_position, anchors, active_anchors):
    """绘制三维空间中的当前标签位置和基站位置"""
    plt.clf()
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制基站位置和连线
    for i, (anchor_name, pos) in enumerate(anchors.items()):
        color = 'blue' if active_anchors[i] == 1 else 'gray'
        ax.scatter(pos[0], pos[1], pos[2], c=color, marker='s', s=100)
        ax.text(pos[0], pos[1], pos[2], f'{anchor_name}\n({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})',
                color=color, fontsize=8)
        
        # 绘制到原点的投影线
        if active_anchors[i] == 1:
            ax.plot([pos[0], pos[0]], [pos[1], pos[1]], [0, pos[2]], 
                   '--', color='lightgray', alpha=0.5)
    
    # 绘制当前位置
    if current_position is not None:
        x, y, z = current_position
        ax.scatter(x, y, z, c='red', marker='o', s=100)
        ax.text(x, y, z, f'Tag\n({x:.2f}, {y:.2f}, {z:.2f})',
                color='red', fontsize=8,
                bbox=dict(facecolor='yellow', alpha=0.3))
        
        # 绘制标签位置的投影
        ax.plot([x, x], [y, y], [0, z], '--', color='red', alpha=0.5)
    
    # 绘制地面网格
    xx, yy = np.meshgrid(np.linspace(0, 4.2, 5), np.linspace(0, 1.2, 3))
    ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.1, color='gray')
    
    # 设置坐标轴
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('3D Tag Position with 4 Anchors')
    
    # 设置坐标轴范围
    ax.set_xlim(-0.3, 4.5)
    ax.set_ylim(-0.3, 1.5)
    ax.set_zlim(0, 2.0)
    
    # 添加网格
    ax.grid(True)
    
    # 设置最佳观察角度
    ax.view_init(elev=20, azim=45)
    
    # 保持坐标轴比例一致
    ax.set_box_aspect([4.8, 1.8, 2.0])

def main():
    """主函数：读取数据并显示当前位置""" 
    serial_port = "COM15"
    baud_rate = 115200
    update_interval = 0.1
    last_update_time = time.time()
    position_filter = PositionFilter()
    
    try:
        # ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
        # print("串口连接成功")
        # print("基站布局：")
        for anchor_name, pos in ANCHOR_POSITIONS.items():
            print(f"{anchor_name}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        
        plt.ion()
        plt.figure(figsize=(12, 8))
        # ser.reset_input_buffer()
        while True:
            # if ser.in_waiting >= expected_length:
                # data = ser.read(expected_length)
            distances_with_ids = [(0, 1.897367), (1, 1.442221), (2,0.721110),(3,1.341641)]
            position = calculate_position_with_valid_anchors(distances_with_ids)
            
            if position is not None:
                filtered_position = position_filter.update(position)
                if filtered_position is not None:
                    x, y, z = filtered_position
                    print(f"计算得到的位置: X = {x:.4f}m, Y = {y:.4f}m, Z = {z:.4f}m")
                
                current_time = time.time()
                if current_time - last_update_time >= update_interval:
                    active_anchors = [1 if i in [anchor_id for anchor_id, _ in distances_with_ids] else 0 
                                    for i in range(ANCHOR_NUM)]
                    plot_position_3d((x, y, z), ANCHOR_POSITIONS, active_anchors)
                    plt.pause(0.01)
                    last_update_time = current_time
            
            # if plt.get_fignums():
            #     if plt.waitforbuttonpress(timeout=0.01):
            #         break
            # else:
            #     break
            
    except Exception as e:
        print(f"发生错误: {e}")
    
    finally:
        # if 'ser' in locals() and ser.is_open:
            # ser.close()
            # print("\n串口已关闭")
        
        plt.ioff()
        plt.close('all')

if __name__ == "__main__":
    main()