import serial
import struct
import numpy as np
from scipy.optimize import curve_fit
import time
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号
# 帧头帧尾的定义（全局常量）
FRAME_HEADER = b'\xff\xaa'
FRAME_TAIL = b'\x00\x00\x80\x7F\x00\x00\x00\x0A'
global flag
def send_position(ser,x,y,flag):
        x_bytes = struct.pack('<f',x)
        y_bytes = struct.pack('<f',y)
        if(int(flag)==1):
            frame = b'\xff\xaa\x02' +b'\x00'+ x_bytes + y_bytes + b'\x00\x00\x00\x00' + FRAME_TAIL
        elif(int(flag)==2):
            frame = b'\xff\xaa\x02' +b'\x01'+ x_bytes + y_bytes + b'\x00\x00\x00\x00' + FRAME_TAIL
        elif(int(flag)==3):
            frame = b'\xff\xaa\x02' +b'\x02'+ x_bytes + y_bytes + b'\x00\x00\x00\x00' + FRAME_TAIL
        else:
            frame = b'\xff\xaa\x02' +b'\x03'+ x_bytes + y_bytes + b'\x00\x00\x00\x00' + FRAME_TAIL
        
        ser.write(frame)
        ser.flush()  # 确保数据发送完成
        return True
def send_calibration_params(ser, k, b ,flag):
    """
    发送校准参数k和b到串口
    格式：帧头(2字节) + k(4字节) + b(4字节) + 帧尾(8字节)
    """
    try:
        # 发送帧头帧尾定义
        FRAME_HEADER_command = b'\xff\xaa\x01'
        # 将k和b打包成单精度浮点数（每个4字节）
        k_bytes = struct.pack('<f', k)
        b_bytes = struct.pack('<f', b)
        # 组合完整的数据帧
        # print(flag)
        # frame = FRAME_HEADER_command +b'\x00'+ k_bytes + b_bytes + FRAME_TAIL
        # ser.write(frame)
        # frame = FRAME_HEADER_command +b'\x01'+ k_bytes + b_bytes + FRAME_TAIL
        # ser.write(frame)
        # frame = FRAME_HEADER_command +b'\x02'+ k_bytes + b_bytes + FRAME_TAIL
        # ser.write(frame)
        # frame = FRAME_HEADER_command +b'\x03'+ k_bytes + b_bytes + FRAME_TAIL
        # ser.write(frame)
        if(int(flag)==1):
            frame = FRAME_HEADER_command +b'\x00'+ k_bytes + b_bytes + FRAME_TAIL
        elif(int(flag)==2):
            frame = FRAME_HEADER_command +b'\x01'+ k_bytes + b_bytes + FRAME_TAIL
        elif(int(flag)==3):
            frame = FRAME_HEADER_command +b'\x02'+ k_bytes + b_bytes + FRAME_TAIL
        else:
            frame = FRAME_HEADER_command +b'\x03'+ k_bytes + b_bytes + FRAME_TAIL
        
        # 发送数据
        ser.write(frame)
        ser.flush()  # 确保数据发送完成
        
        # 打印调试信息
        print(f"发送校准参数:")
        print(f"k = {k:.4f} ({k_bytes.hex()})")
        print(f"b = {b:.4f} ({b_bytes.hex()})")
        print(f"完整帧: {frame.hex()}")
        
        return True
        
    except Exception as e:
        print(f"发送校准参数失败: {e}")
        return False
    
def linear_func(x, k, b):
    """线性方程：y = kx + b"""
    return k * x + b

def parse_frame(data,flag):
    """从接收到的数据中解析有效帧，并提取浮点数值"""
    global FRAME_HEADER, FRAME_TAIL
    start_index = data.find(FRAME_HEADER)
    end_index = data.find(FRAME_TAIL, start_index)
    
    if start_index != -1 and end_index != -1:
        frame = data[start_index:end_index + len(FRAME_TAIL)]
        # print(flag)
        # print(f"完整帧内容: {frame.hex()}")  # 打印完整帧的十六进制
        if(int(flag)==1):#解析d0浮点数
            float_bytes = frame[4:8]
        elif(int(flag)==2):#解析d1浮点数
            float_bytes = frame[8:12]
        elif(int(flag)==3):#解析d2浮点数
            float_bytes = frame[12:16]
        else:#解析d3 浮点数
            float_bytes = frame[16:20]
        try:
            return struct.unpack('<f', float_bytes)[0]
        except struct.error as e:
            print(f"解析浮点数失败: {e}")
            return None
    return None

def read_frame_from_serial(ser,flag, timeout=2):
    """从串口读取数据并提取有效帧"""
    buffer = b''
    start_time = time.time()
    # print(flag)
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer += data
            measured = parse_frame(buffer,flag)
            if measured is not None:
                frame_end = buffer.find(FRAME_TAIL) + len(FRAME_TAIL)
                buffer = buffer[frame_end:]
                return measured
    return None

def verification_mode(ser, k=0.7135, b=-0.3434):
    """验证模式函数"""
    print("\n=== 实时验证模式 ===")
    print(f"使用校准参数: k={k:.4f}, b={b:.4f}")
    print("实时显示原始距离和校准后的距离")
    print("按Ctrl+C退出")
    
    try:
        while True:
            measured = read_frame_from_serial(ser,flag)
            if measured is not None:
                calibrated = k * measured + b
                print(f"原始距离: {measured:.3f}m → 校准后距离: {calibrated:.3f}m")
    except KeyboardInterrupt:
        print("\n已退出验证模式")

def calibration_mode(ser,flag):
    """校准模式函数"""
    # 获取用户输入的数据个数
    n = int(input("请输入测试组数: "))
    # 创建空列表存储数据
    actual_distances = [1.2 + (i * 0.6) for i in range(n)] 
    samples_per_distance = 20  # 每个距离采集的样本数
    
    # 存储所有数据的列表
    all_measurements = []  # 存储所有测量值
    all_actual_distances = []  # 存储对应的实际距离
    
    # 对每个实际距离进行数据采集
    for actual_dist in actual_distances:
        print(f"\n=== 准备采集距离 {actual_dist}m 的数据 ===")
        input(f"请将传感器放置在距离 {actual_dist}m 处，按回车开始采集...")
        
        # 清空串口缓冲区
        ser.reset_input_buffer()
        
        # 采集指定数量的样本
        measurements = []
        while len(measurements) < samples_per_distance:
            measured = read_frame_from_serial(ser,flag)
            if measured is not None:
                measurements.append(measured)
                print(f"采集进度: {len(measurements)}/{samples_per_distance}, "
                      f"测量值: {measured:.3f}m")
        
        # 计算当前距离的统计信息
        mean_value = np.mean(measurements)
        std_value = np.std(measurements)
        print(f"\n距离 {actual_dist}m 的统计信息:")
        print(f"平均值: {mean_value:.3f}m")
        print(f"标准差: {std_value:.3f}m")
        
        # 将数据添加到总列表中
        all_measurements.extend(measurements)
        all_actual_distances.extend([actual_dist] * len(measurements))
    
    # 将列表转换为numpy数组
    all_measurements = np.array(all_measurements)
    all_actual_distances = np.array(all_actual_distances)
    
    # 进行线性拟合
    params, covariance = curve_fit(linear_func, all_measurements, all_actual_distances)
    k, b = params
    
    # 计算R²决定系数
    y_pred = linear_func(all_measurements, k, b)
    r_squared = 1 - (np.sum((all_actual_distances - y_pred) ** 2) / 
                    np.sum((all_actual_distances - np.mean(all_actual_distances)) ** 2))
    
    # 打印拟合结果
    print("\n=== 拟合结果 ===")
    print(f"线性方程: 实际距离 = {k:.4f} × 测量距离 + {b:.4f}")
    print(f"R² 决定系数: {r_squared:.4f}")
    
    # 创建图形和设置大小
    plt.figure(figsize=(10, 6), dpi=100)
    
    # 绘制散点图
    plt.scatter(all_measurements, all_actual_distances, alpha=0.5, label='测量数据', color='lightblue')
    
    # 生成拟合线的点
    x_fit = np.linspace(min(all_measurements), max(all_measurements), 100)
    y_fit = linear_func(x_fit, k, b)
    plt.plot(x_fit, y_fit, 'r-', label=f'拟合线: y = {k:.4f}x + {b:.4f}')

    print("\n正在发送校准参数...")
    if send_calibration_params(ser, k, b,flag) :
        print("校准参数发送成功")
    else:
        print("校准参数发送失败")

    # 设置图表属性
    plt.xlabel('测量距离 (m)')
    plt.ylabel('实际距离 (m)')
    plt.title('距离传感器校准曲线')
    plt.grid(True)
    plt.legend(loc='best')
    
    # 调整图表布局
    plt.tight_layout()
    
    # 保存图像和数据
    plt.savefig('calibration_plot.png', dpi=300, bbox_inches='tight')
    print("\n校准曲线已保存为 calibration_plot.png")
    
    # 保存原始数据到CSV文件
    with open('calibration_data.csv', 'w') as f:
        f.write('实际距离(m),测量距离(m)\n')
        for actual, measured in zip(all_actual_distances, all_measurements):
            f.write(f'{actual:.3f},{measured:.3f}\n')
    print("原始数据已保存为 calibration_data.csv")
    return k, b

def main():
    """主函数：选择模式并执行相应功能"""
    # 串口配置
    serial_port = "COM14"  # 请根据实际情况修改串口号
    baud_rate = 115200
    try:
        
        # 连接串口
        ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
        print("串口连接成功")
        # send_calibration_params(ser, 1, -1.19 ,0)
        # send_calibration_params(ser, 1.0123, -1.2021 ,1)
        # send_calibration_params(ser, 1.0225, -1.1559 ,2)
        # send_calibration_params(ser, 0.9977, -1.1785 ,3)
        # send_calibration_params(ser, 1.0365, -1.2192 ,4)
        while True:
            print("\n=== 模式选择 ===")
            print("1. 校准模式")
            print("2. 固定基站位置模式")
            print("3. 退出程序")
            
            choice = input("请选择模式 (1/2/3): ")
            
            if choice == '1':
                # 校准模式
                flag = input("请选择校准对象 1：Do、2：D1、3：D2、4：D3 ")
                # print(flag)
                k, b = calibration_mode(ser,flag)
                
                # 校准后询问是否进入验证模式
                verify = input("\n是否进入验证模式检验校准结果？(y/n): ")
                if verify.lower() == 'y':
                    verification_mode(ser, k, b)
            
            elif choice == '2':
                # 直接进入位置设置模式，使用预设的校准参数
                # 提示用户输入值，并用空格分隔
                flag = input("请选择基站 1：Do、2：D1、3：D2、4：D3 ")
                x, y = map(float, input("请输入基站"+flag+"的x和y值，用空格隔开: ").split())
                if send_position(ser,x, y,flag) :
                    print("位置参数发送成功")
                else:
                    print("校准参数发送失败")

            elif choice == '3':
                print("\n程序已退出")
                break
            
            else:
                print("\n无效的选择，请重新输入")
    
    except Exception as e:
        print(f"发生错误: {e}")
    
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\n串口已关闭")

if __name__ == "__main__":
    main()
