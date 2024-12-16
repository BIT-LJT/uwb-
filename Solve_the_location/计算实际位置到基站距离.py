import numpy as np

true_position = (1.8, 0.6, 0.0)
manual_distances = [
    (0, 3.218695387886216237),
    (1, 3.655133376499413051),
    (2, 3.28633534503099668),
    (3, 4.219004621945797299)
]
ANCHOR_POSITIONS = {
    # 'd0': (0.0, 0.0, 1.0),
    # 'd1': (0.0, 0.6, 2.0),
    # 'd2': (4.2, 1.2, 3.0),
    # 'd3': (4.2, 0.0, 4.0)
    'd0': (0.0, 0.0 ,0.0),  # 基站0的坐标 (x0, y0,z0)
    'd1': (0.6, 0.6,0.8),  # 基站1的坐标 (x1, y1,z1)
    'd2': (2.4, 0.6,0.4),  # 基站2的坐标 (x2, y2,z2)
    'd3': (3.0, 0.0,0.0)   # 基站3的坐标 (x3, y3,z3)
}

print("基站距离验证:")
for anchor_id, manual_distance in manual_distances:
    anchor_position = ANCHOR_POSITIONS[f'd{anchor_id}']
    calculated_distance = np.sqrt(
        (true_position[0] - anchor_position[0])**2 +
        (true_position[1] - anchor_position[1])**2 +
        (true_position[2] - anchor_position[2])**2
    )
    print(f"基站 d{anchor_id}:")
    print(f"  手动写入距离 = {manual_distance:.6f}")
    print(f"  计算得到的距离 = {calculated_distance:.6f}")
    print(f"  误差 = {abs(manual_distance - calculated_distance):.6f}")
