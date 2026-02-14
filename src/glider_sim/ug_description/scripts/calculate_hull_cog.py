import numpy as np

def calculate_hull_cg(m_hull, m_batt, pos_batt_xyz):
    """
    计算 Hull 需要的质心位置以达到配平。
    基于原点 (0,0,0) 为浮心，且浮力作用在原点的假设。
    """
    
    # 提取电池坐标
    x_b, y_b, z_b = pos_batt_xyz
    
    # 1. 计算 X 轴平衡 (Pitch 配平)
    # m_hull * x_h + m_batt * x_b = 0
    x_hull = - (m_batt * x_b) / m_hull
    
    # 2. 计算 Y 轴平衡 (Roll 配平)
    # m_hull * y_h + m_batt * y_b = 0
    y_hull = - (m_batt * y_b) / m_hull
    
    # 3. Z 轴 (垂直重心)
    # 垂直位置不影响水平静平衡，但影响稳性。
    # 无法通过此方程解出，通常沿用设计值。
    # 这里我们返回 None，由用户指定或保留原值。
    z_hull = None 
    
    return [x_hull, y_hull, z_hull]

# ==========================================
# 输入参数 (来自你的 XML)
# ==========================================
MASS_HULL = 20.485   # kg
MASS_BATT = 10.0     # kg

# 电池包的位置 (你提供的)
# 电池在 -0.12385 (偏前)
POS_BATT = [-0.12385, 0.0, 0.0] 

# ==========================================
# 执行计算
# ==========================================
calculated_cg = calculate_hull_cg(MASS_HULL, MASS_BATT, POS_BATT)

print("-" * 40)
print(f"【配平计算结果】")
print(f"目标：浮心位于 (0,0,0) 且姿态水平")
print(f"已知：")
print(f"  - 电池质量: {MASS_BATT} kg, 位置: {POS_BATT}")
print(f"  - Hull质量: {MASS_HULL} kg")
print("-" * 40)
print(f"求得 Hull (空船) 必须的质心坐标为:")
print(f"X = {calculated_cg[0]:.6f} m")
print(f"Y = {calculated_cg[1]:.6f} m")
print(f"Z = (由几何结构决定，建议为负值以保证稳性)")

# 验证验算
total_moment = (MASS_HULL * calculated_cg[0]) + (MASS_BATT * POS_BATT[0])
print("-" * 40)
print(f"验算力矩总和 (应为0): {total_moment:.6f}")

if calculated_cg[0] > 0:
    print(f"结论：因为电池偏前 ({POS_BATT[0]})，Hull 的重心必须偏后 ({calculated_cg[0]:.4f})。")
else:
    print(f"结论：Hull 重心需偏前。")





# #!/usr/bin/env python3
# """
# 计算hull质心位置，使整机综合质心达到目标位置

# 已知条件：
# - 各link的质量和位置
# - 目标综合质心位置
# - 求解hull质心 (x_h, y_h, z_h)
# """

# import numpy as np

# # ===== 目标综合质心 =====
# target_cog = np.array([0.0, 0.0, -0.05])

# # ===== 各Link参数 =====
# # 格式: (名称, 质量, [质心x, 质心y, 质心z])
# # 质心位置是相对于base_link原点

# links = {
#     # hull质心待求，先用占位符
#     'hull': {
#         'mass': 24.48,
#         'cog': None  # 待求
#     },
#     'battery': {
#         'mass': 10.0,
#         # joint origin: (-0.12385, 0, 0), inertial origin: (0, 0, 0)
#         'cog': np.array([-0.12385, 0.0, 0.0])
#     },
#     'rudder': {
#         'mass': 0.005,
#         # joint origin: (-0.41735, 0, 0.0905), inertial origin: (0, 0, 0)
#         'cog': np.array([-0.41735, 0.0, 0.0905])
#     },
#     # 'wing': {
#     #     'mass': 4.0,
#     #     # joint origin: (-0.325, 0, 0.15), inertial origin: (0, 0, 0)
#     #     # 现在wing_link没有被合并，所以质心就在joint origin处
#     #     'cog': np.array([-0.325, 0.0, 0.15])
#     # },
#     'bladder': {
#         'mass': 0.001,
#         # joint origin: (-0.361, 0, -0.052), inertial origin: (0, 0, 0)
#         'cog': np.array([-0.361, 0.0, -0.052])
#     }
# }

# # ===== 计算 =====

# # 总质量
# total_mass = sum(link['mass'] for link in links.values())
# print(f"总质量: {total_mass:.4f} kg")

# # 除hull外的质量加权质心贡献
# other_mass = 0.0
# other_moment = np.array([0.0, 0.0, 0.0])

# for name, link in links.items():
#     if name == 'hull':
#         continue
#     m = link['mass']
#     cog = link['cog']
#     other_mass += m
#     other_moment += m * cog
#     print(f"{name:10s}: 质量={m:6.3f}kg, 质心=({cog[0]:+.4f}, {cog[1]:+.4f}, {cog[2]:+.4f}), 贡献=({m*cog[0]:+.4f}, {m*cog[1]:+.4f}, {m*cog[2]:+.4f})")

# print(f"\n除hull外总质量: {other_mass:.4f} kg")
# print(f"除hull外质量矩: ({other_moment[0]:+.4f}, {other_moment[1]:+.4f}, {other_moment[2]:+.4f})")

# # hull质量
# hull_mass = links['hull']['mass']
# print(f"\nhull质量: {hull_mass:.4f} kg")

# # 求解hull质心
# # 公式: target_cog = (hull_mass * hull_cog + other_moment) / total_mass
# # 变形: hull_cog = (target_cog * total_mass - other_moment) / hull_mass

# hull_cog = (target_cog * total_mass - other_moment) / hull_mass

# print(f"\n===== 计算结果 =====")
# print(f"hull质心位置 (相对于base_link原点):")
# print(f"  cog_x = {hull_cog[0]:+.6f} m")
# print(f"  cog_y = {hull_cog[1]:+.6f} m")
# print(f"  cog_z = {hull_cog[2]:+.6f} m")

# # 验证
# links['hull']['cog'] = hull_cog
# verify_moment = np.array([0.0, 0.0, 0.0])
# for name, link in links.items():
#     m = link['mass']
#     cog = link['cog']
#     verify_moment += m * cog

# verify_cog = verify_moment / total_mass
# print(f"\n===== 验证 =====")
# print(f"计算得到的综合质心: ({verify_cog[0]:+.6f}, {verify_cog[1]:+.6f}, {verify_cog[2]:+.6f})")
# print(f"目标综合质心:       ({target_cog[0]:+.6f}, {target_cog[1]:+.6f}, {target_cog[2]:+.6f})")
# print(f"误差: ({abs(verify_cog[0]-target_cog[0]):.2e}, {abs(verify_cog[1]-target_cog[1]):.2e}, {abs(verify_cog[2]-target_cog[2]):.2e})")

# # 浮力平衡检查
# print(f"\n===== 浮力平衡检查 =====")
# fluid_density = 1028.0  # kg/m^3
# g = 9.81  # m/s^2
# hull_volume_neutral = 0.0335467  # m^3 (500cc油囊时)

# buoyancy_neutral = fluid_density * g * hull_volume_neutral
# weight = total_mass * g

# print(f"中性浮力(500cc): {buoyancy_neutral:.2f} N")
# print(f"总重力:          {weight:.2f} N")
# print(f"差值:            {buoyancy_neutral - weight:+.2f} N")

# if abs(buoyancy_neutral - weight) < 1.0:
#     print("✓ 500cc时接近中性浮力")
# else:
#     print("⚠ 500cc时浮力与重力不平衡")
