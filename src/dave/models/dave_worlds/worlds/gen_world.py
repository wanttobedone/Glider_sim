# ---------------- 配置区域 ----------------
MODEL_URI = "model://sand_heightmap"
Z_DEPTH = -200           # 想要的深度
GRID_SIZE = 5            # 5x5 的网格
STEP_SIZE = 100          # 模型的边长 (米)，也是步长        
# ----------------------------------------

# 计算起始偏移量，让整个网格中心在 (0,0)
offset = (GRID_SIZE - 1) * STEP_SIZE / 2

# 头部：定义一个容器模型
# static=true 非常重要，确保整个海底固定不动，不消耗额外物理计算资源
print(f"""
<model name="seabed_main">
  <static>true</static>
  <pose>0 0 {Z_DEPTH} 0 0 0</pose>
""")

# 中部：循环生成嵌套模型 (Nested Models)
for x in range(GRID_SIZE):
    for y in range(GRID_SIZE):
        # 计算相对于“容器中心”的局部坐标
        pos_x = x * STEP_SIZE - offset
        pos_y = y * STEP_SIZE - offset
        
        # 注意：这里我们使用 include 嵌套
        # 这里的 pose 是相对于 seabed_main 的局部坐标
        print(f"""
    <include>
      <name>tile_{x}_{y}</name>
      <uri>{MODEL_URI}</uri>
      <pose>{pos_x:.1f} {pos_y:.1f} 0 0 0 0</pose>
    </include>""")

# 尾部：闭合标签
print("</model>")