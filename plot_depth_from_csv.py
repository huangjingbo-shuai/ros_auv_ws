#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import matplotlib.pyplot as plt

# ======= 真实物理参数 ========
WATER_DENSITY = 1000.0       # kg/m^3
GRAVITY = 9.80665            # m/s^2
REFERENCE_TIME_LIMIT = 3    # 前 N 秒用于估算参考压强
IS_PRESSURE_IN_HPA = True    # ✅ 关键修改点：表示数据是百帕
# ============================

filename = "depth_from_pressure.csv"
times, pressures_raw, depths = [], [], []

# Step 1: 读取CSV数据
with open(filename, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    rows = [row for row in reader if len(row) == 3]

# Step 2: 提取时间和压强
for row in rows:
    t = float(row[0])
    p = float(row[1])
    times.append(t)
    pressures_raw.append(p)

# Step 3: 转换单位（hPa → Pa）
if IS_PRESSURE_IN_HPA:
    pressures = [x * 100 for x in pressures_raw]
else:
    pressures = pressures_raw

# Step 4: 提取参考压强（单位：Pa）
reference_pressures = [p for t, p in zip(times, pressures) if t <= REFERENCE_TIME_LIMIT]
if not reference_pressures:
    raise ValueError("没有足够的数据点用于参考压强估计。请检查CSV文件。")

P0 = sum(reference_pressures) / len(reference_pressures)
print(f"自动提取的参考压强 P0 = {P0:.2f} Pa")

# Step 5: 重算水深
depths = [-(p - P0) / (WATER_DENSITY * GRAVITY)-0.3 for p in pressures]

# Step 6: 绘图
plt.figure(figsize=(10, 6))

# 压强图
plt.subplot(2, 1, 1)
plt.plot(times, pressures, label="Pressure (Pa)", color='tab:blue')
plt.ylabel("Pressure (Pa)")
plt.title("Static Pressure and Computed Depth (Corrected from hPa)")
plt.grid(True)
plt.legend()

# 深度图
plt.subplot(2, 1, 2)
plt.plot(times, depths, label="Depth (m)", color='tab:green')
plt.xlabel("Time (s)")
plt.ylabel("Depth (m)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
