#! coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mpl_toolkits import mplot3d
import os, io
import pickle
import json5
import glob
import scipy.io
from plot_tools import get_line_style, set_size, svg_to_emf

# 全局美化格式
with io.open("params.json", "r", encoding="utf-8") as fp:
    params = json5.load(fp)
print(params)
plt.rcParams.update(params)

data_dir = "../data_20220427"
txt_fileL = glob.glob("{}/*.txt".format(data_dir))

## data_stash = [[min_diff_pos_ENU, mav_pos_ENU, obj_pos_ENU], [], [], ...]
data_stash = []


# 1. 原始数据处理
for data_path in txt_fileL:
    mav_pos_ENU = []
    obj_pos_ENU = []
    diff_pos_ENU = []

    with io.open(data_path,"r",encoding="utf-8") as fp:
        dataL = fp.readlines()

    for data in dataL:
        data_floatL = [float(i) for i in data.split(",")]
        mav_E,mav_N,mav_U = data_floatL[4:7]
        obj_N,obj_E,obj_D = data_floatL[7:10]

        mav_pos_ENU.append([mav_E,mav_N,mav_U])
        obj_pos_ENU.append([obj_E,obj_N,-obj_D])
        diff_pos_ENU.append(np.array([obj_E-mav_E,obj_N-mav_N,-obj_D-mav_U]))
    if len(diff_pos_ENU) < 100:
        continue
    # print(data_path)
    dis_list = [np.linalg.norm(x) for x in diff_pos_ENU]
    min_dis = min(dis_list)
    idx = dis_list.index(min_dis)

    min_diff_pos_ENU = diff_pos_ENU[idx]
    start_idx = 0
    for i in range(len(mav_pos_ENU)):
        if mav_pos_ENU[i][1] >= 35:
            start_idx = i
            break
    data_stash.append([min_diff_pos_ENU[:idx+1], mav_pos_ENU[start_idx:idx+1], obj_pos_ENU])


# 2. 取前100数据
data_stash.sort(key=lambda x:np.linalg.norm(x[0]))
data_stash = data_stash[:100]
scipy.io.savemat('data_stash.mat', {"data_stash": data_stash})


# 4. 画轨迹
figsize=set_size(140, hw_ratio=0.35)
fig, ax = plt.subplots(1, 1, figsize=figsize)
ax = plt.axes(projection='3d')

# Show results directly
for i in range(len(data_stash)):
    mav_traj = data_stash[i][1]
    sphere_traj = data_stash[i][2]
    ax.plot([m[0] for m in mav_traj], [m[1]-35 for m in mav_traj], [m[2] for m in mav_traj], color='#1f77b4', linewidth=0.5)
    ax.plot([s[0] for s in sphere_traj], [s[1]-35 for s in sphere_traj], [s[2] for s in sphere_traj], color='#ff7f0e', linewidth=0.5)

# 分别上下旋转和左右旋转，可以手动调整读取参数
ax.view_init(44, -8)
# 背景白色
ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
new_ticks = np.linspace(-10, 10, 5)
ax.set_xticks(new_ticks)
ax.set_xlim(-10, 10)
ax.set_ylim(0, 35)
ax.set_zlim(-1, 5)

fig.savefig("../output/trajectory-8shape.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/trajectory-8shape.svg")
plt.show()


# # 3. 计算并可视化CEP
# data = np.array([x[0] for x in data_stash])
# print(data.shape)

# scale = 0.2
# data *= scale

# E_aver = np.average(data[:,0])
# U_aver = np.average(data[:,2])
# rouL = [np.sqrt((i[0]-E_aver)**2+(i[2]-U_aver)**2) for i in data]
# rouL.sort()

# CEP_r = rouL[int(len(rouL)/2)]
# print(CEP_r)

# cir1 = Circle(xy = (E_aver,U_aver), radius=CEP_r, alpha=0.5)
# fig, ax = plt.subplots()
# plt.axis("equal")
# plt.grid()
# ax.scatter(data[:,0],data[:,2])
# ax.scatter(E_aver,U_aver,color="r")
# ax.add_patch(cir1)
# plt.xlabel("x (m)")
# plt.ylabel("y (m)")
# # plt.title(f"CEP={np.pi*CEP_r*CEP_r:.6f}"+r"$m^2$")
# fig.savefig("../output/CEP-8shape.svg", format='svg', bbox_inches='tight')
# svg_to_emf("../output/CEP-8shape.svg")
# plt.show()


# # 5. 画label
# fig, ax = plt.subplots()
# x = [1,2,3,4,5]
# y = [1,2,3,4,5]
# plt.plot(x,y,label='Interceptor trajectory')
# x = [2,3,4,5,6]
# y = [1,2,3,4,5]
# plt.plot(x,y,label='Intruder trajectory')
# plt.legend(loc=0)
# fig.savefig("../output/label.png", dpi=1200)
# plt.show()