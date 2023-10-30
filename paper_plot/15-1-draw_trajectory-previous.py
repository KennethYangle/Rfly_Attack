#! coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import os, io
import pickle
import json5
from plot_tools import get_line_style, set_size, svg_to_emf

# 全局美化格式
with io.open("params.json", "r", encoding="utf-8") as fp:
    params = json5.load(fp)
print(params)
plt.rcParams.update(params)

# 载入线型
style_dict = get_line_style()


figsize=set_size(70, hw_ratio=0.8)
fig, ax = plt.subplots(1, 1, figsize=figsize)
ax = plt.axes(projection='3d')
f = open(os.path.join("../datas","datas_10Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
# Comment this line when comparing other datas
datas.sort(key=lambda x: x["min_distance"])


# Show results directly
for i in range(len(datas)):
    sphere_traj = datas[i]["sphere_traj"][:-80]
    mav_traj = datas[i]["mav_traj"][:-80]
    ax.plot([s[0] for s in sphere_traj], [s[1] for s in sphere_traj], [s[2] for s in sphere_traj], linewidth=0.5, color='#ff7f0e')
    ax.plot([m[0] for m in mav_traj], [m[1] for m in mav_traj], [m[2] for m in mav_traj], linewidth=0.5, color='#1f77b4')

# # Show results directly
# for t in range(0, len(datas[0]["sphere_traj"])-30, 10):
#     plt.ion()  #打开交互模式
#     for i in range(50):
#         sphere_traj = datas[i]["sphere_traj"][:t]
#         mav_traj = datas[i]["mav_traj"][:t]
#         ax.plot([s[0] for s in sphere_traj], [s[1] for s in sphere_traj], [s[2] for s in sphere_traj], color='#ff7f0e')
#         ax.plot([m[0] for m in mav_traj], [m[1] for m in mav_traj], [m[2] for m in mav_traj], color='#1f77b4')
#     plt.show()
#     plt.pause(0.001)






# 分别上下旋转和左右旋转，可以手动调整读取参数
ax.view_init(32, -8)
# # 前3个参数用来调整各坐标轴的缩放比例
# ax1.get_proj = lambda: np.dot(mplot3d.Axes3D.get_proj(ax1), np.diag([0.8, 1.2, 0.8, 1]))
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
ax.set_ylim(-1, 25)
ax.set_zlim(-1, 5)
# plt.legend(('Interceptor', 'Target'), frameon=False, loc='upper center', ncol=2, handlelength=2)    # 图例

# 保存图片
# fig.savefig("../output/trajectory.raw", dpi=1200)
# fig.savefig("../output/trajectory.png", dpi=1200)
fig.savefig("../output/trajectory-previous.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/trajectory-previous.svg")
plt.show()
