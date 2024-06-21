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


data = []
# 5m/s
f = open(os.path.join("../datas","datas_DKF_5mps.pkl"), 'r')
datas_raw = pickle.load(f)
data.append(datas_raw)
# 7.5m/s
f = open(os.path.join("../datas","datas_DKF_7_5mps.pkl"), 'r')
datas_raw = pickle.load(f)
data.append(datas_raw)
# 10m/s
f = open(os.path.join("../datas","datas_DKF_10mps.pkl"), 'r')
datas_raw = pickle.load(f)
data.append(datas_raw)

labels = [5, 7.5, 10]
ave = [np.average(a) for a in data]
print(ave)

figsize=set_size(83, fraction=1.2)
fig, ax = plt.subplots(1, 1, figsize=figsize)
ax.boxplot(data, labels=labels, showmeans=True, patch_artist=True)
ax.grid(linestyle="--", alpha=0.3)
# ax.set_xlabel("The number of WLs", size=fontsize)
# ax.set_ylabel("Coverage rate (%)", size=fontsize)
# ax.set_ylim(0.04, 0.612)

fig.savefig("../output/DKF_with_speed.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/DKF_with_speed.svg")
plt.show()
