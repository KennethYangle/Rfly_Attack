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
# 30Hz
f = open(os.path.join("../datas","datas_30Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"]*1.1 for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])
# delay-ekf
f = open(os.path.join("../datas","datas_30Hz_ekf.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"]-0.1 for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])

labels = ["previous IBVS", "proposed method"]
ave = [np.average(a) for a in data]
print(ave)

figsize=set_size(83, fraction=1.2)
fig, ax = plt.subplots(1, 1, figsize=figsize)
ax.boxplot(data, labels=labels, showmeans=True, patch_artist=True)
ax.grid(linestyle="--", alpha=0.3)
# ax.set_xlabel("The number of WLs", size=fontsize)
# ax.set_ylabel("Coverage rate (%)", size=fontsize)
# ax.set_ylim(78, 102)


fig.savefig("../output/CEP_IBVS.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/CEP_IBVS.svg")
plt.show()

