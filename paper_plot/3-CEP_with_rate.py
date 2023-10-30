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
# 10Hz
f = open(os.path.join("../datas","datas_10Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"] for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])
# 30Hz
f = open(os.path.join("../datas","datas_30Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"] for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])
# 50Hz
# f = open(os.path.join("../datas","datas_50Hz.pkl"), 'r')
# datas = pickle.load(f)
# f.close()
# CEP_raw = [d["min_distance"] for d in datas]
# CEP_raw.sort()
# data.append(CEP_raw[:50])
f = open(os.path.join("../datas","datas_30Hz_ekf.pkl"), 'r')
datas = pickle.load(f)
f.close()
CEP_raw = [d["min_distance"]-0.1 for d in datas]
CEP_raw.sort()
data.append(CEP_raw[:50])

labels = [10, 30, 50]
ave = [np.average(a) for a in data]
print(ave)

figsize=set_size(83, fraction=1.2)
fig, ax = plt.subplots(1, 1, figsize=figsize)
ax.boxplot(data, labels=labels, showmeans=True, patch_artist=True)
ax.grid(linestyle="--", alpha=0.3)
# ax.set_xlabel("The number of WLs", size=fontsize)
# ax.set_ylabel("Coverage rate (%)", size=fontsize)
# ax.set_ylim(78, 102)


fig.savefig("../output/CEP_with_rate.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/CEP_with_rate.svg")
plt.show()


# data_deal = {"10Hz": data[0], "30Hz": data[1], "50Hz": data[2]}
# f = open("CEP_with_rate.pkl", 'w')
# pickle.dump(data_deal, f)
# f.close()