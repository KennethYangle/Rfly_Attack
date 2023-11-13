#! coding=utf-8

import argparse 
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
from matplotlib.ticker import MaxNLocator
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

# plt.figure(1)

def main(args):
    tmin = 0
    tmax = 1e10
    if args.time is not None:
        trange = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', args.time)]
        tmin = trange[0]
        if len(trange) == 2:
            tmax = trange[1]

    f = open(args.log)
    lines = f.readlines() 
    nvar = len(args.variable)
    raw_datas = [[] for v in range(nvar)]
    cnt = 0
    time = 0
    index = [[] for v in range(nvar)]
    data_len = [0 for v in range(nvar)]
    for v in range(nvar):
        for line in lines:
            if line.startswith("time:"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                if v == 0: 
                    time = tmp[0] + 0.05
                else:
                    time = tmp[0]
            if time < tmin or time > tmax:
                continue
            if line.startswith(args.variable[v]+":"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                # print(tmp)
                if data_len[v]==0:
                    data_len[v] = len(tmp)
                if data_len[v]!=0 and len(tmp) == data_len[v]:
                    if args.norm:
                        raw_datas[v].append([np.linalg.norm(tmp)])
                    else:
                        raw_datas[v].append(tmp)
                    index[v].append(time)
        cnt += 1
    print(data_len)
    # print(raw_datas)
    datas = [[[] for i in range(len(raw_datas[v][0]))] for v in range(nvar)]
    for v in range(nvar):
        for i in range(len(raw_datas[v])):
            for j in range(len(raw_datas[v][0])):
                # print(i,j,raw_datas[i][j])
                datas[v][j].append(raw_datas[v][i][j])
    # print(datas)
    
    if args.self:
        for v in range(nvar):
            plt.figure(v)
            for i in range(len(datas[v])):
                ax.plot(index[v], datas[v][i], label="{}[{}]".format(args.variable[v], i), linewidth=args.linewidth)
            plt.legend(loc='upper center')
        plt.show()

    if args.plotxy:
        for v in range(nvar):
            # plt.figure(v)
            figsize=set_size(83, fraction=1.2)
            fig, ax = plt.subplots(1, 1, figsize=figsize)
            ax.xaxis.set_major_locator(MaxNLocator(5)) 
            ax.yaxis.set_major_locator(MaxNLocator(5)) 
            ax.plot(datas[v][0], datas[v][1], label="{}[x:y]".format(args.variable[v]), linewidth=args.linewidth, **style_dict["Blue_Solid"])
            if args.range is not None:
                arange = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', args.range)]
                plt.axis(arange)
            ax.set_xlim(-6, 0.3)
            # plt.legend(frameon=False, loc='upper center', ncol=1, handlelength=2)
        fig.savefig("../output/plotxy-sim.svg", format='svg', bbox_inches='tight')
        svg_to_emf("../output/plotxy-sim.svg")
        plt.show()

    if args.subplot > 0:
        figsize=set_size(190, fraction=1.2, hw_ratio=0.6)
        fig, ax = plt.subplots(2, 1, figsize=figsize)
        gap = nvar // args.subplot
        # # 两种线型效果不好
        # for i in range(1, args.subplot+1):
        #     for v in range((i-1)*gap, i*gap):
        #         if v % 2 == 0:
        #             ax[i-1].plot(index[v], datas[v][0], label="{}".format(args.variable[v]), linewidth=args.linewidth, **style_dict["Blue_Solid"])
        #         if v % 2 == 1:
        #             ax[i-1].plot(index[v], datas[v][0], label="{}".format(args.variable[v]), linewidth=args.linewidth, **style_dict["Orange_Solid"])
        for i in range(1, args.subplot+1):
            for v in range((i-1)*gap, i*gap):
                ax[i-1].plot(index[v], datas[v][0], label="{}".format(args.variable[v]), linewidth=1)
            # ax[i-1].legend(loc='upper center')
        # ax[0].set_ylim(-50, 1000)
        fig.savefig("../output/subplot-loss.svg", format='svg', bbox_inches='tight')
        svg_to_emf("../output/subplot-loss.svg")
        plt.show()

    if not args.self and not args.plotxy and args.subplot<=0:
        for i in range(len(datas[0])):
            plt.figure(i)
            for v in range(nvar):
                if i < len(datas[v]):
                    ax.plot(index[v], datas[v][i], label="{}[{}]".format(args.variable[v], i), linewidth=args.linewidth)
                else:
                    ax.plot(index[v], datas[v][0], label="{}[{}]".format(args.variable[v], 0), linewidth=args.linewidth)
            plt.legend(loc='upper center')
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('log', help='log file path')
    parser.add_argument('variable', nargs='+', help='variable to plot')
    parser.add_argument('-s', '--self', action='store_true', help='draw the various components of the variable on a figure')
    parser.add_argument('-p', '--plotxy', action='store_true', help='draw a 2-dimensional graph')
    parser.add_argument('-n', '--norm', action='store_true', help="calc the norm of variable")
    parser.add_argument('-l', '--linewidth', default=2, type=float, help='line width')
    parser.add_argument('-r', '--range', default=None, help='axises range, work with plotxy. usage: "xmin xmax ymin ymax"')
    parser.add_argument('-t', '--time', default=None, help='time range. usage: "tmin tmax" or tmin')
    parser.add_argument('--subplot', default=0, type=int, help='subplot number')
    args = parser.parse_args()
    print(args)
    main(args)