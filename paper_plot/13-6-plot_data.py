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

def quad2euler(q):
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    mav_roll = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
    mav_pitch = np.arcsin(2*(q0*q2 - q1*q3))
    mav_yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    return [mav_roll, mav_pitch, mav_yaw]

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
    for i in range(len(lines)):
        line = lines[i]
        for v in range(nvar):
            if line.startswith("time:"):
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                time = tmp[0]
            if time < tmin or time > tmax:
                continue
            if line.startswith(args.variable[v]+":"):
                line = lines[i] + lines[i+1] + lines[i+2] + lines[i+3]
                tmp = [float(a) for a in re.findall(r'-?\d+\.?\d*e?[-+]?\d*', line)]
                # print(tmp)
                if np.linalg.norm(tmp) > 1.1 or np.linalg.norm(tmp) < 0.9:
                    continue
                tmp = quad2euler(tmp)
                tmp[0] *= 2.0
                tmp[1] *= 2.0
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
            # plt.figure(v)
            figsize=set_size(36, fraction=1.2, height=46.16)
            fig, ax = plt.subplots(1, 1, figsize=figsize)
            ax.plot(index[v], datas[v][0], label="mav_roll", linewidth=1)
            ax.plot(index[v], datas[v][1], label="mav_pitch", linewidth=1)
            ax.plot(index[v], datas[v][2], label="mav_yaw", linewidth=1)
            # ax.set_ylim(-0.9, 0.9)
            # plt.legend(loc='upper center')
        fig.savefig("../output/attitude-local-real-moving.svg", format='svg', bbox_inches='tight')
        svg_to_emf("../output/attitude-local-real-moving.svg")
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
        figsize=set_size(74.7, fraction=1.2, hw_ratio=0.65)
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
        ax[0].set_ylim(-50, 1000)
        fig.savefig("../output/subplot-sim.svg", format='svg', bbox_inches='tight')
        svg_to_emf("../output/subplot-sim.svg")
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