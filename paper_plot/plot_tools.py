#! coding=utf-8
import numpy as np
import subprocess, os

# 更多线性：https://matplotlib.org/stable/gallery/lines_bars_and_markers/linestyles.html
def get_line_style():
    style_dict = {
        'Blue':  dict(linestyle='-', markersize=6, color='#2b83ba'),    # marker='v'，蓝色——
        'Orange':dict(linestyle='--', markersize=6, color='#fdae61'),   # marker='o'，橙色--
        'Green': dict(linestyle=':', markersize=6, color='#abdda4'),    # marker='s'，绿色...
        'Red':   dict(linestyle='-.', markersize=6, color='#d7191c'),   # marker='*'，红色-.-.
        'Yellow':dict(linestyle=(5, (10, 3)), markersize=6, color='#ffffbf'), # marker='*'，黄色- - -

        'Blue_Solid':  dict(linestyle='-', markersize=6, color='#2b83ba'),   # marker='v'，蓝色——
        'Orange_Solid':dict(linestyle='-', markersize=6, color='#fdae61'), # marker='o'，橙色——
        'Green_Solid': dict(linestyle='-', markersize=6, color='#abdda4'),   # marker='s'，绿色——
        'Red_Solid':   dict(linestyle='-', markersize=6, color='#d7191c'), # marker='*'，红色——
        'Yellow_Solid':dict(linestyle='-', markersize=6, color='#ffffbf'), # marker='*'，黄色——

        'Blue_Dashed':  dict(linestyle='--', markersize=6, color='#2b83ba'),   # marker='v'，蓝色--
        'Orange_Dashed':dict(linestyle='--', markersize=6, color='#fdae61'), # marker='o'，橙色--
        'Green_Dashed': dict(linestyle='--', markersize=6, color='#abdda4'),   # marker='s'，绿色--
        'Red_Dashed':   dict(linestyle='--', markersize=6, color='#d7191c'), # marker='*'，红色--
        'Yellow_Dashed':dict(linestyle='--', markersize=6, color='#ffffbf'), # marker='*'，黄色--

        'Blue_Dotted':  dict(linestyle=':', markersize=6, color='#2b83ba'),   # marker='v'，蓝色...
        'Orange_Dotted':dict(linestyle=':', markersize=6, color='#fdae61'), # marker='o'，橙色...
        'Green_Dotted': dict(linestyle=':', markersize=6, color='#abdda4'),   # marker='s'，绿色...
        'Red_Dotted':   dict(linestyle=':', markersize=6, color='#d7191c'), # marker='*'，红色...
        'Yellow_Dotted':dict(linestyle=':', markersize=6, color='#ffffbf'), # marker='*'，黄色...
        
        'Blue_Dashdot':  dict(linestyle='-.', markersize=6, color='#2b83ba'),   # marker='v'，蓝色-.-.
        'Orange_Dashdot':dict(linestyle='-.', markersize=6, color='#fdae61'), # marker='o'，橙色-.-.
        'Green_Dashdot': dict(linestyle='-.', markersize=6, color='#abdda4'),   # marker='s'，绿色-.-.
        'Red_Dashdot':   dict(linestyle='-.', markersize=6, color='#d7191c'), # marker='*'，红色-.-.
        'Yellow_Dashdot':dict(linestyle='-.', markersize=6, color='#ffffbf'), # marker='*'，黄色-.-.
        
        'Blue_Longdash':  dict(linestyle=(5, (10, 3)), markersize=6, color='#2b83ba'),   # marker='v'，蓝色- - -
        'Orange_Longdash':dict(linestyle=(5, (10, 3)), markersize=6, color='#fdae61'), # marker='o'，橙色- - -
        'Green_Longdash': dict(linestyle=(5, (10, 3)), markersize=6, color='#abdda4'),   # marker='s'，绿色- - -
        'Red_Longdash':   dict(linestyle=(5, (10, 3)), markersize=6, color='#d7191c'), # marker='*'，红色- - -
        'Yellow_Longdash':dict(linestyle=(5, (10, 3)), markersize=6, color='#ffffbf'), # marker='*'，黄色- - -
    }
    return style_dict

# 来自：https://jwalton.info/Embed-Publication-Matplotlib-Latex/
def set_size(width, fraction=1, hw_ratio=0.618, height=None):
    """Set figure dimensions to avoid scaling in LaTeX.

    Parameters
    ----------
    width: float\n
            Document textwidth or columnwidth in mm.\n
            Usually the single column is set to 83mm and the double column is set to 176mm.
    fraction: float, optional\n
            Fraction of the width which you wish the figure to occupy
    hw_ratio: float, optional\n
            Ratio of the hight to width, default is the Golden ratio

    Returns
    -------
    fig_dim: tuple\n
            Dimensions of figure in inches
    """
    # Width of figure (in mm)
    fig_width_mm = width * fraction

    # Convert from pt to inches
    inches_per_mm = 0.03937008

    # Figure width in inches
    fig_width_in = fig_width_mm * inches_per_mm
    # Figure height in inches
    fig_height_in = fig_width_in * hw_ratio

    if height is not None:
        fig_height_mm = height * fraction
        fig_height_in = fig_height_mm * inches_per_mm

    fig_dim = (fig_width_in, fig_height_in)

    return fig_dim


def svg_to_emf(svg_figpath):
    '''1. 如果是.svg的绝对路径，True改为False；
       2. inkscape.exe的文件路径中必须是双斜杠\\，单斜杠\会出错；
       3. subprocess.call中的shell=True不可省略，否则会报错。'''
    if True:
        cwd_path = os.getcwd()
        svg_figpath = os.path.join(cwd_path, svg_figpath)
        print(svg_figpath)
    # inkscape.exe的绝对路径
    inkscape_path = 'D:\\Inkscape\\bin\\inkscape.exe'
    if svg_figpath is not None:
        path, svgfigname = os.path.split(svg_figpath)
        figname, figform = os.path.splitext(svgfigname)
        emf_figpath = os.path.join(path, figname + '.emf')
        subprocess.call("{} {} -T -o {}".format(inkscape_path, svg_figpath, emf_figpath), shell=True)
        # os.remove(svg_figpath)
