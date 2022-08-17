import pickle
from dataclasses import dataclass
from typing import List

import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from scipy.interpolate import interp1d
from comparisons_long_add_safe import ValidationResult


def latexify(fontsize: int = 10):
    params = {'backend': 'ps',
              'text.latex.preamble': r"\usepackage{gensymb} \usepackage{amsmath}",
              'axes.labelsize': fontsize,
              'axes.titlesize': fontsize,
              'legend.fontsize': fontsize,
              'xtick.labelsize': fontsize,
              'ytick.labelsize': fontsize,
              'text.usetex': True,
              'font.family': 'serif'
              }

    matplotlib.rcParams.update(params)


def boxplot_timings(computation_times: List[np.ndarray], labels: List[str], filename: str):
    latexify()
    fig, ax = plt.subplots(figsize=(4, 4))
    ax = plt.gca()

    ax.grid(True, linestyle='dotted')
    ax.set_axisbelow(True)

    plt.ylabel('Computation Time (s)')

    boxplots = plt.boxplot(computation_times,
                           sym="r+",
                           labels=[''] * len(computation_times),
                           notch=False,
                           # vert=True,
                           whis=1e3,
                           showmeans=True,
                           #           bootstrap=None,
                           #           usermedians=None,
                           #           conf_intervals=None,
                           patch_artist=True,
                           meanline=True,
                           meanprops={"linewidth": 0.5, "linestyle": "solid", "color": "black"},
                           medianprops={"linewidth": 0.5, "linestyle": (0, (1, 0.5)), "color": "black"},
                           boxprops={"linewidth": 1, "facecolor": "whitesmoke"}
                           )
    # ax.add_patch(Rectangle((0, 0), 0, 0, facecolor=facecolors[2],  label=label, alpha=0.3))
    plt.rcParams['hatch.linewidth'] = 0.5
    plt.xticks(np.arange(len(labels)) + 1, labels)

    plt.plot([], [], linestyle="solid", color="black", label="mean")
    plt.plot([], [], linestyle=(0, (1, 0.5)), color="black", label="median")

    plt.legend()
    plt.tight_layout()
    plt.savefig(filename + ".pdf", dpi=300)
    # plt.show()


if __name__ == "__main__":
    labels = ["e-l", "e", "5c-l", "5c", "3c-l", "3c", "hp-l", "hp"]

    with open("save_data_long_add_safe", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    timings = [validation.comp_times for validation in b]
    timings = [np.array(timing).flatten() for timing in timings]
    # labels = [validation.obstacle_avoidance_label + str(validation.lifting_used) for validation in b]
    boxplot_timings(timings, labels=labels, filename="timings_long_add_safe")

    with open("save_data_short_add_safe", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    timings = [validation.comp_times for validation in b]
    timings = [np.array(timing).flatten() for timing in timings]
    # labels = [validation.obstacle_avoidance_label + str(validation.lifting_used) for validation in b]
    boxplot_timings(timings, labels=labels, filename="timings_short_add_safe")

    with open("save_data_long_no_safe", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    timings = [validation.comp_times for validation in b]
    timings = [np.array(timing).flatten() for timing in timings]
    # labels = [validation.obstacle_avoidance_label + str(validation.lifting_used) for validation in b]
    boxplot_timings(timings, labels=labels, filename="timings_long_no_safe")
