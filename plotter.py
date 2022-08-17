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


def boxplot_timings(data_set: List[np.ndarray], labels: List[str], filename: str, y_axis_label: str,
                    crashes: List[bool] = None):
    latexify()
    fig, ax = plt.subplots(figsize=(4, 4))
    ax = plt.gca()

    ax.grid(True, linestyle='dotted')
    ax.set_axisbelow(True)

    plt.ylabel(y_axis_label)

    boxplots = ax.boxplot(data_set,
                          sym="r+",
                          labels=[''] * len(data_set),
                          notch=False,
                          whis=1e3,
                          showmeans=True,
                          patch_artist=True,
                          meanline=True,
                          meanprops={"linewidth": 0.5, "linestyle": "solid", "color": "black"},
                          medianprops={"linewidth": 0.5, "linestyle": (0, (1, 0.5)), "color": "black"},
                          boxprops={"linewidth": 1, "facecolor": "whitesmoke"}
                          )
    for i, box in enumerate(boxplots['boxes']):
        if crashes[i] > 0:
            box.set_facecolor("tab:red")

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

    with open("save_data_long_add_safe_2", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    timings = [validation.comp_times for validation in b]
    timings = [np.array(timing).flatten() for timing in timings]
    crashes = [validation.collisions > 0 for validation in b]
    boxplot_timings(timings, labels=labels, filename="timings_long_add_safe", y_axis_label='Computation Time (s)',
                    crashes=crashes)

    with open("save_data_long_add_safe_2", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    s_values = [validation.s_final_all for validation in b]
    s_values = [np.array(s).flatten() for s in s_values]
    crashes = [validation.collisions > 0 for validation in b]
    boxplot_timings(s_values, labels=labels, filename="s_values_long_add_safe", y_axis_label='Maximum Progress (m)',
                    crashes=crashes)

    with open("save_data_short_safe_2", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    timings = [validation.comp_times for validation in b]
    timings = [np.array(timing).flatten() for timing in timings]
    crashes = [validation.collisions > 0 for validation in b]
    boxplot_timings(timings, labels=labels, filename="timings_short_add_safe", y_axis_label='Computation Time (s)',
                    crashes=crashes)

    with open("save_data_short_safe_2", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    s_values = [validation.s_final_all for validation in b]
    s_values = [np.array(s).flatten() for s in s_values]
    crashes = [validation.collisions > 0 for validation in b]
    boxplot_timings(s_values, labels=labels, filename="s_values_short_add_safe", y_axis_label='Maximum Progress (m)',
                    crashes=crashes)

    with open("save_data_long_no_safe_2", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    timings = [validation.comp_times for validation in b]
    timings = [np.array(timing).flatten() for timing in timings]
    crashes = [validation.collisions > 0 for validation in b]
    boxplot_timings(timings, labels=labels, filename="timings_long_no_safe", y_axis_label='Computation Time (s)',
                    crashes=crashes)

    with open("save_data_long_no_safe_2", "rb") as fp:  # Unpickling
        b = pickle.load(fp)
    s_values = [validation.s_final_all for validation in b]
    s_values = [np.array(s).flatten() for s in s_values]
    crashes = [validation.collisions > 0 for validation in b]
    boxplot_timings(s_values, labels=labels, filename="s_values_long_no_safe", y_axis_label='Maximum Progress (m)',
                    crashes=crashes)
