"""
plot_results.py
"""

from dataclasses import dataclass, field
from typing import Any
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from viz_evaluation import LogData, plot_area, plot_path, plot_total_path, plot_area_with_error
from overlap import total_overlap_ratio_logdata, overlap_ratio
from world_visualizer import parse_json, parse_xml, WorldFigure

Swarm = dict[str, Any]


@dataclass
class Stats:
    """ Stats """
    ts_mean: float = 0
    ts_std: float = 0
    area_pct_mean: float = 0
    area_pct_std: float = 0
    area_per_drone_mean: Swarm = 0  # Per drone in %
    area_per_drone_std: Swarm = 0
    total_area_per_drone_mean: float = 0  # Average drone area in %
    total_area_per_drone_std: float = 0
    path_length_mean: float = 0
    path_length_std: float = 0
    plength_per_drone_mean: Swarm = 0  # Per drone
    plength_per_drone_std: Swarm = 0
    total_plength_per_drone_mean: float = 0  # Average drone path length
    total_plength_per_drone_std: float = 0
    overlap_mean: float = 0
    overlap_std: float = 0

    def __str__(self) -> str:
        area_x_drone = ""
        for k, v in self.area_per_drone_mean.items():
            area_x_drone += f"{k}: {v:.2f} ± {self.area_per_drone_std[k]:.2f} "
        area_x_drone += f"({self.total_area_per_drone_mean:.2f} ± " + \
            f"{self.total_area_per_drone_std:.2f})"

        pl_x_drone = ""
        for k, v in self.plength_per_drone_mean.items():
            pl_x_drone += f"{k}: {v:.2f} ± {self.plength_per_drone_std[k]:.2f} "
        pl_x_drone += f"({self.total_plength_per_drone_mean:.2f} ± " + \
            f"{self.total_plength_per_drone_std:.2f})"
        return (f"Timestamps: {self.ts_mean:.2f} ± {self.ts_std:.2f}\n"
                f"Area: {self.area_pct_mean:.2f} ± {self.area_pct_std:.2f}\n"
                f"Path length: {self.path_length_mean:.2f} ±" +
                f"{self.path_length_std:.2f}\n"
                f"Area per drone: {area_x_drone}\n"
                f"Drone path length: {pl_x_drone}\n"
                f"Overlap ratio: {self.overlap_mean:.2f} ±" +
                f" {self.overlap_std:.2f}\n")


@dataclass
class Experiment:
    """ Experiment. Contains multiple rosbags for the same experiment. """
    name: str
    bags: list[str]

    # POST INIT
    log_datas: dict[str, LogData] = field(init=False)
    ts: str = field(init=False)

    def __post_init__(self):
        experiments: dict[str, LogData] = {}
        ts = self.bags[0]

        for rosbag in self.bags:
            data = LogData.from_rosbag(Path(rosbag))
            experiments[rosbag] = data
            ts = rosbag if len(data.timestamps) > len(
                experiments[ts].timestamps) else ts

        self.log_datas = experiments
        self.ts = ts

    def __repr__(self) -> str:
        return f"Experiment {self.name} with {len(self.bags)} rosbags"

    def print_data_info(self):
        """ Print data info """
        print(f"\nData Info for {self.name}")
        for data in self.log_datas.values():
            print(data)

    def print_stats(self):
        """ Print stats """
        print(f"\nStats for {self.name}")
        print(self.stats)

    @property
    def stats(self):
        """ Statistics """
        ts, area_pct, path_length, overlap_r = [], [], [], []
        iarea, ipath = [], []
        drone_areas = {}
        drone_paths = {}

        for v in self.log_datas.values():
            ts.append(v.timestamps[-1])
            area_pct.append(v.area_pct[-1])
            path_length.append(v.total_path[-1])
            for k, v2 in v.paths.items():
                ipath.append(v2[-1])
                try:
                    drone_paths[k] += v2[-1:]
                except KeyError:
                    drone_paths[k] = v2[-1:]

            for k in v.grids.keys():
                iarea.append(overlap_ratio(v.grids[k][0]))
                try:
                    drone_areas[k] += [overlap_ratio(v.grids[k][0])]
                except KeyError:
                    drone_areas[k] = [overlap_ratio(v.grids[k][0])]
            overlap_r.append(total_overlap_ratio_logdata(v))

        return Stats(np.mean(ts), np.std(ts),
                     np.mean(area_pct), np.std(area_pct),
                     {k: np.mean(v) for k, v in drone_areas.items()},
                     {k: np.std(v) for k, v in drone_areas.items()},
                     np.mean(iarea), np.std(iarea),
                     np.mean(path_length), np.std(path_length),
                     {k: np.mean(v) for k, v in drone_paths.items()},
                     {k: np.std(v) for k, v in drone_paths.items()},
                     np.mean(ipath), np.std(ipath),
                     np.mean(overlap_r), np.std(overlap_r))

    def plot_drones_path(self):
        """ Plot path """
        fig = None

        drones_path_length: dict[str, list] = {}

        for k in self.log_datas[self.ts].paths.keys():
            drones_path_length[k] = None

        # fig = plot_path(self.log_datas[self.ts])
        for data in self.log_datas.values():
            for k, v in data.paths.items():
                tail = [v[-1]] * \
                    (len(self.log_datas[self.ts].paths[k]) - len(v))
                vaux = np.array(v + tail, dtype=np.float64)
                if drones_path_length[k] is None:
                    drones_path_length[k] = vaux
                else:
                    drones_path_length[k] = np.vstack(
                        [drones_path_length[k], vaux])

        ts = np.array(self.log_datas[self.ts].timestamps, dtype=np.float64)
        for k, v in drones_path_length.items():
            mean = np.mean(v, axis=0, dtype=np.float64)
            std = np.std(v, axis=0, dtype=np.float64)
            fig = plot_area_with_error(ts, mean, std, k, fig)
            print(k, mean[-1], std[-1])

        # mean = np.mean(drones_path_length, axis=0, dtype=np.float64)
        # std = np.std(drones_path_length, axis=0, dtype=np.float64)
        # return plot_area_with_error(ts, mean, std, self.name, fig)
        # fig = plot_path(data)
        plt.show()

    def plot_area(self, show=True):
        """ Plot area """
        fig = None
        for data in self.log_datas.values():
            fig = plot_area(data, fig)

        if show:
            plt.show()

    def plot_total_path(self, show=True):
        """ Plot total path """
        fig = None
        for data in self.log_datas.values():
            fig = plot_total_path(data, fig)

        if show:
            plt.show()

    def plot_stats(self):
        """ Plot stats """
        self.print_data_info()
        self.plot_area(False)
        self.plot_total_path(False)
        plt.show()

    def plot_mean_area(self, fig=None):
        """ Plot area figure with error """
        m_data = None
        for data in self.log_datas.values():
            tail = [data.area_pct[-1]] * \
                (len(self.log_datas[self.ts].area_pct) - len(data.area_pct))
            vaux = np.array(data.area_pct + tail, dtype=np.float64)

            m_data = vaux if m_data is None else np.vstack([m_data, vaux])

        ts = np.array(self.log_datas[self.ts].timestamps, dtype=np.float64)
        mean = np.mean(m_data, axis=0, dtype=np.float64)
        std = np.std(m_data, axis=0, dtype=np.float64)
        return plot_area_with_error(ts, mean, std, self.name, fig)


ONE = [
    "rosbags/1_low_1",
    "rosbags/1_low_2",
    "rosbags/1_low_3",
    "rosbags/1_low_4",
    # "rosbags/1_low_5",  # 85%
    "rosbags/exploration_20240409_175107",
    "rosbags/exploration_20240409_181637",
    "rosbags/exploration_20240409_184228",
    "rosbags/exploration_20240409_190723",
    "rosbags/exploration_20240409_193235",
    "rosbags/exploration_20240409_195649",
]
TWO = [
    "rosbags/exploration_20240405_190136",
    "rosbags/exploration_20240405_192513",
    "rosbags/exploration_20240405_194124",
    "rosbags/exploration_20240405_195752",
    "rosbags/exploration_20240405_201329",
    "rosbags/exploration_20240405_202944",
    "rosbags/exploration_20240405_204558",
    "rosbags/exploration_20240405_210227",
    "rosbags/exploration_20240405_211939",
    "rosbags/exploration_20240405_213620",
]
THREE = [
    "rosbags/exploration_20240405_160245",
    "rosbags/exploration_20240405_162030",
    "rosbags/exploration_20240405_163751",
    "rosbags/exploration_20240405_165431",
    "rosbags/exploration_20240405_171250",
    "rosbags/exploration_20240405_173036",
    "rosbags/exploration_20240405_174803",
    "rosbags/exploration_20240405_180520",
    "rosbags/exploration_20240405_182236",
    "rosbags/exploration_20240405_184009",
]
FIVE = [
    # "rosbags/exploration_20240407_010903",  # BAD SIM
    # "rosbags/exploration_20240407_192213",  # NO TIMEOUT
    "rosbags/exploration_20240407_211922",
    "rosbags/exploration_20240407_214447",
    "rosbags/exploration_20240407_220707",
    "rosbags/exploration_20240407_222933",
    "rosbags/exploration_20240407_225201",
    # "rosbags/exploration_20240407_231205",  # crash?
    "rosbags/exploration_20240407_232853",
    # "rosbags/exploration_20240407_235122",  # crash?
    "rosbags/exploration_20240408_000421",
    "rosbags/exploration_20240408_002857",
    "rosbags/exploration_20240408_005107",
    "rosbags/exploration_20240409_143926"
]
SEVEN = [
    "rosbags/exploration_20240408_231627",
    "rosbags/exploration_20240408_234142",
    "rosbags/exploration_20240409_011435",
    # "rosbags/exploration_20240409_014030",  # CRASH
    # "rosbags/exploration_20240409_015429",  # CRASH
    "rosbags/exploration_20240409_021336",
    # "rosbags/exploration_20240409_023831",  # CRASH
    "rosbags/exploration_20240409_025029",
    "rosbags/exploration_20240409_031405",
    "rosbags/exploration_20240409_034244",
    "rosbags/exploration_20240409_041039",  # CRASH AL FINAL, OKEY
    # "rosbags/exploration_20240409_083831",  # BAD
    "rosbags/exploration_20240409_093958",
    "rosbags/exploration_20240409_100535",
]

ONE_MID = [
    "rosbags/1_medium_1",
    "rosbags/1_medium_2",
    "rosbags/1_medium_4",
    "rosbags/1_medium_5",
    "rosbags/1_medium_6",
    "rosbags/1_medium_8",
    "rosbags/1_medium_9",
    "rosbags/1_medium_10",
    "rosbags/1_medium_11",
    "rosbags/exploration_20240413_170903",
]
TWO_MID = [
    "rosbags/exploration_20240411_000148",
    "rosbags/exploration_20240411_002805",
    "rosbags/exploration_20240411_005459",
    "rosbags/exploration_20240411_015048",
    "rosbags/exploration_20240411_021729",
    "rosbags/exploration_20240411_024327",
    "rosbags/exploration_20240411_041025",
    "rosbags/exploration_20240411_043658",
    "rosbags/exploration_20240413_173658",
    "rosbags/exploration_20240413_180417",
]
THREE_MID = [
    "rosbags/exploration_20240411_080853",
    "rosbags/exploration_20240411_083624",
    "rosbags/exploration_20240411_091413",
    "rosbags/exploration_20240411_101108",
    "rosbags/exploration_20240411_103724",
    "rosbags/exploration_20240411_114006",
    "rosbags/exploration_20240411_120748",
    "rosbags/exploration_20240411_123444",
    "rosbags/exploration_20240411_130224",
    "rosbags/exploration_20240413_185930",
]
FIVE_MID = [
    "rosbags/exploration_20240411_141840",
    "rosbags/exploration_20240411_145306",
    "rosbags/exploration_20240411_152810",
    "rosbags/exploration_20240411_152810",
    "rosbags/exploration_20240411_160443",
    "rosbags/exploration_20240411_163912",
    "rosbags/exploration_20240411_173215",
    "rosbags/exploration_20240411_183330",
    "rosbags/exploration_20240411_190656",
    "rosbags/exploration_20240413_200207",
    "rosbags/exploration_20240413_192624",
]
SEVEN_MID = [
    "rosbags/exploration_20240415_090414",
    "rosbags/exploration_20240415_102927",
    "rosbags/exploration_20240415_120547",
    "rosbags/exploration_20240415_125001",
    "rosbags/exploration_20240415_144352",
]

ONE_HIGH = [
    "rosbags/exploration_20240415_163815",  # 96%
    "rosbags/exploration_20240415_180031",  # 95%
    "rosbags/exploration_20240415_203711",  # 95%
    # "rosbags/exploration_20240415_220110",  # 90%
    # "rosbags/exploration_20240415_232708", # 75%
    "rosbags/exploration_20240416_005354",  # 96%
    # "rosbags/exploration_20240416_021735",  # 87%
    # "rosbags/exploration_20240416_034003",  # 90%
    # "rosbags/exploration_20240416_050314",  # BAD
    # "rosbags/exploration_20240416_062710",  # BAD
    "rosbags/exploration_20240417_181209",  # 94%
    "rosbags/exploration_20240417_201611",  # 94%
    "rosbags/exploration_20240417_222045",  # 94%
    "rosbags/exploration_20240418_002449",  # 96%
    "rosbags/exploration_20240418_022853",  # 95%
    "rosbags/exploration_20240418_042803",  # 92%
]
TWO_HIGH = [
    # "rosbags/exploration_20240416_115855",  # STUCK --> BAD
    "rosbags/exploration_20240416_150508",  # 95%
    "rosbags/exploration_20240416_164023",  # 95%
    "rosbags/exploration_20240416_181334",  # 95%
    "rosbags/exploration_20240416_193912",  # 96%
    "rosbags/exploration_20240416_211709",  # 96%
    "rosbags/exploration_20240416_224632",  # 97%
    "rosbags/exploration_20240417_001443",  # 96%
    "rosbags/exploration_20240417_020148",  # 95%
    "rosbags/exploration_20240417_035120",  # 97%
    "rosbags/exploration_20240417_052505",  # 96%
]
THREE_HIGH = [
    "rosbags/exploration_20240417_065805",  # 95%
    "rosbags/exploration_20240417_082748",  # 94%
    "rosbags/exploration_20240417_095525",  # 96%
    "rosbags/exploration_20240417_111921",  # 93%
    "rosbags/exploration_20240417_141311",  # 95%
    "rosbags/exploration_20240418_063208",  # 95%
    "rosbags/exploration_20240418_075436",  # 95%
    "rosbags/exploration_20240418_092316",  # 96%
    "rosbags/exploration_20240418_105430",  # 94%
    "rosbags/exploration_20240418_122115",  # 94%
]
FIVE_HIGH = [
    # "rosbags/exploration_20240418_183518",  # 92%
    # "rosbags/exploration_20240418_190148",  # 91%
    "rosbags/exploration_20240418_193133",  # 96%
    "rosbags/exploration_20240418_201911",  # 97%
    "rosbags/exploration_20240418_205429",  # 97%
    "rosbags/exploration_20240418_212849",  # 96%
    "rosbags/exploration_20240418_222256",  # 96%
    "rosbags/exploration_20240418_225802",  # 95%
    # "rosbags/exploration_20240418_233152",  # 91%
    "rosbags/exploration_20240419_145543",  # 96%
    "rosbags/exploration_20240419_152629",  # 96%
    "rosbags/exploration_20240419_155814",  # 96%
    "rosbags/exploration_20240419_163203",  # 93%
]
SEVEN_HIGH = [
    "rosbags/exploration_20240419_010504",  # 96%
    "rosbags/exploration_20240419_021015",  # 96%
    "rosbags/exploration_20240419_031114",  # 96%
    "rosbags/exploration_20240419_172416",  # 96%
    # "rosbags/exploration_20240419_183352",  # 92%
    # "rosbags/exploration_20240419_194630",  # 91%
    # "rosbags/exploration_20240419_201613",  # 95% BAD
    "rosbags/exploration_20240419_213418",  # 96%
    "rosbags/exploration_20240419_234717",  # 96%
    "rosbags/exploration_20240422_103636",  # 96%
    "rosbags/exploration_20240422_114034",  # 97%
    "rosbags/exploration_20240422_122438",  # 96%
    "rosbags/exploration_20240422_130010",  # 96%
]

ONE_HUGE = [
    # "rosbags/1_high1",
    # "rosbags/1_high2",
    "rosbags/exploration_20240409_231435",
    "rosbags/exploration_20240410_005826",
    # "rosbags/exploration_20240410_024553",
    # "rosbags/exploration_20240410_054808",
]

COLORS = {
    '/drone0': ('r', '#ff8989'),
    '/drone1': ('g', '#96ff89'),
    '/drone2': ('b', '#89cfff'),
    '/drone3': ('y', '#ffff89'),
    '/drone4': ('m', '#eb89ff'),
    '/drone5': ('c', '#89ffea'),
    '/drone6': ('k', '#cfcfcf'),
}


def zenithal_view(rosbag: str, json_filename: str):
    """ Five drones with zenithal view """
    if not Path(json_filename).exists():
        raise FileNotFoundError(f"{json_filename} not found")
    world_name, drones = parse_json(json_filename)

    xml_filename = Path(json_filename).parent / f"{world_name}.sdf"
    poles = parse_xml(xml_filename)

    data = LogData.from_rosbag(Path(rosbag))

    fig = WorldFigure("World")
    fig.draw_obstacles(poles, color='ko')
    for drone, pose, path in zip(drones.keys(), drones.values(), data.poses.values()):
        fig.draw_drones({drone: pose}, COLORS['/' + drone][0] + 'D')
        fig.draw_paths({drone: path}, COLORS['/' + drone][0])
    fig.draw_grid_maps(data.grids, colors=[
                       COLORS[drone][1] for drone in data.grids.keys()])
    fig.main_plot.legend()
    fig.show()


def main_plot(bags: list[str], label):
    """ Main plot """
    exps = Experiment(label, bags)

    exps.plot_stats()

    exps.print_stats()

    exps.plot_mean_area()
    plt.show()

# JUST SHORTCUTS FOR PLOTTING


def one_low_plot():
    """ Experiment with one drone in low obstacle density world """
    main_plot(ONE, "1 drone")


def one_mid_plot():
    """ Experiment with one drone in medium obstacle density world """
    main_plot(ONE_MID, "1drones")


def one_high_plot():
    """ Experiment with one drone in high obstacle density world """
    main_plot(ONE_HIGH, "1drones")


def two_low_plot():
    """ Experiment with two drones in low obstacle density world """
    main_plot(TWO, "2drones")


def three_low_plot():
    """ Experiment with one drone in low obstacle density world """
    main_plot(THREE, "3drones")


def five_low_plots():
    """ Experiment with five drones in low obstacle density world """
    main_plot(FIVE, "5drones")


def seven_low_plots():
    """ Experiment with seven drones in low obstacle density world """
    main_plot(SEVEN, "7drones")


def comparison_low_plot():
    """ Plot experiments in low density world"""
    low_experiments = [
        Experiment("1 drone", ONE),
        Experiment("2 drones", TWO),
        Experiment("3 drones", THREE),
        Experiment("5 drones", FIVE),
        Experiment("7 drones", SEVEN),
    ]

    fig = None
    for exp in low_experiments:
        fig = exp.plot_mean_area(fig)

        # exp.plot_stats(value)
        exp.print_stats()
    plt.show()


def comparison_one_plot():
    """ Plot one drone in low and medium density world"""
    one_experiments = [
        Experiment("low", ONE),
        Experiment("medium", ONE_MID),
        Experiment("high", ONE_HIGH),
    ]

    fig = None
    for exp in one_experiments:
        fig = exp.plot_mean_area(fig)

        # exp.plot_stats(value)
        exp.print_stats()
    plt.show()


def comparison_mid_plot():
    """ Plot one to seven drones in medium density world"""
    medium_experiments = [
        Experiment("1drones", ONE_MID),
        Experiment("2drones", TWO_MID),
        Experiment("3drones", THREE_MID),
        Experiment("5drones", FIVE_MID),
        Experiment("7drones", SEVEN_MID),
    ]

    fig = None
    for exp in medium_experiments:
        fig = exp.plot_mean_area(fig)

        # exp.plot_stats(value)
        exp.print_stats()
    plt.show()


def comparison_high_plot():
    """ Plot one to seven drones in medium density world"""
    high_experiments = [
        Experiment("1drones", ONE_HIGH),
        Experiment("2drones", TWO_HIGH),
        Experiment("3drones", THREE_HIGH),
        Experiment("5drones", FIVE_HIGH),
        Experiment("7drones", SEVEN_HIGH),
    ]

    fig = None
    for exp in high_experiments:
        fig = exp.plot_mean_area(fig)

        # exp.plot_stats(value)
        exp.print_stats()
    plt.show()


def comparison_three_plot():
    """ Plot one to seven drones in medium density world"""
    three_experiments = [
        Experiment("low", THREE),
        Experiment("medium", THREE_MID),
        Experiment("high", THREE_HIGH),
    ]

    fig = None
    for exp in three_experiments:
        fig = exp.plot_mean_area(fig)

        # exp.plot_stats(value)
        exp.print_stats()
    plt.show()


def plot_ts_error_bar():
    """ Plot error bar """

    low_experiments = [
        Experiment("1 drone", ONE),
        Experiment("2 drones", TWO),
        Experiment("3 drones", THREE),
        Experiment("5 drones", FIVE),
        Experiment("7 drones", SEVEN),
    ]

    medium_experiments = [
        Experiment("1 drone", ONE_MID),
        Experiment("2 drones", TWO_MID),
        Experiment("3 drones", THREE_MID),
        Experiment("5 drones", FIVE_MID),
        Experiment("7 drones", SEVEN_MID),
    ]

    high_experiments = [
        Experiment("1 drone", ONE_HIGH),
        Experiment("2 drones", TWO_HIGH),
        Experiment("3 drones", THREE_HIGH),
        Experiment("5 drones", FIVE_HIGH),
        Experiment("7 drones", SEVEN_HIGH),
    ]

    mmm = {
        "low": low_experiments,
        "medium": medium_experiments,
        "high": high_experiments
    }

    x = [1, 2, 3, 5, 7]
    for key, value in mmm.items():
        y = []
        e = []
        for exp in value:
            y.append(exp.stats.ts_mean)
            e.append(exp.stats.ts_std)

        plt.errorbar(x, y, e, marker='^', label=key)
        # plt.title("Timestamps")
        plt.xlabel("Drones")
        plt.ylabel("Time (s)")
        plt.legend()

    plt.show()

    for key, value in mmm.items():
        y = []
        e = []
        for exp in value:
            y.append(exp.stats.path_length_mean)
            e.append(exp.stats.path_length_std)

        plt.errorbar(x, y, e, linestyle='None', marker='^', label=key)
        plt.title("Path Length")
        plt.xlabel("Drones")
        plt.ylabel("Path Length (m)")
        plt.legend()

    plt.show()

    for key, value in mmm.items():
        y = []
        e = []
        for exp in value:
            y.append(exp.stats.area_pct_mean)
            e.append(exp.stats.area_pct_std)

        plt.errorbar(x, y, e, linestyle='None', marker='^', label=key)
        plt.title("Area")
        plt.xlabel("Drones")
        plt.ylabel("Area (%)")
        plt.legend()

    plt.show()

    for key, value in mmm.items():
        y = []
        e = []
        for exp in value:
            y.append(exp.stats.overlap_mean)
            e.append(exp.stats.overlap_std)

        plt.errorbar(x, y, e, linestyle='None', marker='^', label=key)
        plt.title("Overlap")
        plt.xlabel("Drones")
        plt.ylabel("Overlap (%)")
        plt.legend()

    plt.show()


def plot_drone_path_error_bar():
    """Plot drone path error bar"""

    low_experiments = [
        Experiment("1drones", ONE),
        Experiment("2drones", TWO),
        Experiment("3drones", THREE),
        Experiment("5drones", FIVE),
        Experiment("7drones", SEVEN),
    ]

    x = [1, 2, 3, 5, 7, 10]

    for exp in low_experiments:
        y = []
        e = []
        for k, v in exp.stats.plength_per_drone_mean.items():
            y.append(v)
            e.append(exp.stats.plength_per_drone_std[k])
        print(exp.stats.plength_per_drone_mean)
        print(exp.stats.plength_per_drone_std)
        print(exp.stats.total_plength_per_drone_mean)
        print(exp.stats.total_plength_per_drone_std)
        plt.errorbar(x[:len(y)], y, e, linestyle='None',
                     marker='^', label=exp.name)

    plt.title("Timestamps")
    plt.xlabel("Drones")
    plt.ylabel("Time (s)")
    plt.legend()

    plt.show()


def print_extended_stats_low_env():
    """Plot drone path error bar"""

    low_experiments = [
        Experiment("1drones", ONE),
        Experiment("2drones", TWO),
        Experiment("3drones", THREE),
        Experiment("5drones", FIVE),
        Experiment("7drones", SEVEN),
    ]

    for exp in low_experiments:
        print(exp.name)
        print(exp.stats)
        print("\n")


if __name__ == "__main__":
    # FIVE DRONES LOW ENVIRONMENT
    # zenithal_view('rosbags/exploration_20240409_143926',
    #               'assets/worlds/low5.json')

    # THREE DRONES LOW ENVIRONMENT FINAL HEURISTIC
    # zenithal_view("rosbags/exploration_20240405_160245",
    #               "assets/worlds/low3.json")

    # # THREE DRONES LOW ENVIRONMENT MAX DISTANCE
    # zenithal_view("rosbags/exploration_20240405_100734",
    #               "assets/worlds/low3.json")

    # THREE DRONES LOW ENVIRONMENT NEAREST FRONTIER
    # zenithal_view("rosbags/exploration_20240424_140241",
    #               "assets/worlds/low3.json")

    # # THREE DRONE 60 OBS NEAREST FRONTIER
    # zenithal_view("rosbags/exploration_20240320_151339",
    #               "assets/worlds/world60_three_drone.json")

    # HIGH ONE
    # zenithal_view("rosbags/exploration_20240409_231435",
    #               "assets/worlds/high2.json")

    # MEDIUM ONE
    # zenithal_view("rosbags/1_medium_1", "assets/worlds/medium1.json")

    # MEDIUM FIVE
    # zenithal_view(FIVE_MID[4], "assets/worlds/medium5.json")

    # HIGH 3
    # zenithal_view(THREE_HIGH[0], "assets/worlds/high3.json")

    # zenithal_view("rosbags/exploration_20240419_201613",
    #               "assets/worlds/high7.json")

    # one_low_plot()
    # one_mid_plot()
    # one_high_plot()
    # two_low_plot()
    # three_low_plot()
    # five_low_plots()
    # seven_low_plots()

    # comparison_low_plot()
    # comparison_mid_plot()
    # comparison_high_plot()

    # comparison_one_plot()
    # comparison_three_plot()

    plot_ts_error_bar()

    test = [

    ]
    # main_plot(FIVE_HIGH, "test")

    # exp = Experiment("test", THREE[:2])
    # print(exp.stats)

    # print_extended_stats_low_env()
