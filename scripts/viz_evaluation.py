"""Visualize the exploration results"""
import bisect
from dataclasses import dataclass, field
import json
from pathlib import Path
import matplotlib.pyplot as plt


def find_closest_index(a, x):
    """https://stackoverflow.com/a/56335408/9553849"""
    i = bisect.bisect_left(a, x)
    if i >= len(a):
        i = len(a) - 1
    elif i and a[i] - x > x - a[i - 1]:
        i = i - 1
    return (i, a[i])


@dataclass
class LogData:
    """Data read from log file"""
    filename: Path
    timestamps: list[float] = field(default_factory=list)
    area_pct: list[float] = field(default_factory=list)
    area_m2: list[float] = field(default_factory=list)
    paths: dict[str, list[float]] = field(default_factory=dict)
    total_path: list[float] = field(default_factory=list)

    @classmethod
    def from_log_file(cls, log_file: str):
        """Read the log file"""
        log_data = cls(log_file)
        with open(log_file, "r", encoding='utf-8') as f:
            for line in f.readlines():
                ts, area, unknown, free, occupied, * \
                    path_tokens = line.split(" ")
                log_data.append_path(path_tokens)
                log_data.timestamps.append(float(ts))
                log_data.area_pct.append(100 * (int(free) + int(occupied)) /
                                         (int(unknown) + int(free) + int(occupied)))
                log_data.area_m2.append(float(area))
        return log_data

    def append_path(self, tokens: list[str]) -> None:
        """Append path to paths dict"""
        dict_read = json.loads(' '.join(tokens).replace("'", '"'))
        total_path = 0
        for k, v in dict_read.items():
            if k not in self.paths:
                self.paths[k] = []
            self.paths[k].append(v)
            total_path += v
        self.total_path.append(total_path)

    def __str__(self):
        """Print stats"""
        text = f"{self.filename.stem}\n"
        text += f"{self.timestamps[-1]:8.2f}s "
        text += f"{self.area_m2[-1]:8.2f}m^2 ({self.area_pct[-1]:5.2f}%) "
        text += f"{self.total_path[-1]:8.2f}m\n"
        return text

    def stats(self, pct_step: float = 25.0):
        """Print stats"""
        text = "  %      Time[s]   Area[m2]    Path[m]\n"
        text += "--------------------------------------\n"
        pct = 0.0
        while pct <= 100.0:
            i, _ = find_closest_index(self.area_pct, pct)
            text += f"{pct:3.0f}% {self.timestamps[i]:10.2f} "
            text += f"{self.area_m2[i]:10.2f} {self.total_path[i]:10.2f}\n"
            pct += pct_step

        return text


def plot_area(data: LogData, fig: plt.Figure = None) -> plt.Figure:
    """Plot area graph"""
    if fig is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Exploration results')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('area (%)')
        ax.grid()

        ax2 = ax.twinx()
        ax2.set_ylabel('area (m^2)')
    else:
        ax = fig.axes[0]
        ax2 = fig.axes[1]

    ax.plot(data.timestamps, data.area_pct, label=data.filename.stem)
    ax2.plot(data.timestamps, data.area_m2)

    fig.savefig("/tmp/area.png")
    ax.legend()
    return fig


def plot_total_path(data: LogData, fig: plt.Figure = None) -> plt.Figure:
    """Plot paths"""
    if fig is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title('Path length')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('path length (m)')
        ax.grid()
    else:
        ax = fig.axes[0]

    ax.plot(data.timestamps, data.total_path, label=data.filename.stem)

    ax.legend()
    fig.savefig("/tmp/total_path.png")
    return fig


def plot_path(data: LogData):
    """Plot paths"""
    fig, ax = plt.subplots()
    for k, v in data.paths.items():
        # FIXME: ts and v not same length
        ax.plot(data.timestamps[1:], v, label=k)
    ax.set_title(f'Path length {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('path length (m)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/path_{data.filename.stem}.png")
    return fig


def main(log_file: str):
    """Main function"""
    if Path(log_file).is_dir():
        log_files = Path(log_file).glob("*.log")
    elif Path(log_file).is_file():
        log_files = [Path(log_file)]
    else:
        raise FileNotFoundError(f"File not found: {log_file}")

    fig, fig2 = None, None
    for log in log_files:
        data = LogData.from_log_file(log)

        print(data)
        fig = plot_area(data, fig)
        fig2 = plot_total_path(data, fig2)

        # plot_path(data)
        # print(data.stats(25.0))
    plt.show()


if __name__ == "__main__":
    main('logs/')
    # main('logs/3drones_exploration_20231113_125522.log')
