"""Visualize the exploration results"""
from dataclasses import dataclass, field
import json
import matplotlib.pyplot as plt


@dataclass
class LogData:
    """Data read from log file"""
    timestamps: list[float] = field(default_factory=list)
    area_pct: list[float] = field(default_factory=list)
    area_m2: list[float] = field(default_factory=list)
    paths: dict[str, list[float]] = field(default_factory=dict)

    @classmethod
    def from_log_file(cls, log_file: str):
        """Read the log file"""
        log_data = cls()
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
        for k, v in dict_read.items():
            if k not in self.paths:
                self.paths[k] = []
            self.paths[k].append(v)

    @property
    def total_path_length(self) -> float:
        """Total path length"""
        length = 0
        for v in self.paths.values():
            length += v[-1]
        return length

    def __str__(self):
        """Print stats"""
        return f"""Total time: {self.timestamps[-1]}s\
        Total area explored: {round(self.area_m2[-1], 2)}m^2 ({round(self.area_pct[-1], 2)}%)\
        Total path length: {round(self.total_path_length, 2)}m
        """


def plot_area(data: LogData):
    """Plot area graph"""
    fig, ax = plt.subplots()
    ax.set_title('Exploration results')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('area (%)')
    ax.grid()
    ax.plot(data.timestamps, data.area_pct)

    ax2 = ax.twinx()
    ax2.set_ylabel('area (m^2)')
    ax2.plot(data.timestamps, data.area_m2)

    fig.savefig("/tmp/area.png")
    fig.tight_layout()
    plt.show()


def plot_path(data: LogData):
    """Plot paths"""
    for k, v in data.paths.items():
        # FIXME: ts and v not same length
        plt.plot(data.timestamps[1:], v, label=k)
    plt.title('Path length')
    plt.xlabel('time (s)')
    plt.ylabel('path length (m)')
    plt.legend()
    plt.grid()
    plt.savefig("/tmp/path.png")
    plt.show()


def main(log_file):
    """Main function"""
    data = LogData.from_log_file(log_file)

    print(data)
    plot_area(data)
    plot_path(data)


if __name__ == "__main__":
    main('logs/3drones_1fail_full_exploration_20231110_143642.log')
