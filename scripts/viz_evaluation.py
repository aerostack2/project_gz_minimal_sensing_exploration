"""Visualize the exploration results"""
import matplotlib.pyplot as plt


# TODO: Probably better to parse with json
def append_path(paths: dict[str, list[float]], tokens: list[str]) -> None:
    """Append path to paths dict"""
    if len(tokens) < 2:
        return
    for i in range(0, len(tokens), 2):
        if str(tokens[i][2:-2]) not in paths:
            paths[str(tokens[i][2:-2])] = []
        paths[str(tokens[i][2:-2])].append(float(tokens[i+1].strip()[:-1]))


def read_log(log_file: str) -> tuple[list[float], list[float]]:
    """Read the log file"""
    xdata = []  # time (s)
    ydata = []  # area explored (%)
    ydata_m2 = []  # area explored (m^2)
    paths: dict[str, list[float]] = {}
    with open(log_file, "r", encoding='utf-8') as f:
        for line in f.readlines():
            ts, area, unknown, free, occupied, *path_tokens = line.split(" ")
            append_path(paths, path_tokens)
            xdata.append(float(ts))
            ydata.append(100 * (int(free) + int(occupied)) /
                         (int(unknown) + int(free) + int(occupied)))
            ydata_m2.append(float(area))

    return xdata, ydata, ydata_m2, paths


def plot_area(ts, area_pct, area_m2):
    """Plot area graph"""
    fig, ax = plt.subplots()
    ax.set_title('Exploration results')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('area (%)')
    ax.grid()
    ax.plot(ts, area_pct)

    ax2 = ax.twinx()
    ax2.set_ylabel('area (m^2)')
    ax2.plot(ts, area_m2)

    fig.savefig("/tmp/area.png")
    fig.tight_layout()
    plt.show()


def plot_path(ts, paths):
    """Plot paths"""
    for k, v in paths.items():
        # FIXME: ts and v not same length
        plt.plot(ts[1:], v, label=k)
    plt.title('Path length')
    plt.xlabel('time (s)')
    plt.ylabel('path length (m)')
    plt.legend()
    plt.grid()
    plt.savefig("/tmp/path.png")
    plt.show()


def main(log_file):
    """Main function"""
    ts, area_pct, area_m2, paths = read_log(log_file)

    plot_area(ts, area_pct, area_m2)
    plot_path(ts, paths)


if __name__ == "__main__":
    main('logs/3drones_full_exploration_20231107_162109.log')
