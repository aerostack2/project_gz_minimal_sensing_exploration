"""Visualize the exploration results"""
import matplotlib.pyplot as plt


def read_log(log_file: str) -> tuple[list[float], list[float]]:
    """Read the log file"""
    with open(log_file, "r", encoding='utf-8') as f:
        timestamp_0, width, height, resolution = f.readline().split(" ")
        data = f.readlines()
    total = int(width) * int(height)
    xdata = []  # time (s)
    ydata = []  # area explored (%)
    ydata_m2 = []  # area explored (m^2)
    for line in data:
        timestamp, _, unknown, *_ = line.split(" ")
        known = total - int(unknown[:-1])
        xdata.append(float(timestamp) - float(timestamp_0))
        ydata.append(100 * known / total)
        ydata_m2.append(known * float(resolution) * float(resolution))

    return xdata, ydata, ydata_m2


def plot_log(log_file):
    """Plot the log file"""
    # Data for plotting
    t, s, s2 = read_log(log_file)

    fig, ax = plt.subplots()
    ax.set_title('Exploration results')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('area (%)')
    ax.grid()
    ax.plot(t, s)

    ax2 = ax.twinx()
    ax2.set_ylabel('area (m^2)')
    ax2.plot(t, s2)

    # fig.savefig("test.png")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_log('test.log')
