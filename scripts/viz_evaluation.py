"""Visualize the exploration results"""
import matplotlib.pyplot as plt


def read_log(log_file: str) -> tuple[list[float], list[float]]:
    """Read the log file"""
    with open(log_file, "r", encoding='utf-8') as f:
        timestamp_0, width, height, resolution = f.readline().split(" ")
        data = f.readlines()
    total = int(width) * int(height)
    x = []
    y = []
    for line in data:
        timestamp, _, unknown, *_ = line.split(" ")
        known = total - int(unknown[:-1])
        x.append(float(timestamp) - float(timestamp_0))
        y.append(100 * known / total)

    return x, y


def plot_log(log_file):
    """Plot the log file"""
    # Data for plotting
    t, s = read_log(log_file)

    fig, ax = plt.subplots()
    ax.plot(t, s)

    ax.set(xlabel='time (s)', ylabel='area (%)',
           title='Exploration results')
    ax.grid()

    # fig.savefig("test.png")
    plt.show()


if __name__ == "__main__":
    plot_log('test.log')
