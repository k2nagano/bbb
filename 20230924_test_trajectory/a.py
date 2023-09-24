import numpy as np
from trajectory import *
from matplotlib import pylab as plt


def main():
    interpolation_method = "minjerk"
    act_pos = [1.0, 2.0]
    duration = max(np.floor(np.abs(np.max(act_pos)) * 2.5), 2.0)
    sampling = 0.01
    print(f"{duration} sec")
    ts = np.arange(0.0, duration, sampling)
    waypg = WayPointGenerator(
        sampling,
        start_point=act_pos,
        dimensions=2,
        interpolation_method=interpolation_method,
    )
    waypg.append_way_point([0.0, 0.0], duration * 0.9)
    waypg.append_way_point([0.0, 0.0], duration * 0.1)
    waypg.print_settings()

    ys_waypg = []
    for _ in enumerate(ts):
        ys_waypg.append(waypg.generate_command())
    ys = np.array(ys_waypg).T
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(ts, ys[0], label="node1")
    ax.plot(ts, ys[1], label="node2")
    ax.legend()
    ax.grid()
    plt.show()


if __name__ == "__main__":
    main()
