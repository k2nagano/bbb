import numpy as np

from matplotlib import pylab as plt


class Interpolator(object):
    def __init__(self, t0=0, x0=0.0, v0=None, a0=None, t1=1, x1=1.0, v1=None, a1=None):
        self.a_mat = np.array([[t0, 1], [t1, 1]])
        self.b_vec = np.array([x0, x1])
        # solve A x = b
        self.x_vec = np.linalg.solve(self.a_mat, self.b_vec)
        self.t0 = t0
        self.t1 = t1

    def calc(self, t):
        tt = t
        if tt > self.t1:
            tt = self.t1
        elif tt < self.t0:
            tt = self.t0
        return np.polyval(self.x_vec, tt)


class Minjerk(Interpolator):
    def __init__(self, t0=0, x0=0.0, v0=0.0, a0=0.0, t1=1, x1=1.0, v1=0.0, a1=0.0):
        self.a_mat = np.array([[t0 ** 5, t0 ** 4, t0 ** 3, t0 ** 2, t0, 1],
                               [5 * t0 ** 4, 4 * t0 ** 3,
                                   3 * t0 ** 2, 2 * t0, 1, 0],
                               [20 * t0 ** 3, 12 * t0 ** 2, 6 * t0, 2, 0, 0],
                               [t1 ** 5, t1 ** 4, t1 ** 3, t1 ** 2, t1, 1],
                               [5 * t1 ** 4, 4 * t1 ** 3,
                                   3 * t1 ** 2, 2 * t1, 1, 0],
                               [20 * t1 ** 3, 12 * t1 ** 2, 6 * t1, 2, 0, 0]])
        self.b_vec = np.array([x0, v0, a0, x1, v1, a1])
        # solve A x = b
        self.x_vec = np.linalg.solve(self.a_mat, self.b_vec)
        self.t0 = t0
        self.t1 = t1


def create_interpolator(method, t0, x0, t1, x1):
    if method == "minjerk":
        return Minjerk(t0=t0, x0=x0, t1=t1, x1=x1)
    else:
        return Interpolator(t0=t0, x0=x0, t1=t1, x1=x1)


class WayPointGenerator(object):

    """ trapezoid command generator """

    def __init__(self, sampling, start_point, dimensions=1, interpolation_method="linear"):

        self._sampling = sampling  # [sec]
        self._interpolation_method = interpolation_method
        self._dimensions = dimensions
        self.reset(start_point)

    def get_num_of_dimensions(self):
        return self._dimensions

    def append_way_point(self, way_point, duration):
        if self.get_num_of_dimensions() != len(way_point):
            raise Exception(
                "Unmatched dimensions between interpolators and way_point")
        self._way_points.append(way_point)
        time_from_start = duration
        if len(self._durations) > 0:
            time_from_start += self._durations[-1]
        self._durations.append(time_from_start)
        self._durations_count.append(self._time_to_count(time_from_start))

    def _time_to_count(self, time):
        if time < self._sampling:
            time = self._sampling
        return int(time / self._sampling)

    def print_settings(self):
        print("sampling: {}[sec], interpolator_method: {}".format(
            self._sampling, self._interpolation_method))
        for i, wp in enumerate(self._way_points):
            print("way point index:{}, time_from_start:{}[sec], point:{}".format(
                i, self._durations[i], wp))

    def is_generated(self):
        return self._is_generate

    def reset(self, point):
        if self.get_num_of_dimensions() != len(point):
            raise Exception(
                "Unmatched dimensions between interpolators and reset_point")
        else:
            print("Reset WayPointGenerator...")
        self._current_point = point
        self._active_index = 0
        self._count = 0
        self._way_points = []
        self._durations = []
        self._durations_count = []
        self._interpolators = []
        self._is_generate = False

    def generate_command(self):
        """ generate command with interpolator """

        if len(self._way_points) == 0:
            # No waypoint
            return self._current_point

        self._is_generate = True
        command = []

        if self._count >= self._durations_count[-1]:
            self._is_generate = False
        elif self._count == 0 or self._count == self._durations_count[self._active_index]:
            print("[Update interpolator] count:{}".format(self._count))
            i0 = self._active_index
            i1 = self._active_index
            if self._count != 0:
                i1 += 1
            t0 = self._count
            t1 = self._durations_count[i1]
            if self._count == 0:
                x0 = self._current_point
            else:
                x0 = self._way_points[i0]
            x1 = self._way_points[i1]
            self._interpolators = []
            for i in range(self.get_num_of_dimensions()):
                self._interpolators.append(create_interpolator(
                    self._interpolation_method, t0, x0[i], t1, x1[i]))
            self._active_index = i1

        for i, interp in enumerate(self._interpolators):
            command.append(interp.calc(self._count))
        self._current_point = command
        self._count += 1
        return self._current_point


def main(args=None):
    interpolation_method = "minjerk"
    ts = np.linspace(0.0, 10.0, 100)
    waypg = WayPointGenerator(0.1, start_point=[
                              0.0, 0.0], dimensions=2, interpolation_method=interpolation_method)
    waypg.append_way_point([0.0, 0.0], 1.0)
    waypg.append_way_point([1.0, 2.0], 2.0)
    waypg.append_way_point([1.0, 2.0], 1.0)
    waypg.append_way_point([0.0, 0.0], 2.0)
    waypg.append_way_point([0.0, 0.0], 1.0)
    waypg.print_settings()

    ys_waypg = []
    for i, t in enumerate(ts):
        ys_waypg.append(waypg.generate_command())
    plt.plot(ts, ys_waypg, label=interpolation_method)
    plt.legend()
    plt.grid()
    plt.show()

    waypg.reset(ys_waypg[-1])
    waypg.generate_command()
    print("Generated:{}".format(waypg.is_generated()))


if __name__ == "__main__":
    main()
