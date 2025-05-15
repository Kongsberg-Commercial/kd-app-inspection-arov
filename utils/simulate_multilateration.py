import numpy as np
import time
import matplotlib.pyplot as plt
from numpy.linalg import norm
from scipy.optimize import least_squares
from scipy.stats import gaussian_kde      

def initial_guess_for_xy(positions, distances, fixed_z):
    N = len(positions)
    r = []
    for pos, d in zip(positions, distances):
        vertical_offset = fixed_z - pos[2]
        horizontal = np.sqrt(np.abs(d**2 - vertical_offset**2))
        r.append(horizontal)
    r = np.array(r)

    x1, y1 = positions[0, 0], positions[0, 1]
    r1 = r[0]

    A, b = [], []
    for i in range(1, N):
        xi, yi = positions[i, 0], positions[i, 1]
        ri = r[i]
        A.append([2 * (xi - x1), 2 * (yi - y1)])
        b.append(xi**2 - x1**2 + yi**2 - y1**2 + r1**2 - ri**2)

    A = np.array(A)
    b = np.array(b)
    guess_xy, *_ = np.linalg.lstsq(A, b, rcond=None)
    return guess_xy


def multilaterate(positions, distances, fixed_z):
    def residual_func(point, beacon_positions, measured_distances, fz):
        return [
            np.sqrt((point[0] - pos[0])**2 +
                    (point[1] - pos[1])**2 +
                    (fz - pos[2])**2) - d
            for pos, d in zip(beacon_positions, measured_distances)
        ]

    guess_xy = initial_guess_for_xy(positions, distances, fixed_z)
    result = least_squares(residual_func,
                           x0=np.array([guess_xy[0], guess_xy[1]]),
                           args=(positions, distances, fixed_z))
    return np.array([result.x[0], result.x[1], fixed_z])

class ROV:
    def __init__(self, center_x, center_y, center_z, radius=10.0, speed=0.025):
        self.cx, self.cy, self.cz = center_x, center_y, center_z
        self.radius = radius
        self.speed = speed
        self.angle = 0.0
        self.x = self.cx + self.radius * np.cos(self.angle)
        self.y = self.cy + self.radius * np.sin(self.angle)
        self.z = self.cz

    def move(self):
        self.angle += self.speed
        self.x = self.cx + self.radius * np.cos(self.angle)
        self.y = self.cy + self.radius * np.sin(self.angle)
        self.z = self.cz + 1.0 * np.sin(self.angle * 0.5)

    def get_pos(self):
        return np.array([self.x, self.y, self.z])


class Transponder:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z
        self.distance = 0.0

    def update_distance(self, rov):
        true_dist = norm(rov.get_pos() - self.get_pos())
        noise = np.random.rand() * 0.15
        self.distance = true_dist + noise

    def get_pos(self):
        return np.array([self.x, self.y, self.z])



def main():
    np.set_printoptions(precision=3, suppress=True)

    iterations = 100000    # 100k iterations for good sampling set
    np.random.seed(1)

    rov = ROV(center_x=30, center_y=80, center_z=-8,
              radius=10.0, speed=0.025)

    t1 = Transponder(30, 30, -1)
    t2 = Transponder(-4, 1, -1)
    t3 = Transponder(1, 1, -1)
    t4 = Transponder(5, 1, -1)
    transponders = [t1, t2, t3, t4]

    true_positions, estimated_positions, errors = [], [], []

    for _ in range(iterations):
        rov.move()
        for t in transponders:
            t.update_distance(rov)

        positions = np.array([t.get_pos() for t in transponders])
        distances = np.array([t.distance for t in transponders])
        fixed_z = rov.get_pos()[2] + np.random.rand() * 0.15

        est = multilaterate(positions, distances, fixed_z)
        error = norm(rov.get_pos() - est)

        true_positions.append(rov.get_pos())
        estimated_positions.append(est)
        errors.append(error)

    errors = np.array(errors)
    true_positions = np.array(true_positions)
    estimated_positions = np.array(estimated_positions)

    plt.figure(figsize=(8, 4))
    plt.hist(errors, bins="auto", density=True, alpha=0.6, label="Histogram")
    x_grid = np.linspace(errors.min(), errors.max(), 300)
    plt.plot(x_grid, gaussian_kde(errors)(x_grid), label="KDE")
    plt.xlabel("3-D error (m)")
    plt.ylabel("Probability density")
    plt.title(f"Error distribution after {iterations} iterations")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
