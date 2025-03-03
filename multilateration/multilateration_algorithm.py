import numpy as np
import time
from numpy.linalg import norm
from scipy.optimize import least_squares

# Helper function to convert from latlon coordinates to meters from a reference point (origo)
def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    R = 6371000  # Earth's radius in meters
    dlat = np.deg2rad(lat - origin_lat)
    dlon = np.deg2rad(lon - origin_lon)
    x = R * dlon * np.cos(np.deg2rad(origin_lat))
    y = R * dlat
    return x, y


# Helper function to convert from meters to latlon coordinates from a reference point (origo)
def meters_to_latlon(x, y, origin_lat, origin_lon):
    R = 6371000
    dlat = y / R
    dlon = x / (R * np.cos(np.deg2rad(origin_lat)))
    lat = origin_lat + np.rad2deg(dlat)
    lon = origin_lon + np.rad2deg(dlon)
    return lat, lon


# Helper function to get an initial estimate for the position using linear least squares
# This function closely follows the mathematics described in the bachelor thesis.
def initial_guess_for_xy(positions, distances, fixed_z):
    N = len(positions)
    r = []
    for pos, d in zip(positions, distances):
        vertical_offset = fixed_z - pos[2]
        horizontal = np.sqrt(np.abs(d**2 - vertical_offset**2))
        r.append(horizontal)
    r = np.array(r)
    
    # Using the first transponder as a reference
    x1, y1 = positions[0, 0], positions[0, 1]
    r1 = r[0]
    
    A = []
    b = []
    for i in range(1, N):
        xi, yi = positions[i, 0], positions[i, 1]
        ri = r[i]
        A.append([2*(xi - x1), 2*(yi - y1)])
        b.append(xi**2 - x1**2 + yi**2 - y1**2 + r1**2 - ri**2)
    
    A = np.array(A)
    b = np.array(b)
    guess_xy, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    return guess_xy


# Estimate the ROV's position using multilateration. 
def multilaterate(positions, distances, fixed_z):
    def residual_func(point, beacon_positions, measured_distances, fixed_z):
        return [
            np.sqrt((point[0] - pos[0])**2 + (point[1] - pos[1])**2 + (fixed_z - pos[2])**2) - d
            for pos, d in zip(beacon_positions, measured_distances)
        ]
    guess_xy = initial_guess_for_xy(positions, distances, fixed_z)
    initial_guess = np.array([guess_xy[0], guess_xy[1]])
    result = least_squares(residual_func, x0=initial_guess, args=(positions, distances, fixed_z))
    return np.array([result.x[0], result.x[1], fixed_z])


class ROV:
    def __init__(self, center_x, center_y, center_z, radius=10.0, speed=0.025):
        self.cx = center_x
        self.cy = center_y
        self.cz = center_z
        self.radius = radius
        self.angle = 0.0
        self.speed = speed
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
        self.x = x
        self.y = y
        self.z = z
        self.distance = 0.0

    def update_distance(self, rov):
        true_dist = norm(rov.get_pos() - self.get_pos())
        noise = np.random.rand() * 0.15
        self.distance = true_dist + noise

    def get_pos(self):
        return np.array([self.x, self.y, self.z])


def main():
    np.set_printoptions(precision=3, suppress=True)
    
    # Create an ROV moving along a circular path
    rov = ROV(center_x=30, center_y=80, center_z=-8, radius=10.0, speed=0.025)
    
    # Define transponder positions in lat–lon.
    transponder_latlon = [
        (59.428465173, 10.465137681),   # t1: the origin
        (59.428445150, 10.465241581),   # t2
        (59.428427280, 10.465334121),   # t3
        (59.428407132, 10.465439580),   # t4
    ]
    origin_lat, origin_lon = transponder_latlon[0]

    # Initialize the transponders with positions from latlon to meters. Setting a fixed depth of -1 meters.
    transponders = []
    for lat, lon in transponder_latlon:
        x, y = latlon_to_meters(lat, lon, origin_lat, origin_lon)
        transponders.append(Transponder(x, y, -1))
    
    # Cheesy hack to sole geometry issue when all transponders are on the exact same line 
    # This is quite important. The 10cm discrepency has a minimal effect on the inaccuracy
    transponders[1].x += 0.1
    transponders[1].y += 0.1

    # Main loop
    while True:
        rov.move()
        for t in transponders:
            t.update_distance(rov)
        
        positions = np.array([t.get_pos() for t in transponders])
        distances = np.array([t.distance for t in transponders])
        
        # Use the barometer (with noise) for the depth.
        fixed_z = rov.get_pos()[2] + np.random.rand() * 0.15
        
        est = multilaterate(positions, distances, fixed_z)
        true_pos = rov.get_pos()
        error = norm(true_pos - est)
        
        # Convert ROV positions from meters to lat–lon.
        true_latlon = meters_to_latlon(true_pos[0], true_pos[1], origin_lat, origin_lon)
        est_latlon = meters_to_latlon(est[0], est[1], origin_lat, origin_lon)
        
        print("Estimated position (meters):", est)
        print("True position (meters):     ", true_pos)
        print("Step error (meters):        ", error)
        print("Estimated position (lat, lon, z):", est_latlon[0], est_latlon[1], est[2])
        print("True position (lat, lon, z):     ", true_latlon[0], true_latlon[1], true_pos[2])
        print("-" * 95)
        time.sleep(0.15)


if __name__ == "__main__":
    main()