import numpy as np
import time
from numpy.linalg import norm
from scipy.optimize import least_squares

# ------------------------------------------------------------
# Conversion functions (lat/lon <-> meters using an origin)
# ------------------------------------------------------------
def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    R = 6371000  # Earth's radius in meters
    dlat = np.deg2rad(lat - origin_lat)
    dlon = np.deg2rad(lon - origin_lon)
    x = R * dlon * np.cos(np.deg2rad(origin_lat))
    y = R * dlat
    return x, y

def meters_to_latlon(x, y, origin_lat, origin_lon):
    R = 6371000
    dlat = y / R
    dlon = x / (R * np.cos(np.deg2rad(origin_lat)))
    lat = origin_lat + np.rad2deg(dlat)
    lon = origin_lon + np.rad2deg(dlon)
    return lat, lon

# ------------------------------------------------------------
# Multilateration function (ignoring z variation)
# ------------------------------------------------------------
def multilaterate(positions, distances, initial_guess=None):
    def residual_func(point, beacon_positions, measured_distances):
        return [
            np.linalg.norm(point - beacon_positions[i]) - measured_distances[i]
            for i in range(len(beacon_positions))
        ]
    
    if initial_guess is None:
        initial_guess = np.mean(positions, axis=0)
    
    result = least_squares(
        residual_func, 
        x0=initial_guess, 
        args=(positions, distances)
    )
    return result.x

# ------------------------------------------------------------
# ROV and Transponder classes
# ------------------------------------------------------------
class ROV:
    def __init__(self, center_x, center_y, center_z, radius=5.0, speed=0.1):
        self.cx = center_x
        self.cy = center_y
        self.cz = center_z
        
        self.radius = radius
        self.angle = 0.0
        self.speed = speed

        # Initial position (start at angle=0)
        self.x = self.cx + self.radius * np.cos(self.angle)
        self.y = self.cy + self.radius * np.sin(self.angle)
        self.z = self.cz

    def move(self):
        # Increase angle each step to move along the circle
        self.angle += self.speed
        
        # Circular path in xy
        self.x = self.cx + self.radius * np.cos(self.angle)
        self.y = self.cy + self.radius * np.sin(self.angle)
        
        # Up/down oscillation in z (which we won't use for multilateration)
        self.z = self.cz + 1.0 * np.sin(self.angle * 0.5)

    def get_pos(self):
        return np.array([self.x, self.y, self.z])

class Transponder:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.distance = 0.0

    def update_distance(self, rov_):
        # True distance plus a little noise.
        true_dist = norm(rov_.get_pos() - self.get_pos())
        noise = np.random.rand() * 0.15
        self.distance = true_dist + noise

    def get_true_distance(self, rov_):
        return norm(rov_.get_pos() - self.get_pos())

    def get_pos(self):
        return np.array([self.x, self.y, self.z])

# ------------------------------------------------------------
# Main function using lat/lon transponder logic.
# ------------------------------------------------------------
def main():
    np.set_printoptions(precision=3, suppress=True)

    # Initialize the ROV (positions in meters).
    rov = ROV(center_x=30, center_y=80, center_z=-15, radius=6.0, speed=0.05)

    # Define transponder positions in lat/lon.
    transponder_latlon = [
        (59.428465173, 10.465137681),   # t1 (used as origin)
        (59.428445150, 10.465241581),   # t2
        (59.428427280, 10.465334121),   # t3
        (59.428407132, 10.465439580),   # t4
    ]
    origin_lat, origin_lon = transponder_latlon[0]

    # Create transponder objects by converting lat/lon to meters.
    # We use a fixed z (here, -1 m).
    transponders = []
    for i, (lat, lon) in enumerate(transponder_latlon):
        z = -1
        x, y = latlon_to_meters(lat, lon, origin_lat, origin_lon)
        # Optional: adjust one transponder slightly to improve geometry.
        if i == 1:
            z = -5
        transponders.append(Transponder(x, y, z))

    error_accum = 0.0
    counter = 0

    while True:
        counter += 1

        # Move the ROV along its path.
        rov.move()

        # Update measured distances for each transponder.
        for t in transponders:
            t.update_distance(rov)

        # Prepare data for multilateration.
        positions = np.array([t.get_pos() for t in transponders])
        distances = np.array([t.distance for t in transponders])

        # Estimate the ROV's position (using full 3D vectors, but note that z is fixed).
        est = multilaterate(positions, distances)

        # Compute error.
        true_pos = rov.get_pos()
        step_error = norm(true_pos - est)
        error_accum += step_error
        avg_error = error_accum / counter

        # Convert true position and estimated horizontal position to lat/lon.
        # (We use only the x,y components; z is not converted.)
        true_lat, true_lon = meters_to_latlon(true_pos[0], true_pos[1], origin_lat, origin_lon)
        est_lat, est_lon = meters_to_latlon(est[0], est[1], origin_lat, origin_lon)

        # Print information.
        print("Estimated distances: ", distances)
        print("True distances:      ", [t.get_true_distance(rov) for t in transponders])
        print("Estimated point (m): ", est)
        print("True point (m):      ", true_pos)
        print("Step error (m):      ", step_error)
        print("Average error (m):   ", avg_error)
        print("Estimated position (lat, lon, z):", est_lat, est_lon, est[2])
        print("True position (lat, lon, z):     ", true_lat, true_lon, true_pos[2])
        print("-" * 55)
        
        time.sleep(0.15)

if __name__ == "__main__":
    main()
