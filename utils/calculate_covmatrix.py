import numpy as np

def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    R = 6371000
    dlat = np.deg2rad(lat - origin_lat)
    dlon = np.deg2rad(lon - origin_lon)
    x = R * dlon * np.cos(np.deg2rad(origin_lat))
    y = R * dlat
    return x, y

def parse_local(line):
    parts = line.strip().split(',')
    try:
        return [float(parts[2]), float(parts[3]), float(parts[4])]
    except (IndexError, ValueError):
        return None

def parse_global(line, origin_lat, origin_lon):
    parts = line.strip().split(',')
    try:
        lat = float(parts[2])
        lon = float(parts[3])
        depth = float(parts[4])
        x, y = latlon_to_meters(lat, lon, origin_lat, origin_lon)
        return [x, y, depth]
    except (IndexError, ValueError):
        return None

def compute_stats(positions):
    arr = np.array(positions)
    mean_val = np.mean(arr, axis=0)
    cov = np.cov(arr, rowvar=False)
    eigenvalues, _ = np.linalg.eig(cov)
    r = np.sqrt(np.max(eigenvalues))
    return mean_val, cov, r

def main():
    origin_lat, origin_lon = 59.429155409, 10.464601094
    with open("position_log.txt", "r") as f:
        lines = f.read().splitlines()

    local_positions = [parse_local(line) for line in lines if line.startswith("LOCAL POSITION")]
    local_positions = [p for p in local_positions if p is not None]

    global_positions = [parse_global(line, origin_lat, origin_lon) for line in lines if line.startswith("GLOBAL_POSITION_INT")]
    global_positions = [p for p in global_positions if p is not None]

    if local_positions:
        mean_local, cov_local, r_local = compute_stats(local_positions)
        print("Local Position Mean [x, y, z]:", mean_local)
        print("Local Position Covariance Matrix:")
        for row in cov_local:
            print(row)
        print("Local Position Inaccuracy (r):", r_local, "meters")
    print() # split output
    if global_positions:
        mean_global, cov_global, r_global = compute_stats(global_positions)
        print("Global Position Mean [x, y, z]:", mean_global)
        print("Global Position Covariance Matrix:")
        for row in cov_global:
            print(row)
        print("Global Position Inaccuracy (r):", r_global, "meters")

if __name__ == "__main__":
    main()
