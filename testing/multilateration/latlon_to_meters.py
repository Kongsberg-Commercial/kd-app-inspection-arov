import numpy as np

def latlon_to_meters(lat, lon, origin_lat, origin_lon):
    R = 6371000  
    dlat = np.deg2rad(lat - origin_lat)
    dlon = np.deg2rad(lon - origin_lon)
    x = R * dlon * np.cos(np.deg2rad(origin_lat))
    y = R * dlat
    return x, y

# Define the origin (first transponder)
origin_lat, origin_lon = 59.428465173, 10.465137681

# Example transponder coordinates
lat_t2, lon_t2 = 59.428465173, 10.465137681
x_t2, y_t2 = latlon_to_meters(lat_t2, lon_t2, origin_lat, origin_lon)
print(f"{x_t2:.4f}, {y_t2:.4f}")

lat_t2, lon_t2 = 59.428445150, 10.465241581
x_t2, y_t2 = latlon_to_meters(lat_t2, lon_t2, origin_lat, origin_lon)
print(f"{x_t2:.4f}, {y_t2:.4f}")

lat_t2, lon_t2 = 59.428427280, 10.465334121
x_t2, y_t2 = latlon_to_meters(lat_t2, lon_t2, origin_lat, origin_lon)
print(f"{x_t2:.4f}, {y_t2:.4f}")


lat_t2, lon_t2 = 59.428407132, 10.465439580
x_t2, y_t2 = latlon_to_meters(lat_t2, lon_t2, origin_lat, origin_lon)
print(f"{x_t2:.4f}, {y_t2:.4f}")


# 59.428465173, 10.465137681
# 59.428445150, 10.465241581
# 59.428427280, 10.465334121
# 59.428407132, 10.465439580