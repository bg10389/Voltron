import os
from pykml import parser
from math import atan2, degrees, sqrt

# Step 1: Find the first KML file in the current directory
def find_kml_file():
    for file in os.listdir('.'):
        if file.endswith('.kml'):
            return file
    raise FileNotFoundError("No KML file found in the current directory")

# Step 2: Extract coordinates from KML
def extract_coordinates(kml_file):
    with open(kml_file, 'r') as file:
        root = parser.parse(file).getroot()
        coordinates = []
        for placemark in root.Document.Placemark:
            for line in placemark.LineString.coordinates.text.strip().split():
                lon, lat, _ = map(float, line.split(','))
                coordinates.append((lat, lon))
    return coordinates

# Step 3: Calculate steering angles
def calculate_steering_angles(coords):
    angles = []
    for i in range(1, len(coords)):
        lat1, lon1 = coords[i-1]
        lat2, lon2 = coords[i]
        angle = atan2(lat2 - lat1, lon2 - lon1)
        angles.append(degrees(angle))
    return angles

# Step 4: Calculate throttle values
def calculate_throttle_values(coords, base_speed, lap_speed_factor):
    adjusted_speed = base_speed * lap_speed_factor
    distances = []
    for i in range(1, len(coords)):
        lat1, lon1 = coords[i-1]
        lat2, lon2 = coords[i]
        distance = sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)
        distances.append(distance)
    throttle_values = [distance / adjusted_speed for distance in distances]
    return throttle_values

# Step 5: Map to SBUS values
def map_to_sbus(throttle_values, steering_angles):
    # SBUS channel values range from 172 to 1811
    sbus_min = 172
    sbus_max = 1811

    throttle_sbus = [int(throttle * (sbus_max - sbus_min) + sbus_min) for throttle in throttle_values]
    steering_sbus = [int(((angle + 180) / 360) * (sbus_max - sbus_min) + sbus_min) for angle in steering_angles]
    return throttle_sbus, steering_sbus

# Main function to execute the steps
def create_throttle_and_steering_map(base_speed, lap_speed_factor):
    kml_file = find_kml_file()
    coords = extract_coordinates(kml_file)
    steering_angles = calculate_steering_angles(coords)
    throttle_values = calculate_throttle_values(coords, base_speed, lap_speed_factor)
    throttle_sbus, steering_sbus = map_to_sbus(throttle_values, steering_angles)
    return throttle_sbus, steering_sbus

# Example usage
base_speed = 10  # Base speed
lap_speed_factor = 1.5  # Example lap speed factor to increase the speed by 50%
throttle_sbus, steering_sbus = create_throttle_and_steering_map(base_speed, lap_speed_factor)
print("Throttle SBUS Values:", throttle_sbus)
print("Steering SBUS Values:", steering_sbus)
