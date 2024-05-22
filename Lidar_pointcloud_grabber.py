import ouster.sdk.client as client
from ouster.sdk.client import ChanField, XYZLut
import numpy as np
import os

def capture_and_save_scans(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503, num_scans: int = 1):
    try:
        # Initialize the sensor with IP address and ports
        sensor = client.Sensor(hostname=sensor_ip, lidar_port=lidar_port, imu_port=imu_port)
        print(f"Connected to sensor: {sensor_ip}")

        # Retrieve sensor metadata
        metadata = sensor.metadata
        print(f"Metadata: {metadata}")

        # Create an XYZ lookup table
        xyz_lut = XYZLut(metadata)

        # Create a LidarScan stream with increased timeout
        scans = client.Scans(sensor, timeout=10.0)

        for i, scan in enumerate(scans):
            # Extract the XYZ point cloud data
            xyz = xyz_lut(scan)

            # Remove any points that are all zeros
            valid_points = xyz.reshape(-1, 3)
            valid_points = valid_points[~np.all(valid_points == 0, axis=1)]

            # Check if the XYZ data is valid
            if valid_points.size > 0:
                # Determine the path to the Downloads folder
                downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")

                # Save point cloud to a text file in the Downloads folder
                file_path = os.path.join(downloads_path, f"scan_{i+1}_point_cloud.txt")
                np.savetxt(file_path, valid_points, delimiter=" ", header="X Y Z")
                print(f"Point cloud saved to {file_path}")

                if i + 1 >= num_scans:
                    break  # Stop after capturing the specified number of scans
            else:
                print("Received empty or invalid XYZ data")

        # Close the sensor connection
        sensor.close()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    sensor_ip = "192.168.3.4"  # Replace with your sensor's static IP address
    capture_and_save_scans(sensor_ip)
