import ouster.sdk.client as client
from ouster.sdk.client import ChanField, XYZLut
import numpy as np
import os

def capture_single_frame(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503):
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

        for scan in scans:
            # Extract the XYZ point cloud data
            xyz = xyz_lut(scan)

            # Check if the XYZ data is valid
            if xyz is not None and xyz.size > 0 and not np.all(xyz == 0):
                # Determine the path to the Downloads folder
                downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")

                # Save point cloud to a text file in the Downloads folder
                file_path = os.path.join(downloads_path, "single_frame_point_cloud.txt")
                np.savetxt(file_path, xyz.reshape(-1, 3), delimiter=" ", header="X Y Z")
                print(f"Point cloud saved to {file_path}")

                break  # Process only one scan
            else:
                print("Received empty or invalid XYZ data")

        # Close the sensor connection
        sensor.close()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    sensor_ip = "192.168.3.4"  # Replace with your sensor's static IP address
    capture_single_frame(sensor_ip)
