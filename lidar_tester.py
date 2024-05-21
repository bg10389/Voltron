# testing week of comp
#connects to lidar and scans, takes data and converts to point cloud, saves to file, visualizes file

import ouster.client as client
import numpy as np
import open3d as o3d

def capture_live_data(sensor_hostname: str, lidar_port: int = 7502, imu_port: int = 7503):
    # Create a sensor client
    try:
        # Initialize the sensor with hostname and ports
        with client.Sensor(hostname=sensor_hostname, lidar_port=lidar_port, imu_port=imu_port) as sensor:
            print(f"Connected to sensor: {sensor_hostname}")

            # Retrieve sensor metadata
            metadata = sensor.metadata
            print(f"Metadata: {metadata}")

            # Create a LidarScan stream
            scans = client.Scans(sensor)

            for scan in scans:
                # Convert LidarScan to point cloud
                xyz = client.destagger(metadata, scan.field(client.ChanField.XYZ))
                
                # Convert to Open3D format for visualization
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))

                # Save point cloud to file
                o3d.io.write_point_cloud("point_cloud.pcd", point_cloud)
                
                # Visualize point cloud
                o3d.visualization.draw_geometries([point_cloud])
                
                break  # For this example, we process only one scan

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    sensor_hostname = "os-1221234.local"  # Replace with your sensor's hostname or IP address
    capture_live_data(sensor_hostname)
