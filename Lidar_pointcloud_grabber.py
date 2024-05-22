import ouster.sdk.client as client
import numpy as np
import open3d as o3d

def capture_single_frame(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503):
    try:
        # Initialize the sensor with IP address and ports
        with client.Sensor(hostname=sensor_ip, lidar_port=lidar_port, imu_port=imu_port) as sensor:
            print(f"Connected to sensor: {sensor_ip}")

            # Retrieve sensor metadata
            metadata = sensor.metadata
            print(f"Metadata: {metadata}")

            # Create a LidarScan stream with increased timeout
            scans = client.Scans(sensor, timeout=10.0)

            for scan in scans:
                # Convert LidarScan to point cloud
                xyz = client.destagger(metadata, scan.field(client.ChanField.XYZ))

                # Convert to Open3D format for visualization
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))

                # Save point cloud to file
                o3d.io.write_point_cloud("single_frame_point_cloud.pcd", point_cloud)

                # Visualize point cloud
                o3d.visualization.draw_geometries([point_cloud])

                break  # Process only one scan

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    sensor_ip = "192.168.3.4"  # Replace with your sensor's static IP address
    capture_single_frame(sensor_ip)
