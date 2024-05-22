import ouster.sdk.client as client
from ouster.sdk.client import ChanField, XYZLut
import numpy as np
import open3d as o3d
import os
from scipy.spatial import KDTree

def filter_outliers(point_cloud, nb_neighbors=50, std_ratio=5.0):
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    filtered_cloud = point_cloud.select_by_index(ind)
    return filtered_cloud

def interpolate_points(point_cloud, num_interpolations=1):
    points = np.asarray(point_cloud.points)
    tree = KDTree(points)
    interpolated_points = []
    
    for i in range(len(points)):
        distances, indices = tree.query(points[i], k=3)  # Find the 2 closest neighbors
        for j in range(1, len(indices)):
            for _ in range(num_interpolations):
                interpolated_point = points[i] + (points[indices[j]] - points[i]) * np.random.random()
                interpolated_points.append(interpolated_point)
    
    interpolated_points = np.array(interpolated_points)
    combined_points = np.vstack((points, interpolated_points))
    combined_cloud = o3d.geometry.PointCloud()
    combined_cloud.points = o3d.utility.Vector3dVector(combined_points)
    return combined_cloud

def capture_and_visualize_single_scan(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503, nb_neighbors=20, std_ratio=2.0):
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

            # Remove any points that are all zeros
            valid_points = xyz.reshape(-1, 3)
            valid_points = valid_points[~np.all(valid_points == 0, axis=1)]

            # Filter points to include only those in the front 180 degrees
            front_points = valid_points[valid_points[:, 0] > 0]

            # Check if the filtered XYZ data is valid
            if front_points.size > 0:
                # Convert to Open3D format for outlier filtering
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(front_points)

                # Apply statistical outlier removal filter
                filtered_cloud = filter_outliers(point_cloud, nb_neighbors, std_ratio)

                # Interpolate points to fill in gaps
                interpolated_cloud = interpolate_points(filtered_cloud)

                # Determine the path to the Downloads folder
                downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")

                # Save filtered and interpolated point cloud to a text file in the Downloads folder
                file_path = os.path.join(downloads_path, "single_frame_point_cloud.txt")
                np.savetxt(file_path, np.asarray(interpolated_cloud.points), delimiter=" ", header="X Y Z")
                print(f"Filtered and interpolated point cloud saved to {file_path}")

                # Visualize filtered and interpolated point cloud
                o3d.visualization.draw_geometries([interpolated_cloud])

                break  # Process only one scan
            else:
                print("Received empty or invalid XYZ data")

        # Close the sensor connection
        sensor.close()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    sensor_ip = "192.168.3.4"  # Replace with your sensor's static IP address
    capture_and_visualize_single_scan(sensor_ip)
