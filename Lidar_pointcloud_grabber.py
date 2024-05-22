import ouster.sdk.client as client
from ouster.sdk.client import ChanField, XYZLut
import numpy as np
import open3d as o3d
import os
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN

def filter_outliers(point_cloud, nb_neighbors=50, std_ratio=5.0):
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    filtered_cloud = point_cloud.select_by_index(ind)
    return filtered_cloud

def interpolate_points(point_cloud, density_factor=1):
    points = np.asarray(point_cloud.points)
    tree = KDTree(points)
    interpolated_points = []
    
    for i in range(len(points)):
        distances, indices = tree.query(points[i], k=3)  # Find the 2 closest neighbors
        for j in range(1, len(indices)):
            for _ in range(density_factor):
                interpolated_point = points[i] + (points[indices[j]] - points[i]) * np.random.random()
                interpolated_points.append(interpolated_point)
    
    interpolated_points = np.array(interpolated_points)
    combined_points = np.vstack((points, interpolated_points))
    combined_cloud = o3d.geometry.PointCloud()
    combined_cloud.points = o3d.utility.Vector3dVector(combined_points)
    return combined_cloud

def cluster_point_cloud(point_cloud, eps=0.5, min_samples=10):
    points = np.asarray(point_cloud.points)
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = db.labels_
    unique_labels = set(labels)
    clusters = []
    
    for k in unique_labels:
        if k != -1:  # Ignore noise
            class_member_mask = (labels == k)
            cluster_points = points[class_member_mask]
            clusters.append(cluster_points)
    
    return clusters

def save_clusters_info(clusters, file_path):
    with open(file_path, 'w') as f:
        f.write("ClusterID X Y Z\n")
        for i, cluster in enumerate(clusters):
            centroid = np.mean(cluster, axis=0)
            f.write(f"{i} {centroid[0]:.3f} {centroid[1]:.3f} {centroid[2]:.3f}\n")

def capture_and_visualize_scans(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503, nb_neighbors=50, std_ratio=5.0, eps=0.5, min_samples=10, num_scans=5, density_factor=1, min_distance=0.0, max_distance=100.0):
    try:
        # Determine the path to the Downloads folder
        downloads_path = os.path.join(os.path.expanduser("~"), "Downloads", "lidar_scans")

        # Create the output folder if it doesn't exist
        if not os.path.exists(downloads_path):
            os.makedirs(downloads_path)

        # Initialize the sensor with IP address and ports
        sensor = client.Sensor(hostname=sensor_ip, lidar_port=lidar_port, imu_port=imu_port)
        print(f"Connected to sensor: {sensor_ip}")

        # Retrieve sensor metadata
        metadata = sensor.metadata
        print(f"Metadata: {metadata}")

        # Create an XYZ lookup table for converting raw LIDAR data to XYZ coordinates
        xyz_lut = XYZLut(metadata)

        # Create a LidarScan stream with increased timeout to read data from the sensor
        scans = client.Scans(sensor, timeout=10.0)

        point_clouds = []

        # Loop through the scans and process each one
        for i, scan in enumerate(scans):
            if i >= num_scans:
                break

            # Extract the XYZ point cloud data from the scan
            xyz = xyz_lut(scan)

            # Remove any points that are all zeros
            valid_points = xyz.reshape(-1, 3)
            valid_points = valid_points[~np.all(valid_points == 0, axis=1)]

            # Calculate the distance of each point from the origin
            distances = np.linalg.norm(valid_points, axis=1)

            # Filter points to include only those in the front 180 degrees and within the distance range
            front_points = valid_points[(valid_points[:, 0] > 0) & (distances >= min_distance) & (distances <= max_distance)]

            # Check if the filtered XYZ data is valid
            if front_points.size > 0:
                # Convert the filtered points to Open3D format for further processing
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(front_points)

                # Apply statistical outlier removal filter
                filtered_cloud = filter_outliers(point_cloud, nb_neighbors, std_ratio)

                # Interpolate points to fill in gaps and increase point density
                interpolated_cloud = interpolate_points(filtered_cloud, density_factor)

                # Cluster the point cloud to find objects
                clusters = cluster_point_cloud(interpolated_cloud, eps, min_samples)

                # Save filtered and interpolated point cloud to a text file in the Downloads folder
                point_cloud_file_path = os.path.join(downloads_path, f"scan_{i+1}_point_cloud.txt")
                np.savetxt(point_cloud_file_path, np.asarray(interpolated_cloud.points), delimiter=" ", header="X Y Z")
                print(f"Filtered and interpolated point cloud saved to {point_cloud_file_path}")

                # Save cluster information to a text file in the Downloads folder
                clusters_info_file_path = os.path.join(downloads_path, f"scan_{i+1}_clusters_info.txt")
                save_clusters_info(clusters, clusters_info_file_path)
                print(f"Cluster information saved to {clusters_info_file_path}")

                point_clouds.append(interpolated_cloud)

            else:
                print("Received empty or invalid XYZ data")

        # Visualize all the collected point clouds together
        if point_clouds:
            try:
                # Initialize the Open3D visualization window
                vis = o3d.visualization.Visualizer()
                vis.create_window()

                # Add each point cloud to the visualization window
                for pc in point_clouds:
                    vis.add_geometry(pc)

                vis.run()
                vis.destroy_window()
            except Exception as e:
                print(f"An error occurred during visualization: {e}")

        # Close the sensor connection
        sensor.close()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    sensor_ip = "192.168.3.4"  # Replace with your sensor's static IP address
    # Capture and visualize scans with specified distance range and save files to the Downloads folder
    capture_and_visualize_scans(sensor_ip, min_distance=0.0, max_distance=50.0)  # Adjust the distance range as needed
