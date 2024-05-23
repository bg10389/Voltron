#GPT test 4o (1)

import ouster.sdk.client as client  # Import the Ouster SDK client
from ouster.sdk.client import ChanField, XYZLut  # Import specific classes from the Ouster SDK client
import numpy as np  # Import NumPy for numerical operations
import open3d as o3d  # Import Open3D for point cloud processing
import os  # Import OS module for interacting with the operating system
from scipy.spatial import KDTree  # Import KDTree from SciPy for nearest neighbor searches
from sklearn.cluster import DBSCAN  # Import DBSCAN from scikit-learn for clustering

# Function to filter outliers from the point cloud using statistical analysis
def filter_outliers(point_cloud, nb_neighbors=100, std_ratio=20.0):
    # Perform statistical outlier removal
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    # Select the inlier points (those not identified as outliers)
    filtered_cloud = point_cloud.select_by_index(ind)
    return filtered_cloud  # Return the filtered point cloud

# Function to interpolate points to increase point density
def interpolate_points(point_cloud, density_factor=1):
    points = np.asarray(point_cloud.points)  # Convert point cloud to NumPy array
    tree = KDTree(points)  # Create a KDTree for the points
    interpolated_points = []  # List to hold interpolated points

    # Loop through each point in the point cloud
    for i in range(len(points)):
        # Find the 2 closest neighbors for each point
        distances, indices = tree.query(points[i], k=3)
        # Interpolate new points between the current point and its neighbors
        for j in range(1, len(indices)):
            for _ in range(density_factor):
                interpolated_point = points[i] + (points[indices[j]] - points[i]) * np.random.random()
                interpolated_points.append(interpolated_point)

    interpolated_points = np.array(interpolated_points)  # Convert to NumPy array
    combined_points = np.vstack((points, interpolated_points))  # Combine original and interpolated points
    combined_cloud = o3d.geometry.PointCloud()  # Create a new Open3D point cloud
    combined_cloud.points = o3d.utility.Vector3dVector(combined_points)  # Set the points of the point cloud
    return combined_cloud  # Return the combined point cloud

# Function to cluster the point cloud using DBSCAN algorithm
def cluster_point_cloud(point_cloud, eps=0.5, min_samples=10):
    points = np.asarray(point_cloud.points)  # Convert point cloud to NumPy array
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)  # Perform DBSCAN clustering
    labels = db.labels_  # Get cluster labels
    unique_labels = set(labels)  # Get unique cluster labels
    clusters = []  # List to hold clusters

    # Loop through each unique label (cluster)
    for k in unique_labels:
        if k != -1:  # Ignore noise points
            class_member_mask = (labels == k)  # Create a mask for the current cluster
            cluster_points = points[class_member_mask]  # Select points belonging to the current cluster
            clusters.append(cluster_points)  # Add the cluster points to the list

    return clusters  # Return the list of clusters

# Function to create a cylinder from a point cloud cluster
def create_cylinder_from_cluster(cluster_points, color):
    centroid = np.mean(cluster_points, axis=0)  # Calculate the centroid of the cluster
    height = np.max(cluster_points[:, 2]) - np.min(cluster_points[:, 2])  # Calculate the height of the cluster
    radius = np.mean(np.linalg.norm(cluster_points[:, :2] - centroid[:2], axis=1))  # Estimate the radius
    mesh = o3d.geometry.TriangleMesh.create_cylinder(radius, height)  # Create a cylinder mesh
    mesh.translate(centroid)  # Translate the cylinder to the centroid
    mesh.paint_uniform_color(color)  # Paint the cylinder with the specified color
    return mesh  # Return the cylinder mesh

# Main function to capture and visualize LIDAR scans
def capture_and_visualize_scans(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503, nb_neighbors=50, std_ratio=5.0, eps=0.5, min_samples=10, num_scans=5, density_factor=1, min_distance=0.0, max_distance=100.0, ground_threshold=0.2):
    try:
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

        point_clouds = []  # List to hold point clouds

        # Loop through the scans and process each one
        for i, scan in enumerate(scans):
            if i >= num_scans:  # Stop if the desired number of scans is reached
                break

            # Extract the XYZ point cloud data from the scan
            xyz = xyz_lut(scan)

            # Remove any points that are all zeros
            valid_points = xyz.reshape(-1, 3)
            valid_points = valid_points[~np.all(valid_points == 0, axis=1)]

            # Remove ground points below the ground threshold
            valid_points = valid_points[valid_points[:, 0] > ground_threshold]

            # Calculate the distance of each point from the origin
            distances = np.linalg.norm(valid_points, axis=1)

            # Filter points to include only those in the front 180 degrees and within the distance range
            front_points = valid_points[(valid_points[:, 0] > 0) & (distances >= min_distance) & (distances <= max_distance)]

            # Check if the filtered XYZ data is valid
            if front_points.size > 0:
                # Convert to Open3D format for outlier filtering
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(front_points)

                # Apply statistical outlier removal filter
                filtered_cloud = filter_outliers(point_cloud, nb_neighbors, std_ratio)

                # Interpolate points to fill in gaps
                interpolated_cloud = interpolate_points(filtered_cloud, density_factor)

                # Cluster the point cloud to find objects
                clusters = cluster_point_cloud(interpolated_cloud, eps, min_samples)

                # Add the processed point cloud to the list
                point_clouds.append(interpolated_cloud)

            else:
                print("Received empty or invalid XYZ data")

        # Check if there are any point clouds to visualize
        if point_clouds:
            try:
                # Initialize the Open3D visualization window
                vis = o3d.visualization.Visualizer()
                vis.create_window()

                # Create a black cylinder at the origin of the lidar
                lidar_origin_cylinder = o3d.geometry.TriangleMesh.create_cylinder(0.1, 0.5)
                lidar_origin_cylinder.paint_uniform_color([0, 0, 0])
                vis.add_geometry(lidar_origin_cylinder)

                # Visualize the clusters as orange cylinders
                cone_color = [1, 0.549, 0]  # Orange color
                for i, cluster in enumerate(clusters[:4]):  # Limit to 4 cones
                    cylinder = create_cylinder_from_cluster(cluster, cone_color)
                    vis.add_geometry(cylinder)

                # Add measuring reference rings
                for radius in range(1, 11):  # 1 to 10 meters
                    ring = o3d.geometry.TriangleMesh.create_torus(radius=radius, tube_radius=0.01)
                    ring.paint_uniform_color([0.5, 0.5, 0.5])  # Grey color
                    vis.add_geometry(ring)

                vis.run()  # Run the visualization
                vis.destroy_window()  # Destroy the visualization window
            except Exception as e:
                print(f"An error occurred during visualization: {e}")

        # Close the sensor connection
        sensor.close()

    except Exception as e:
        print(f"An error occurred: {e}")

# Main execution block
if __name__ == "__main__":
    sensor_ip = "192.168.3.4"  # Replace with your sensor's static IP address
    # Capture and visualize scans with specified distance range and ground threshold
    capture_and_visualize_scans(sensor_ip, min_distance=0.0, max_distance=9.8, ground_threshold=0.2)  # Adjust the distance range and ground threshold as needed
