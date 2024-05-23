import ouster.sdk.client as client  # Import the Ouster SDK client
from ouster.sdk.client import ChanField, XYZLut  # Import specific classes from the Ouster SDK client
import numpy as np  # Import NumPy for numerical operations
import open3d as o3d  # Import Open3D for point cloud processing
import os  # Import OS module for interacting with the operating system
from scipy.spatial import KDTree  # Import KDTree from SciPy for nearest neighbor searches
from sklearn.cluster import DBSCAN  # Import DBSCAN from scikit-learn for clustering

# Function to filter outliers from the point cloud using statistical analysis
def filter_outliers(point_cloud, nb_neighbors=50, std_ratio=5.0):
    # Perform statistical outlier removal
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    # Select the inlier points
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

# Function to save cluster information to a text file
def save_clusters_info(clusters, file_path):
    with open(file_path, 'w') as f:  # Open the file for writing
        f.write("ClusterID X Y Z\n")  # Write the header
        # Loop through each cluster
        for i, cluster in enumerate(clusters):
            centroid = np.mean(cluster, axis=0)  # Calculate the centroid of the cluster
            # Write the cluster ID and centroid coordinates to the file
            f.write(f"{i} {centroid[0]:.3f} {centroid[1]:.3f} {centroid[2]:.3f}\n")

# Main function to capture and visualize LIDAR scans
def capture_and_visualize_scans(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503, nb_neighbors=50, std_ratio=5.0, eps=0.5, min_samples=10, num_scans=25, density_factor=1, min_distance=0.0, max_distance=100.0, ground_threshold=0.2):
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
            valid_points = valid_points[valid_points[:, 2] > ground_threshold]

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

                # Determine the path to the Downloads folder
                downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")

                # Save filtered and interpolated point cloud to a text file in the Downloads folder
                point_cloud_file_path = os.path.join(downloads_path, f"scan_{i+1}_point_cloud.txt")
                np.savetxt(point_cloud_file_path, np.asarray(interpolated_cloud.points), delimiter=" ", header="X Y Z")
                print(f"Filtered and interpolated point cloud saved to {point_cloud_file_path}")

                # Save cluster information to a text file in the Downloads folder
                clusters_info_file_path = os.path.join(downloads_path, f"scan_{i+1}_clusters_info.txt")
                save_clusters_info(clusters, clusters_info_file_path)
                print(f"Cluster information saved to {clusters_info_file_path}")

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

                # Add each point cloud to the visualization window
                for pc in point_clouds:
                    vis.add_geometry(pc)

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
    capture_and_visualize_scans(sensor_ip, min_distance=0.0, max_distance=4.8, ground_threshold=0.02)  # Adjust the distance range and ground threshold as needed
