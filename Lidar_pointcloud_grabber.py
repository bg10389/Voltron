import ouster.sdk.client as client  # Import the Ouster SDK client
from ouster.sdk.client import ChanField, XYZLut  # Import specific classes from the Ouster SDK client
import numpy as np  # Import NumPy for numerical operations
import open3d as o3d  # Import Open3D for point cloud processing
import os  # Import OS module for interacting with the operating system
from scipy.spatial import KDTree  # Import KDTree from SciPy for nearest neighbor searches
from sklearn.cluster import DBSCAN  # Import DBSCAN from scikit-learn for clustering

# Function to filter outliers from the point cloud using statistical analysis
def filter_outliers(point_cloud, nb_neighbors=100, std_ratio=20.0):
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    filtered_cloud = point_cloud.select_by_index(ind)
    return filtered_cloud  # Return the filtered point cloud

# Function to interpolate points to increase point density
def interpolate_points(point_cloud, density_factor=1):
    points = np.asarray(point_cloud.points)  # Convert point cloud to NumPy array
    tree = KDTree(points)  # Create a KDTree for the points
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

# Function to cluster the point cloud using DBSCAN algorithm
def cluster_point_cloud(point_cloud, eps=0.5, min_samples=10):
    points = np.asarray(point_cloud.points)
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = db.labels_
    unique_labels = set(labels)
    clusters = []

    for k in unique_labels:
        if k != -1:  # Ignore noise points
            class_member_mask = (labels == k)
            cluster_points = points[class_member_mask]
            clusters.append(cluster_points)

    return clusters

# Function to save cluster information to a text file
def save_clusters_info(clusters, file_path):
    with open(file_path, 'w') as f:
        f.write("ClusterID X Y Z\n")
        for i, cluster in enumerate(clusters):
            centroid = np.mean(cluster, axis=0)
            f.write(f"{i} {centroid[0]:.3f} {centroid[1]:.3f} {centroid[2]:.3f}\n")

# Function to convert clusters to cones
def convert_clusters_to_cones(clusters):
    cones = []
    for cluster in clusters:
        if len(cluster) == 0:
            continue
        points = np.array(cluster)
        min_z = np.min(points[:, 2])
        max_z = np.max(points[:, 2])
        centroid = np.mean(points, axis=0)
        radius = np.linalg.norm(points[:, :2] - centroid[:2], axis=1).max()
        height = max_z - min_z

        # Ignore cones with radius larger than 0.3 meters
        if radius > 0.3:
            continue

        cone = o3d.geometry.TriangleMesh.create_cone(radius=radius, height=height)
        transformation = np.eye(4)
        transformation[:3, 3] = [centroid[0], centroid[1], min_z]  # Place the cone base at the minimum Z
        cone.transform(transformation)
        cone.paint_uniform_color([1, 0, 0])  # Set cone color to red
        cones.append(cone)
    return cones

# Function to create a black cylinder at the zero position
def create_zero_cylinder():
    radius = 0.1  # Radius of the cylinder
    height = 1.0  # Height of the cylinder
    zero_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=height)
    zero_cylinder.paint_uniform_color([0, 0, 0])  # Set cylinder color to black
    zero_cylinder.translate([0, 0, height / 2])  # Position the cylinder at the origin
    return zero_cylinder

# Main function to capture and visualize LIDAR scans
def capture_and_visualize_scans(sensor_ip: str, lidar_port: int = 7502, imu_port: int = 7503, nb_neighbors=50, std_ratio=5.0, eps=0.5, min_samples=10, num_scans=5, density_factor=1, min_distance=0.0, max_distance=100.0, ground_threshold=0.2):
    try:
        sensor = client.Sensor(hostname=sensor_ip, lidar_port=lidar_port, imu_port=imu_port)
        print(f"Connected to sensor: {sensor_ip}")

        metadata = sensor.metadata
        print(f"Metadata: {metadata}")

        xyz_lut = XYZLut(metadata)

        scans = client.Scans(sensor, timeout=10.0)

        point_clouds = []
        original_points = []

        for i, scan in enumerate(scans):
            if i >= num_scans:
                break

            xyz = xyz_lut(scan)

            valid_points = xyz.reshape(-1, 3)
            valid_points = valid_points[~np.all(valid_points == 0, axis=1)]

            valid_points = valid_points[valid_points[:, 2] > ground_threshold]

            distances = np.linalg.norm(valid_points, axis=1)

            front_points = valid_points[(valid_points[:, 0] > 0) & (distances >= min_distance) & (distances <= max_distance)]

            if front_points.size > 0:
                point_cloud = o3d.geometry.PointCloud()
                point_cloud.points = o3d.utility.Vector3dVector(front_points)
                original_points.append(point_cloud)

                filtered_cloud = filter_outliers(point_cloud, nb_neighbors, std_ratio)

                interpolated_cloud = interpolate_points(filtered_cloud, density_factor)

                clusters = cluster_point_cloud(interpolated_cloud, eps, min_samples)

                # Convert clusters to cones
                cones = convert_clusters_to_cones(clusters)

                # Ensure we have at least 4 cones
                if len(cones) < 4:
                    print(f"Warning: Only {len(cones)} cones detected. Trying to gather more scans.")
                    continue

                downloads_path = os.path.join(os.path.expanduser("~"), "Downloads")

                point_cloud_file_path = os.path.join(downloads_path, f"scan_{i+1}_point_cloud.txt")
                np.savetxt(point_cloud_file_path, np.asarray(interpolated_cloud.points), delimiter=" ", header="X Y Z")
                print(f"Filtered and interpolated point cloud saved to {point_cloud_file_path}")

                clusters_info_file_path = os.path.join(downloads_path, f"scan_{i+1}_clusters_info.txt")
                save_clusters_info(clusters, clusters_info_file_path)
                print(f"Cluster information saved to {clusters_info_file_path}")

                point_clouds.extend(cones)

            else:
                print("Received empty or invalid XYZ data")

        if len(point_clouds) >= 4:
            try:
                vis = o3d.visualization.Visualizer()
                vis.create_window()

                # Add original points to the visualization window
                for pc in original_points:
                    pc.paint_uniform_color([1, 0.6, 0])  # Set point cloud color to orange
                    vis.add_geometry(pc)

                # Add cones to the visualization window
                for cone in point_clouds:
                    vis.add_geometry(cone)

                # Add the zero position cylinder to the visualization window
                zero_cylinder = create_zero_cylinder()
                vis.add_geometry(zero_cylinder)

                vis.run()
                vis.destroy_window()
            except Exception as e:
                print(f"An error occurred during visualization: {e}")

        else:
            print("Insufficient number of cones detected. Please ensure there are at least 4 cones in view.")

        sensor.close()

    except Exception as e:
        print(f"An error occurred: {e}")

# Main execution block
if __name__ == "__main__":
    sensor_ip = "192.168.3.4"  # Replace with your sensor's static IP address
    capture_and_visualize_scans(sensor_ip, min_distance=0.0, max_distance=5.5, ground_threshold=0.2)  # Adjust the distance range and ground threshold as needed
