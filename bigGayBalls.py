import numpy as np
import open3d as o3d
from ouster import client

def capture_live_data(sensor_hostname: str, lidar_port: int = 7502, imu_port: int = 7503):
    try:
        # Initialize the sensor
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

                # Process point cloud for cone isolation and crash avoidance
                process_point_cloud(point_cloud)

                break  # For this example, we process only one scan

    except Exception as e:
        print(f"An error occurred: {e}")

def process_point_cloud(pcd):
    # Downsample point cloud
    pcd_down = pcd.voxel_down_sample(voxel_size=0.05)

    # Estimate normals
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))

    # Segment the ground plane
    plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.02,
                                                  ransac_n=3,
                                                  num_iterations=1000)
    inlier_cloud = pcd_down.select_by_index(inliers)
    outlier_cloud = pcd_down.select_by_index(inliers, invert=True)

    # Clustering to find cones
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(outlier_cloud.cluster_dbscan(eps=0.1, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")

    # Isolate cones
    cones = outlier_cloud.select_by_index(np.where(labels >= 0)[0])

    # Save or visualize cones
    o3d.io.write_point_cloud("isolated_cones.pcd", cones)
    o3d.visualization.draw_geometries([cones])

    # Process isolated cones for crash avoidance
    crash_avoidance(cones)

def crash_avoidance(cones):
    about_to_crash = 0
    dont = 0

    # Assuming 'cones' is the point cloud of isolated cones
    distances = cones.compute_point_cloud_distance(cones)
    min_distance = min(distances) if distances else float('inf')

    if min_distance < 0.3048:  # 1 ft in meters
        about_to_crash = 1
        if about_to_crash > 0:
            dont = 1

    # Movement logic
    assigned = 'left'  # Example assigned direction, modify as needed
    if dont and assigned == 'left':
        print("Move right")
    elif dont and assigned == 'right':
        print("Move left")
    else:
        print("No immediate danger detected.")

if __name__ == "__main__":
    sensor_hostname = "os-1221234.local"  # Replace with your sensor's hostname or IP address
    capture_live_data(sensor_hostname)
