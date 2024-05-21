import numpy as np
import open3d as o3d
from ouster import client, pcap

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
    points = np.asarray(cones.points)
    if len(points) == 0:
        print("No cones detected.")
        return

    # Separate cones into left and right based on their x-coordinates
    left_cones = points[points[:, 1] > 0]
    right_cones = points[points[:, 1] < 0]

    if len(left_cones) == 0 or len(right_cones) == 0:
        print("Not enough cones to determine a path.")
        return

    # Find the closest cone on the left and right
    closest_left_cone = left_cones[np.argmin(np.linalg.norm(left_cones, axis=1))]
    closest_right_cone = right_cones[np.argmin(np.linalg.norm(right_cones, axis=1))]

    # Compute the midpoint between the closest left and right cones
    midpoint = (closest_left_cone + closest_right_cone) / 2

    print(f"Closest left cone: {closest_left_cone}")
    print(f"Closest right cone: {closest_right_cone}")
    print(f"Midpoint: {midpoint}")

    # Movement logic: adjust position to aim for the midpoint
    current_position = np.array([0, 0, 0])  # Assuming current position is the origin
    direction_vector = midpoint - current_position
    if direction_vector[1] > 0:
        print("Move right")
    elif direction_vector[1] < 0:
        print("Move left")
    else:
        print("Move forward")

if __name__ == "__main__":
    sensor_hostname = "os-1221234.local"  # Replace with your sensor's hostname or IP address
    capture_live_data(sensor_hostname)
