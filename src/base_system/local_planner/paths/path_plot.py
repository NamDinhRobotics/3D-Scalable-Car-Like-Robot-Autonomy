import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Function to read .ply file with path_id and group_id
def read_ply_with_properties(filename):
    # Load the .ply file
    pcd = o3d.io.read_point_cloud(filename, format='ply')
    
    # Read data manually to include additional properties
    points = []
    path_ids = []
    group_ids = []
    
    with open(filename, 'r') as f:
        lines = f.readlines()
        header_ended = False
        for line in lines:
            if header_ended:
                values = line.strip().split()
                x, y, z = map(float, values[:3])
                path_id = int(values[3])
                group_id = int(values[4])
                points.append([x, y, z])
                path_ids.append(path_id)
                group_ids.append(group_id)
            elif line.strip() == "end_header":
                header_ended = True

    points = np.array(points)
    path_ids = np.array(path_ids)
    group_ids = np.array(group_ids)
    return points, path_ids, group_ids

# Function to assign unique colors based on IDs
def assign_colors(ids):
    unique_ids = np.unique(ids)
    colormap = plt.get_cmap("tab20")  # Use a categorical colormap
    colors = np.array([colormap(i / len(unique_ids))[:3] for i in range(len(unique_ids))])
    id_to_color = {id_: colors[i] for i, id_ in enumerate(unique_ids)}
    return np.array([id_to_color[id_] for id_ in ids])

# Visualize paths with colors for each group_id and path_id
def visualize_ply_with_colors(points, path_ids, group_ids):
    # Assign colors based on group_id
    group_colors = assign_colors(group_ids)
    
    # Assign colors based on path_id
    path_colors = assign_colors(path_ids)
    
    # Create two point clouds for visualization
    group_pcd = o3d.geometry.PointCloud()
    group_pcd.points = o3d.utility.Vector3dVector(points)
    group_pcd.colors = o3d.utility.Vector3dVector(group_colors)
    
    path_pcd = o3d.geometry.PointCloud()
    path_pcd.points = o3d.utility.Vector3dVector(points)
    path_pcd.colors = o3d.utility.Vector3dVector(path_colors)
    
    # Visualize
    print("Visualizing Group IDs")
    o3d.visualization.draw_geometries([group_pcd], window_name="Group ID Visualization")
    print("Visualizing Path IDs")
    o3d.visualization.draw_geometries([path_pcd], window_name="Path ID Visualization")

# Main workflow
ply_file = "paths.ply"  # Replace with the path to your .ply file
points, path_ids, group_ids = read_ply_with_properties(ply_file)
visualize_ply_with_colors(points, path_ids, group_ids)

