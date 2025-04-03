import os
import re
import open3d as o3d
import numpy as np

def combine_pcd_frames(folder_path, prefix="Record3 (Frame ", extension=".pcd"):
    

    # List all files in the folder
    all_files = os.listdir(folder_path)

    # Filter only those matching the pattern "Record3 (Frame X).pcd"
    # We'll use a regex to extract the integer frame number.
    pattern = re.compile(r'^Record3 \(Frame (\d+)\)\.pcd$')

    valid_files = []
    for f in all_files:
        match = pattern.match(f)
        if match:
            # Extract frame number to sort properly
            frame_num = int(match.group(1))
            full_path = os.path.join(folder_path, f)
            valid_files.append((frame_num, full_path))

    # Sort by the frame number
    valid_files.sort(key=lambda x: x[0])

    # Combine them
    combined_pcd = o3d.geometry.PointCloud()
    for frame_num, file_path in valid_files:
        print(f"Loading frame {frame_num} -> {file_path}")
        pcd = o3d.io.read_point_cloud(file_path)
        combined_pcd += pcd

    return combined_pcd

def combine_pcd_files(file_paths, voxel_size=None):
    """
    Combine multiple PCD files into a single point cloud
    
    Args:
        file_paths: List of paths to PCD files
        voxel_size: Optional voxel size for downsampling (None = no downsampling)
    
    Returns:
        Combined Open3D PointCloud
    """
    if not file_paths:
        raise ValueError("No PCD files provided")
    
    # Combine all point clouds
    combined_pcd = o3d.geometry.PointCloud()
    for file_path in file_paths:
        pcd = o3d.io.read_point_cloud(file_path)
        combined_pcd += pcd
    
    # Optionally downsample
    if voxel_size is not None and voxel_size > 0:
        combined_pcd = combined_pcd.voxel_down_sample(voxel_size)
    
    return combined_pcd

def save_combined_pcd(combined_pcd, output_path):
    """
    Save a combined point cloud to a PCD file
    
    Args:
        combined_pcd: Open3D PointCloud to save
        output_path: Path where to save the PCD file
    
    Returns:
        Path to the saved file
    """
    o3d.io.write_point_cloud(output_path, combined_pcd)
    return output_path

def main():
    folder_path = r"C:\Users\salip\Desktop\Final Year Project\Scans\Record3"

    # 1) Combine frames from the folder
    aggregated_cloud = combine_pcd_frames(folder_path)

    # 2) Print total points
    print(f"Combined point cloud has {len(aggregated_cloud.points)} points.")

    # 3) Save to a new PCD file
    output_file = os.path.join(folder_path, "aggregated_scan_record3.pcd")
    o3d.io.write_point_cloud(output_file, aggregated_cloud)
    print(f"Saved combined point cloud to: {output_file}")

    # 4) Optional: Visualize
    o3d.visualization.draw_geometries([aggregated_cloud])

if __name__ == "__main__":
    main()
