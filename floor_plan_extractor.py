import open3d as open3d
import numpy as numpy
import copy
import random
import logging
import ezdxf
from ezdxf.enums import TextEntityAlignment
from shapely.geometry import MultiPoint, Polygon, LineString, Point
import matplotlib.pyplot as plt
from shapely.geometry import MultiPoint, Polygon

# Configuration
VOXEL_SIZE = 0.04
RANSAC_DISTANCE_CANDIDATES = [0.02, 0.05, 0.1, 0.12]
MIN_WALL_LENGTH = 0.8
RANDOM_SEED = 42
random.seed(RANDOM_SEED)
DBSCAN_EPS = 0.8
DBSCAN_MIN_POINTS = 30
NORMAL_TOLERANCE = 0.2

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("floor_plan_extraction.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("floor_plan_extractor")

def load_and_preprocess_pcd(pcd_file):
    logger.info(f"Loading point cloud: {pcd_file}")
    pcd = open3d.io.read_point_cloud(pcd_file)
    if not pcd.has_points():
        logger.error(f"No points found in PCD file: {pcd_file}")
        raise ValueError(f"No points found in PCD file: {pcd_file}")
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
    return pcd

def save_processed_pcd(pcd, output_file):
    """Save a processed point cloud for later reuse"""
    logger.info(f"Saving processed point cloud to: {output_file}")
    open3d.io.write_point_cloud(output_file, pcd)
    logger.info(f"Saved processed point cloud successfully")
    return output_file

def load_processed_pcd(processed_file):
    """Load a previously processed point cloud"""
    print(f"Loading processed point cloud: {processed_file}")
    pcd = open3d.io.read_point_cloud(processed_file)
    if not pcd.has_points():
        raise ValueError("No points found in processed PCD file.")
    return pcd

def segment_walls(pcd):
    """Segments out vertical walls using plane detection (RANSAC) + DBSCAN"""
    # Set a fixed seed for consistent results
    numpy.random.seed(RANDOM_SEED)
    logger.info(f"Starting wall segmentation with {len(pcd.points)} points")
    vertical_wall_points = []
    walls_indices = []
    wall_models = []
    remaining_cloud = pcd
    
    for _ in range(5):
        if len(remaining_cloud.points) == 0:
            break
        
        best_inliers = []
        best_model = None
        for threshold in RANSAC_DISTANCE_CANDIDATES:
            plane_model, inliers = remaining_cloud.segment_plane(
                distance_threshold=threshold, 
                ransac_n=3, 
                num_iterations=1000
            )
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_model = plane_model
        
        if not best_inliers:
            break
            
        [a, b, c, d] = best_model
        if abs(c) < NORMAL_TOLERANCE:
            wall_cloud = remaining_cloud.select_by_index(best_inliers)
            labels = numpy.array(wall_cloud.cluster_dbscan(eps=DBSCAN_EPS, min_points=DBSCAN_MIN_POINTS))
            
            if labels.max() >= 0:
                majority_label = numpy.argmax(numpy.bincount(labels[labels >= 0]))
                main_cluster_indices = numpy.where(labels == majority_label)[0]
                main_cluster = wall_cloud.select_by_index(main_cluster_indices)
                
                if len(main_cluster_indices) >= DBSCAN_MIN_POINTS:
                    obb = main_cluster.get_oriented_bounding_box()
                    if max(obb.extent) >= MIN_WALL_LENGTH:
                        vertical_wall_points.append(numpy.asarray(main_cluster.points))
                        walls_indices.append(main_cluster_indices.tolist())
                        wall_models.append(best_model)
        
        remaining_cloud = remaining_cloud.select_by_index(best_inliers, invert=True)
    
    if not vertical_wall_points:
        logger.error("No vertical walls found in the point cloud")
        raise RuntimeError("No vertical walls found.")
        
    all_wall_points = numpy.vstack(vertical_wall_points)
    logger.info(f"Wall segmentation complete. Found {len(walls_indices)} wall segments with {len(all_wall_points)} points")
    return all_wall_points[:, :2], walls_indices, wall_models

def create_floor_plan(points_xy):
    """Creates a simple rectangular floor plan from wall points"""
    if len(points_xy) == 0:
        raise RuntimeError("No wall points available for floor plan extraction.")
    logger.info(f"Creating floor plan from {len(points_xy)} wall points")
    mp = MultiPoint(points_xy)
    polygon = mp.minimum_rotated_rectangle
    logger.info(f"Floor plan created. Area: {polygon.area:.2f} m², Perimeter: {polygon.length:.2f} m")
    return polygon

def export_floor_plan_to_dxf(polygon: Polygon, dxf_filename: str, add_dimensions=False):
    """Exports the floor plan polygon to DXF format"""
    doc = ezdxf.new(dxfversion="R2010")
    msp = doc.modelspace()
    
    # Add a layer for dimensions
    doc.layers.add(name="DIMENSIONS", color=2)  # color 2 is yellow
    
    exterior_coords = list(polygon.exterior.coords)
    for i in range(len(exterior_coords) - 1):
        x1, y1 = exterior_coords[i]
        x2, y2 = exterior_coords[i+1]
        msp.add_line((x1, y1), (x2, y2))
        
        # Add dimensions if requested
        if add_dimensions:
            # Calculate wall length
            wall_length = numpy.sqrt((x2-x1)**2 + (y2-y1)**2)
            
            # Add dimension text
            midpoint_x = (x1 + x2) / 2
            midpoint_y = (y1 + y2) / 2
            
            # Calculate perpendicular offset for dimension line
            dx, dy = x2-x1, y2-y1
            length = numpy.sqrt(dx*dx + dy*dy)
            if length > 0:
                nx, ny = -dy/length * 0.2, dx/length * 0.2  # Perpendicular vector, 0.2 units away
            else:
                nx, ny = 0, 0.2
            
            # Add dimension line
            dim_line = msp.add_line(
                (x1 + nx, y1 + ny),
                (x2 + nx, y2 + ny),
                dxfattribs={"layer": "DIMENSIONS"}
            )
            
            # Add dimension text
            text = msp.add_text(
                f"{wall_length:.2f}m",
                height=0.1, 
                dxfattribs={"layer": "DIMENSIONS"}
            )
            # Set position using the correct method for current ezdxf version
            # Use the proper enum value instead of a string
            text.set_placement((midpoint_x + nx, midpoint_y + ny), align=TextEntityAlignment.MIDDLE_CENTER)
    
    # Add area calculation
    area = polygon.area
    area_text = msp.add_text(f"Area: {area:.2f} m²", height=0.15)
    # Set position using the correct method for current ezdxf version
    # Use the proper enum value for alignment
    area_text.set_placement(polygon.centroid.coords[0], align=TextEntityAlignment.MIDDLE_CENTER)
    
    doc.saveas(dxf_filename)
    logger.info(f"DXF exported: {dxf_filename}")

def plot_and_save_floor_plan_pdf(points_xy, polygon, pdf_filename):
    """Creates and saves a PDF visualization of the floor plan"""
    plt.figure(figsize=(8, 8))
    
    # Plot wall points with lower opacity
    plt.scatter(points_xy[:, 0], points_xy[:, 1], s=1, c='gray', alpha=0.3, label='Wall Points')
    
    # Plot floor plan outline
    outline_x, outline_y = polygon.exterior.xy
    plt.plot(outline_x, outline_y, 'r-', linewidth=2, label='Floor Plan')
    
    # Add dimensions for each wall
    exterior_coords = list(polygon.exterior.coords)
    for i in range(len(exterior_coords) - 1):
        x1, y1 = exterior_coords[i]
        x2, y2 = exterior_coords[i+1]
        
        # Calculate wall length
        wall_length = numpy.sqrt((x2-x1)**2 + (y2-y1)**2)
        
        # Calculate midpoint for text placement
        midpoint_x = (x1 + x2) / 2
        midpoint_y = (y1 + y2) / 2
        
        # Calculate perpendicular offset for dimension text
        dx, dy = x2-x1, y2-y1
        length = numpy.sqrt(dx*dx + dy*dy)
        if length > 0:
            nx, ny = -dy/length * 0.2, dx/length * 0.2  # Perpendicular vector, 0.2 units away
        else:
            nx, ny = 0, 0.2
            
        # Add dimension text
        plt.text(midpoint_x + nx, midpoint_y + ny, f"{wall_length:.2f}m", 
                 ha='center', va='center', backgroundcolor='white', fontsize=8)
    
    # Add area calculation
    area = polygon.area
    centroid = polygon.centroid
    plt.text(centroid.x, centroid.y, f"Area: {area:.2f} m²", 
             ha='center', va='center', fontsize=10, fontweight='bold',
             bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.5'))
    
    plt.axis('equal')
    plt.legend(loc='upper right')
    plt.savefig(pdf_filename)
    plt.close()
