import ezdxf
import numpy as np
# Don't import plt here, we'll handle plotting in the main thread
import matplotlib
from shapely.geometry import Polygon, LineString
from dataclasses import dataclass
from PyQt5.QtCore import QThread, pyqtSignal
import math

@dataclass
class ComparisonMetrics:
    area_reference: float
    area_extracted: float
    area_difference: float
    area_difference_percent: float
    perimeter_reference: float
    perimeter_extracted: float
    perimeter_difference: float
    perimeter_difference_percent: float
    similarity_score: float

# Tolerance for detecting horizontal/vertical lines (in radians)
AXIS_ALIGNMENT_TOLERANCE = 0.1

@dataclass
class ComparisonResult:
    area_difference: float
    perimeter_difference: float
    wall_length_differences: list
    angle_differences: list
    iou_score: float

class ComparisonThread(QThread):
    finished = pyqtSignal(object)
    error_signal = pyqtSignal(str)

    def __init__(self, reference_dxf_path, extracted_dxf_path, scale_factor=1.0):
        super().__init__()
        self.reference_dxf_path = reference_dxf_path
        self.extracted_dxf_path = extracted_dxf_path
        self.scale_factor = scale_factor

    def run(self):
        try:
            # Compare the two DXF files
            result = compare_floorplans_detailed(self.reference_dxf_path, self.extracted_dxf_path)
            
            # Load polygons and prepare data for visualization
            reference_polygon = load_dxf_polygon(self.reference_dxf_path)
            extracted_polygon = load_dxf_polygon(self.extracted_dxf_path)
            metrics = compare_floorplans(self.reference_dxf_path, self.extracted_dxf_path)
            plot_data = prepare_comparison_data(reference_polygon, extracted_polygon, metrics)
            
            # Generate the comparison report
            report_path = 'comparison_report.pdf'
            generate_comparison_report_in_main_thread(plot_data, report_path)
            
            self.finished.emit((reference_polygon, extracted_polygon, metrics, plot_data, report_path))
        except Exception as e:
            self.error_signal.emit(str(e))

def load_dxf_polygon(dxf_path):
    """Load a DXF file and extract the main polygon."""
    doc = ezdxf.readfile(dxf_path)
    msp = doc.modelspace()
    
    vertices = []
    for entity in msp:
        if entity.dxftype() == 'LWPOLYLINE':
            vertices.extend(entity.get_points())
        elif entity.dxftype() == 'LINE':
            vertices.extend([(entity.dxf.start[0], entity.dxf.start[1])])
            vertices.extend([(entity.dxf.end[0], entity.dxf.end[1])])
    
    # Filter out any dimension lines or text entities
   
    
    # Create and clean polygon
    if vertices:
        poly = Polygon(vertices)
        if not poly.is_valid:
            poly = poly.buffer(0)
        return poly
    return None

def detect_principal_axes(polygon):
    """
    Detect the principal axes (main horizontal and vertical directions) of a polygon.
    Returns the angle (in radians) of the principal axis relative to the x-axis.
    """
    coords = list(polygon.exterior.coords)
    angles = []
    
    # Calculate angles of all segments
    for i in range(len(coords) - 1):
        p1, p2 = coords[i], coords[i+1]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = math.sqrt(dx*dx + dy*dy)
        
        # Skip very short segments
        if length < 0.1:
            continue
            
        # Calculate angle with x-axis (between -pi and pi)
        angle = math.atan2(dy, dx)
        
        # Normalize angle to be between 0 and pi/2 
        angle = abs(angle) % (math.pi/2)
        angles.append(angle)
    
    angle_counts = {}  # Initialize the dictionary to store angle frequencies
    # Find the most common angle 
    for angle in angles:
        # Round to nearest tolerance
        key = round(angle / AXIS_ALIGNMENT_TOLERANCE) * AXIS_ALIGNMENT_TOLERANCE
        angle_counts[key] = angle_counts.get(key, 0) + 1
    
    # Get the most common angle
    if angle_counts:
        principal_angle = max(angle_counts.items(), key=lambda x: x[1])[0]
        return principal_angle
    else:
        # Default to 0 if no angles found
        return 0.0

def align_polygons(reference_polygon, extracted_polygon):
    """
    Align the extracted polygon with the reference polygon by:
    1. Detecting principal axes in both polygons
    2. Rotating the extracted polygon to match the reference orientation
    3. Translating to align centroids
    
    Returns the aligned extracted polygon.
    """
    # Detect principal axes
    ref_angle = detect_principal_axes(reference_polygon)
    ext_angle = detect_principal_axes(extracted_polygon)
    
    # Calculate rotation angle needed
    rotation_angle = ref_angle - ext_angle
    
    # Get centroids for alignment
    ref_centroid = reference_polygon.centroid
    ext_centroid = extracted_polygon.centroid
    
    # Rotate extracted polygon around its centroid
    rotated_polygon = rotate_polygon(extracted_polygon, rotation_angle)
    
    # Translate to align centroids
    dx = ref_centroid.x - rotated_polygon.centroid.x
    dy = ref_centroid.y - rotated_polygon.centroid.y
    aligned_polygon = translate_polygon(rotated_polygon, dx, dy)
    
    return aligned_polygon

def rotate_polygon(polygon, angle):
    """Rotate a polygon around its centroid by the given angle (in radians)."""
    if angle == 0:
        return polygon
        
    centroid = polygon.centroid
    rotated_coords = []
    
    for x, y in polygon.exterior.coords:
        # Translate to origin (centroid)
        tx = x - centroid.x
        ty = y - centroid.y
        
        # Rotate
        rx = tx * math.cos(angle) - ty * math.sin(angle)
        ry = tx * math.sin(angle) + ty * math.cos(angle)
        
        # Translate back
        rotated_coords.append((rx + centroid.x, ry + centroid.y))
    
    return Polygon(rotated_coords)

def translate_polygon(polygon, dx, dy):
    """Translate a polygon by the given offsets."""
    translated_coords = [(x + dx, y + dy) for x, y in polygon.exterior.coords]
    return Polygon(translated_coords)

def calculate_iou(poly1, poly2):
    """Calculate Intersection over Union score."""
    if poly1.is_empty or poly2.is_empty:
        return 0.0
    intersection = poly1.intersection(poly2).area
    union = poly1.union(poly2).area
    return intersection / union if union > 0 else 0.0

def compare_floorplans(reference_dxf, generated_dxf):
    """Compare two floor plans and return metrics."""
    # Load polygons from DXF files
    ref_poly = reference_dxf if isinstance(reference_dxf, Polygon) else load_dxf_polygon(reference_dxf)
    gen_poly = generated_dxf if isinstance(generated_dxf, Polygon) else load_dxf_polygon(generated_dxf)
    
    # Align the extracted polygon with the reference polygon
    aligned_gen_poly = align_polygons(ref_poly, gen_poly)
    gen_poly = aligned_gen_poly
    
    # Calculate metrics
    area_ref = ref_poly.area
    area_gen = gen_poly.area
    area_diff = abs(area_ref - area_gen)
    area_diff_percent = (area_diff / area_ref) * 100 if area_ref > 0 else 0
    
    perimeter_reference = ref_poly.length
    perimeter_extracted = gen_poly.length
    perimeter_difference = abs(perimeter_reference - perimeter_extracted)
    perimeter_difference_percent = (perimeter_difference / perimeter_reference) * 100 if perimeter_reference > 0 else 0
    
    # Calculate similarity score (100 - average percentage difference)
    similarity_score = 100 - ((area_diff_percent + perimeter_difference_percent) / 2)
    
    return ComparisonMetrics(area_reference=area_ref, area_extracted=area_gen, area_difference=area_diff, 
                            area_difference_percent=area_diff_percent, perimeter_reference=perimeter_reference, 
                            perimeter_extracted=perimeter_extracted, perimeter_difference=perimeter_difference, 
                            perimeter_difference_percent=perimeter_difference_percent, similarity_score=similarity_score)

def compare_floorplans_detailed(reference_dxf, generated_dxf):
    """Compare two DXF floorplans and return detailed metrics."""
    ref_poly = load_dxf_polygon(reference_dxf)
    gen_poly = load_dxf_polygon(generated_dxf)
    
    # Align the extracted polygon with the reference polygon
    aligned_gen_poly = align_polygons(ref_poly, gen_poly)
    gen_poly = aligned_gen_poly
    
    if not ref_poly or not gen_poly:
        raise ValueError("Could not load one or both DXF files")
    
    # Calculate basic metrics
    area_difference = abs(ref_poly.area - gen_poly.area)
    perimeter_difference = abs(ref_poly.length - gen_poly.length)
    iou = calculate_iou(ref_poly, gen_poly)
    
    # Compare wall lengths
    ref_coords = list(ref_poly.exterior.coords)
    gen_coords = list(gen_poly.exterior.coords)
    
    wall_length_differences = []
    for i in range(len(ref_coords) - 1):
        ref_wall = LineString([ref_coords[i], ref_coords[i+1]])
        ref_len = ref_wall.length
        
        # Find closest matching wall
        min_diff = float('inf')
        for j in range(len(gen_coords) - 1):
            gen_wall = LineString([gen_coords[j], gen_coords[j+1]])
            diff = abs(ref_len - gen_wall.length)
            min_diff = min(min_diff, diff)
        wall_length_differences.append((ref_len, gen_wall.length))
    
    # Compare angles
    ref_angles = []
    gen_angles = []
    angle_differences = []
    
    for coords, angles in [(ref_coords, ref_angles), (gen_coords, gen_angles)]:
        for i in range(len(coords) - 2):
            v1 = np.array(coords[i+1]) - np.array(coords[i])
            v2 = np.array(coords[i+2]) - np.array(coords[i+1])
            angle = np.degrees(np.arctan2(np.cross(v1, v2), np.dot(v1, v2)))
            angles.append(angle)
    
    for ref_angle in ref_angles:
        min_diff = min(abs(ref_angle - gen_angle) for gen_angle in gen_angles)
        angle_differences.append(min_diff)
    
    return ComparisonResult(
        area_difference=area_difference,
        perimeter_difference=perimeter_difference,
        wall_length_differences=wall_length_differences,
        angle_differences=angle_differences,
        iou_score=iou
    )

def prepare_comparison_data(reference_polygon, extracted_polygon, metrics):
    """Prepare data for comparison visualization without creating plots."""
    # Extract polygon coordinates
    ref_x, ref_y = reference_polygon.exterior.xy
    
    # Align the extracted polygon with the reference polygon
    aligned_extracted_polygon = align_polygons(reference_polygon, extracted_polygon)
    extracted_polygon = aligned_extracted_polygon
    ext_x, ext_y = extracted_polygon.exterior.xy

    # Calculate centroids for alignment
    ref_centroid = (np.mean(ref_x), np.mean(ref_y))
    ext_centroid = (np.mean(ext_x), np.mean(ext_y))

    # Prepare metrics for bar chart
    metrics_labels = ['Area Diff (%)', 'Perimeter Diff (%)', 'Similarity (%)']
    metrics_values = [metrics.area_difference_percent, metrics.perimeter_difference_percent, metrics.similarity_score]
    
    # Prepare wall length comparison data - properly extract wall lengths from polygons
    wall_length_differences = []
    
    # Get coordinates of polygon exteriors
    ref_coords = list(reference_polygon.exterior.coords)
    ext_coords = list(extracted_polygon.exterior.coords)
    
    # Calculate wall lengths for reference polygon
    ref_wall_lengths = []
    for i in range(len(ref_coords) - 1):
        p1, p2 = ref_coords[i], ref_coords[i+1]
        length = LineString([p1, p2]).length
        ref_wall_lengths.append(length)
    
    # Calculate wall lengths for extracted polygon
    ext_wall_lengths = []
    for i in range(len(ext_coords) - 1):
        p1, p2 = ext_coords[i], ext_coords[i+1]
        length = LineString([p1, p2]).length
        ext_wall_lengths.append(length)
    
    # Create pairs of wall lengths (use min length to avoid index errors)
    min_walls = min(len(ref_wall_lengths), len(ext_wall_lengths))
    wall_length_differences = [(ref_wall_lengths[i], ext_wall_lengths[i]) for i in range(min_walls)]
    
    return (ref_x, ref_y, ext_x, ext_y, metrics_labels, metrics_values, wall_length_differences)

def generate_comparison_report_in_main_thread(plot_data, output_pdf, include_wall_details=True):
    """Generate a visual comparison report with the provided metrics."""
    fig, ax1 = matplotlib.pyplot.subplots(figsize=(12, 6))
    
    # Plot 1: Both polygons overlaid
    ref_x, ref_y = plot_data[0], plot_data[1]
    ext_x, ext_y = plot_data[2], plot_data[3]

    ax1.plot(ref_x, ref_y, 'b-', linewidth=2, label='Reference')
    ax1.plot(ext_x, ext_y, 'r--', linewidth=2, label='Extracted (Aligned)')
    
    # Add reference lines through centroid
    ax1.axhline(y=np.mean(ref_y), color='gray', linestyle=':', alpha=0.7)
    ax1.axvline(x=np.mean(ref_x), color='gray', linestyle=':', alpha=0.7)
    ax1.set_title('Floor Plan Comparison')
    ax1.set_aspect('equal')
    ax1.legend()

    # Add text annotation for metrics
    metrics_labels, metrics_values = plot_data[4], plot_data[5]
    metrics_text =  (f"Area Diff: {metrics_values[0]:.1f}%\n"
                   f"Perimeter Diff: {metrics_values[1]:.1f}%\n"
                   f"Similarity Score: {metrics_values[2]:.1f}%")
    ax1.text(0.02, 0.02, metrics_text, transform=ax1.transAxes, 
           bbox=dict(facecolor='white', alpha=0.8), fontsize=9)

    # Save the figure to the output PDF
    fig.tight_layout()
    matplotlib.pyplot.savefig(output_pdf)
    matplotlib.pyplot.close(fig)
