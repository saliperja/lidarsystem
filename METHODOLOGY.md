# LiDAR Floor Plan Extraction System - Technical Methodology

This document provides a detailed technical explanation of the algorithms and approaches used in the LiDAR Floor Plan Extraction System.

## System Architecture Overview

The system follows a pipeline architecture with several key processing stages:

1. **Data Acquisition & Preprocessing**: Loading and cleaning point cloud data
2. **Wall Segmentation**: Identifying vertical surfaces representing walls
3. **Floor Plan Generation**: Creating a 2D representation of the room boundaries
4. **Export & Comparison**: Converting to industry formats and comparing with references

Each stage employs specific algorithms optimized for architectural feature extraction from point cloud data.

## Detailed Methodology

### 1. Point Cloud Processing

#### 1.1 Data Loading and Initial Preprocessing

The system begins by loading point cloud data from PCD files using the Open3D library. Initial preprocessing includes:

- **Statistical Outlier Removal**: Points that are statistical outliers (too far from neighboring points) are removed to clean the data
- **Voxel Downsampling**: The point cloud is downsampled using a voxel grid filter (default size 0.04m) to reduce computational complexity while preserving structural information

#### 1.2 Point Cloud Merging (Optional)

For scans with multiple frames, the system provides functionality to merge point clouds:

- **Frame Selection**: Users can select multiple PCD frames from a scan sequence
- **Registration**: Point clouds are aligned using Iterative Closest Point (ICP) algorithm with point-to-plane error metrics
- **Merging**: Aligned point clouds are combined into a single coherent point cloud
- **Downsampling**: The merged cloud is downsampled to manage memory and processing requirements

### 2. Wall Segmentation

#### 2.1 Plane Detection with RANSAC

The system identifies planar surfaces (potential walls) using Random Sample Consensus (RANSAC):

- **Multi-threshold Approach**: Multiple distance thresholds (0.02m, 0.05m, 0.1m, 0.12m) are tested to find optimal plane fits
- **Normal Filtering**: Only planes with normals approximately perpendicular to the z-axis (vertical walls) are considered
- **Iterative Extraction**: Multiple wall planes are extracted iteratively, with previously detected points removed from consideration

#### 2.2 Wall Clustering with DBSCAN

After plane detection, Density-Based Spatial Clustering of Applications with Noise (DBSCAN) is applied:

- **Spatial Clustering**: Points belonging to the same wall are grouped together (eps=0.8, min_points=30)
- **Cluster Filtering**: Small clusters are filtered out to remove noise
- **Minimum Wall Length**: Only wall segments longer than a threshold (default 0.8m) are retained

### 3. Floor Plan Generation

#### 3.1 2D Projection

The 3D wall points are projected onto the XY plane to create a 2D representation:

- **Dimension Reduction**: The Z-coordinate is discarded, converting 3D points to 2D
- **Point Cloud Aggregation**: All wall points from different segments are combined

#### 3.2 Boundary Extraction

A polygon representing the room boundary is extracted:

- **Minimum Rotated Rectangle**: The system computes the minimum-area rotated rectangle that contains all wall points
- **Shapely Library**: The MultiPoint and Polygon classes from Shapely are used for geometric operations

### 4. Export and Comparison

#### 4.1 DXF Export

The floor plan is exported to industry-standard DXF format:

- **Line Generation**: Polygon edges are converted to DXF LINE entities
- **Dimension Annotation**: Wall lengths are calculated and added as text annotations
- **Area Calculation**: Total floor area is calculated and added to the drawing

#### 4.2 PDF Visualization

A visual representation is generated using Matplotlib:

- **Point Cloud Visualization**: Wall points are plotted as scatter points
- **Boundary Outline**: The floor plan polygon is outlined
- **Dimension Annotation**: Wall lengths and room area are labeled

#### 4.3 Reference Comparison

The system can compare extracted floor plans with reference DXF files:

- **Polygon Alignment**: The extracted floor plan is aligned with the reference using principal axes detection
- **Metric Calculation**: Area difference, perimeter difference, and similarity score are computed
- **Visual Comparison**: Both floor plans are visualized together with highlighted differences

## Algorithm Parameters

The system uses several configurable parameters that affect processing quality and performance:

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| VOXEL_SIZE | 0.04 | Voxel grid size for downsampling (meters) |
| RANSAC_DISTANCE_CANDIDATES | [0.02, 0.05, 0.1, 0.12] | Distance thresholds for RANSAC plane detection (meters) |
| MIN_WALL_LENGTH | 0.8 | Minimum length for a wall segment to be considered (meters) |
| DBSCAN_EPS | 0.8 | Maximum distance between points in a DBSCAN cluster (meters) |
| DBSCAN_MIN_POINTS | 30 | Minimum number of points to form a DBSCAN cluster |
| NORMAL_TOLERANCE | 0.2 | Maximum deviation from vertical for a plane to be considered a wall |

## Technical Implementation

The system is implemented in Python with the following key libraries:

- **Open3D**: For point cloud processing, visualization, and geometric operations
- **NumPy**: For efficient numerical operations on point data
- **Shapely**: For 2D geometric operations and polygon manipulation
- **ezdxf**: For DXF file creation and manipulation
- **Matplotlib**: For visualization and PDF generation
- **PyQt5**: For the graphical user interface

The modular architecture allows for easy extension and customization of individual processing steps.
