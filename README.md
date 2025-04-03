# LiDAR Floor Plan Extraction System

A comprehensive application for automatically generating floor plans from LiDAR point cloud data with comparison capabilities for reference designs. This tool bridges the gap between 3D scanning technology and traditional 2D architectural documentation.

## Features

  - Load and process PCD (Point Cloud Data) files from LiDAR scanners
  - Merge multiple scan frames into a single coherent point cloud
  - Automatic noise filtering and downsampling for efficient processing

  - Intelligent wall detection using Random Sample Consensus (RANSAC) and Density-Based Spatial Clustering of Applications with Noise (DBSCAN) algorithms
  - Accurate room boundary extraction with minimum rotated rectangle
  - Automatic dimension calculation and area measurement

  - DXF export for CAD software compatibility (AutoCAD, Revit, etc.)
  - PDF visualization with dimensions and area information
  - Customizable export options

  - Compare extracted floor plans with reference DXF files
  - Detailed metrics including area difference, perimeter difference, and similarity score
  - Visual comparison with overlay visualization
  - Comprehensive comparison reports in PDF format

  - Intuitive PyQt5-based graphical user interface
  - Real-time visualization of point clouds and floor plans
  - Progress tracking for long-running operations
  - Multithreaded processing to maintain responsive UI

## System Requirements


## Installation

1. Clone this repository or download the source code
2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```
3. Launch the application:
   ```
   python gui_app.py
   ```

## Usage Guide

### Loading Point Cloud Data

1. Click "Load Point Cloud Data Scan" and select a PCD file
2. The system will automatically process the point cloud and extract a floor plan
3. The result will be displayed in the preview area

### Merging Multiple Scans

1. Click "Merge PCD Frames" and select multiple PCD files
2. Specify an output location for the merged point cloud
3. Once complete, you can load the merged file for processing

### Exporting Floor Plans

1. After processing a point cloud, select your desired export formats (DXF/PDF)
2. Click "Export" and choose a destination folder
3. The floor plan will be exported in the selected formats

### Comparing with Reference Designs

1. Process a point cloud to extract a floor plan
2. Click "Compare with Reference DXF"
3. Select a reference DXF file
4. Review the comparison metrics and visualization
5. Optionally export a comparison report

## Technical Details

The system uses a pipeline of algorithms to process point cloud data:

1. **Preprocessing**: Statistical outlier removal and voxel downsampling
2. **Wall Segmentation**: RANSAC plane detection with normal filtering
3. **Clustering**: DBSCAN to identify distinct wall segments
4. **Floor Plan Generation**: Minimum rotated rectangle on projected wall points
5. **Export**: Conversion to DXF/PDF with dimensions and area information

For more detailed information about the algorithms and methodology, see [METHODOLOGY.md](METHODOLOGY.md).

## Troubleshooting

For common issues and solutions, please refer to the [HELP.md](HELP.md) file.

## License

This project is licensed under the MIT License - see the LICENSE file for details.



