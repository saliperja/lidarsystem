# LiDAR Floor Plan Extraction System - Help Guide

This document provides detailed instructions for using the LiDAR Floor Plan Extraction System, along with troubleshooting tips and parameter explanations.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Detailed Usage Instructions](#detailed-usage-instructions)
3. [Parameter Explanations](#parameter-explanations)
4. [Troubleshooting](#troubleshooting)
5. [FAQ](#faq)

## Getting Started

### First Launch

When you first launch the application, you'll see the main interface with several buttons and a preview area. The interface is divided into three main sections:

1. **File Operations**: Buttons for loading and merging point cloud data
2. **Preview Area**: Visualization of point clouds and floor plans
3. **Export Options**: Controls for exporting and comparing floor plans

### System Check

Before processing large point clouds, it's recommended to verify your system capabilities:

1. Check available RAM (8GB minimum, 16GB recommended)
2. Ensure graphics drivers are up to date
3. Verify that all dependencies are correctly installed

## Detailed Usage Instructions

### Loading Point Cloud Data

1. Click "Load Point Cloud Data Scan"
2. Select a PCD file in the file dialog
3. The system will automatically:
   - Load and preprocess the point cloud
   - Segment walls using RANSAC and DBSCAN
   - Create a floor plan from the segmented walls
   - Display the result in the preview area

### Merging Multiple PCD Frames

If you have multiple PCD files from different scan:

1. Click "Merge PCD Frames"
2. Select all PCD files you want to merge
3. Specify an output location for the merged point cloud
4. The system will align and merge the frames
5. Once complete, you can load the merged point cloud

### Exporting Floor Plans

After processing a point cloud:

1. Check the export options you want (DXF and/or PDF)
2. Click "Export"
3. Select a directory to save the files
4. The system will generate:
   - DXF file (if selected): Vector format for CAD software
   - PDF file (if selected): Visual representation with measurements

### Comparing with Reference Designs

To compare an extracted floor plan with a reference design:

1. Process a point cloud to extract a floor plan
2. Click "Compare with Reference DXF"
3. Select a reference DXF file
4. The system will:
   - Align the extracted floor plan with the reference
   - Calculate comparison metrics
   - Display a visual comparison
   - Show detailed metrics in the text area
5. Optionally, click "Export Comparison Report" to save a PDF report

## Parameter Explanations

### Point Cloud Processing

The system uses several parameters that affect point cloud processing quality:

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| VOXEL_SIZE | 0.04 | Voxel grid size for downsampling (meters). Lower values preserve more detail but increase processing time. |
| STATISTICAL_OUTLIER_NB_NEIGHBORS | 20 | Number of neighbors to consider for outlier removal. |
| STATISTICAL_OUTLIER_STD_RATIO | 2.0 | Standard deviation threshold for outlier removal. Points with distances larger than this will be removed. |

### Wall Segmentation

Parameters that control wall detection and segmentation:

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| RANSAC_DISTANCE_CANDIDATES | [0.02, 0.05, 0.1, 0.12] | Distance thresholds for RANSAC plane detection (meters). Multiple values are tested to find optimal plane fits. |
| MIN_WALL_LENGTH | 0.8 | Minimum length for a wall segment to be considered (meters). Shorter segments are filtered out. |
| DBSCAN_EPS | 0.8 | Maximum distance between points in a DBSCAN cluster (meters). Controls how points are grouped into wall segments. |
| DBSCAN_MIN_POINTS | 30 | Minimum number of points to form a DBSCAN cluster. Helps filter out noise. |
| NORMAL_TOLERANCE | 0.2 | Maximum deviation from vertical for a plane to be considered a wall. Lower values enforce stricter verticality. |
| RANDOM_SEED | 42 | Fixed random seed for consistent results across multiple runs. |

## Troubleshooting

### Common Issues and Solutions

#### Application Crashes During Processing

**Possible causes:**

**Solutions:**
1. Close other applications to free up memory
2. Check the point cloud file for corruption
3. Verify all dependencies are installed correctly

#### No Walls Detected

**Possible causes:**

**Solutions:**
1. Verify the point cloud contains wall data
2. Try adjusting RANSAC_DISTANCE_CANDIDATES in floor_plan_extractor.py
3. Reduce NORMAL_TOLERANCE to detect more potential walls

#### Poor Quality Floor Plan

**Possible causes:**

**Solutions:**
1. Use a higher resolution scan or merge multiple scans
2. Apply additional preprocessing to remove noise
3. Adjust MIN_WALL_LENGTH to capture more wall segments

#### DXF Export Issues

**Possible causes:**

**Solutions:**
1. Reinstall ezdxf library
2. Check for geometry issues in the floor plan
3. Try exporting to PDF first to verify the floor plan is valid

### Error Messages


## FAQ

### General Questions

**Q: What LiDAR scanners are supported?**  
A: Any scanner that can produce PCD (Point Cloud Data) files is supported. This includes most commercial LiDAR scanners like Velodyne, Ouster, and Intel RealSense depth cameras.

**Q: How accurate are the floor plans?**  
A: Accuracy depends on the quality of the input point cloud. With good quality scans, the system typically achieves accuracy within 1-2% of actual dimensions.

**Q: Can the system handle multiple rooms?**  
A: Yes, but best results are achieved when scanning one room at a time and then combining the floor plans.

**Q: What's the maximum point cloud size the system can handle?**  
A: This depends on your system's RAM. With 16GB RAM, point clouds with up to 10 million points can be processed.

### Technical Questions

**Q: Does the system detect doors and windows?**  
A: The current version focuses on wall detection. Door and window detection is planned for future versions.

**Q: Can I use the system with RGB-D cameras like Kinect?**  
A: Yes, as long as you can convert the depth data to PCD format.

**Q: How does the comparison algorithm work?**  
A: The comparison aligns the extracted floor plan with the reference using principal axes detection, then calculates metrics like area difference, perimeter difference, and Intersection over Union (IoU).

**Q: Can I batch process multiple point clouds?**  
A: The current GUI doesn't support batch processing, but the underlying functions can be used in scripts for batch processing.

### File Format Questions

**Q: What DXF version is used for export?**  
A: The system exports DXF files in R2010 format for maximum compatibility.

**Q: Can I import the DXF files into AutoCAD/Revit/etc.?**  
A: Yes, the exported DXF files are compatible with most CAD software.

**Q: What units are used in the exported files?**  
A: All measurements are in meters.

**Q: Can I export to other formats besides DXF and PDF?**  
A: Currently only DXF and PDF are supported. Additional formats may be added in future versions.
