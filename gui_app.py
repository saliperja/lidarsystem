import sys, os
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QDir
from PyQt5.QtGui import QIcon, QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import matplotlib
from combine_pcd_frames import combine_pcd_files, save_combined_pcd 
import numpy as np
from floor_plan_extractor import load_and_preprocess_pcd, segment_walls, create_floor_plan, export_floor_plan_to_dxf, plot_and_save_floor_plan_pdf, save_processed_pcd, load_processed_pcd
from dxf_comparison import load_dxf_polygon, compare_floorplans, prepare_comparison_data, generate_comparison_report_in_main_thread, ComparisonMetrics

class ProcessingThread(QThread):
    finished = pyqtSignal(object)
    progress = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, point_cloud_path, params):
        super().__init__()
        self.point_cloud_path = point_cloud_path
        self.params = params

    def run(self):
        try:
            self.progress.emit("Loading point cloud...")
            point_cloud = load_and_preprocess_pcd(self.point_cloud_path)
            
            self.progress.emit("Segmenting walls...")
            points_xy, walls_indices, wall_models = segment_walls(point_cloud)
            
            self.progress.emit("Creating floor plan...")
            polygon = create_floor_plan(points_xy)
            
            self.finished.emit((points_xy, polygon))
        except Exception as e:
            self.error.emit(str(e))

class PcdMergeThread(QThread):
    finished = pyqtSignal(str)
    progress = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, pcd_files, output_path, voxel_size=None):
        super().__init__()
        self.pcd_files = pcd_files
        self.output_path = output_path
        self.voxel_size = voxel_size

    def run(self):
        try:
            self.progress.emit(f"Merging {len(self.pcd_files)} PCD files...")
            combined_pcd = combine_pcd_files(self.pcd_files, self.voxel_size)
            save_combined_pcd(combined_pcd, self.output_path)
            self.finished.emit(self.output_path)
        except Exception as e:
            self.error.emit(str(e))

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
            from shapely.geometry import Polygon
            import logging
            # Validate that both files exist
            if not os.path.exists(self.reference_dxf_path):
                raise FileNotFoundError(f"Reference DXF file not found: {self.reference_dxf_path}")
            if not os.path.exists(self.extracted_dxf_path):
                raise FileNotFoundError(f"Extracted DXF file not found: {self.extracted_dxf_path}")
            
            try:
                # Load reference polygon
                reference_polygon = load_dxf_polygon(self.reference_dxf_path)
                # Apply scale factor to reference polygon if needed
                if self.scale_factor != 1.0:
                    reference_polygon = Polygon([(p[0] * self.scale_factor, p[1] * self.scale_factor) 
                                               for p in reference_polygon.exterior.coords])
                logging.info(f"Successfully loaded reference DXF: {self.reference_dxf_path}")
            except Exception as e:
                raise RuntimeError(f"Error loading reference DXF: {str(e)}")
                
            try:
                # Load extracted polygon
                extracted_polygon = load_dxf_polygon(self.extracted_dxf_path)
                logging.info(f"Successfully loaded extracted DXF: {self.extracted_dxf_path}")
            except Exception as e:
                raise RuntimeError(f"Error loading extracted DXF: {str(e)}")
            
            # Compare the floor plans
            metrics = compare_floorplans(reference_polygon, extracted_polygon)
            
            # Instead of generating the report here, just prepare the data
            from dxf_comparison import prepare_comparison_data
            plot_data = prepare_comparison_data(reference_polygon, extracted_polygon, metrics)
            
            # Define report path but don't generate it yet
            report_path = os.path.join(QDir.tempPath(), "floor_plan_comparison.pdf")
            
            # Send the data back to the main thread for plotting
            self.finished.emit((reference_polygon, extracted_polygon, metrics, plot_data, report_path))
        except Exception as e:
            import traceback
            error_msg = f"Process failed: {str(e)}\n{traceback.format_exc()}"
            self.error_signal.emit(error_msg)

class FloorPlanApp(QWidget):
    def __init__(self):
        super().__init__()
        self.point_cloud_path = ""
        self.points_xy = None
        self.polygon = None
        self.scale_factor = 1.0  # Default scale factor
        self.reference_dxf_path = ""
        self.processing_thread = None
        self.pcd_files = []
        self.merged_pcd_path = ""
        self.initUI()

    def initUI(self):
        self.setWindowTitle('LiDAR Floor Plan Extractor')
        self.setMinimumSize(800, 600)
        self.layout = QVBoxLayout()

        # File operations group
        file_group = QGroupBox("File Operations")
        file_layout = QHBoxLayout()

        self.mergePcdBtn = QPushButton('Merge PCD Frames')
        self.mergePcdBtn.setIcon(QIcon.fromTheme("edit-copy"))
        self.mergePcdBtn.clicked.connect(self.mergePcdFrames)
        file_layout.addWidget(self.mergePcdBtn)
        
        self.loadBtn = QPushButton('Load Point Cloud Data Scan')
        self.loadBtn.setIcon(QIcon.fromTheme("document-open"))
        self.loadBtn.clicked.connect(self.loadPointCloudData)
        # Initially disable until we have a merged PCD or user loads one directly
        self.loadBtn.setEnabled(True)
        file_layout.addWidget(self.loadBtn)
        
        self.statusLabel = QLabel("No file loaded")
        file_layout.addWidget(self.statusLabel)
        file_group.setLayout(file_layout)
        self.layout.addWidget(file_group)

        # Preview area
        self.figure = Figure(figsize=(8, 8))
        self.canvas = FigureCanvas(self.figure)
        self.layout.addWidget(self.canvas)
        
        # Comparison results text area
        self.comparisonResults = QTextEdit()
        self.comparisonResults.setReadOnly(True)
        self.comparisonResults.setVisible(False)
        self.layout.addWidget(self.comparisonResults)
        
        # Progress bar
        self.progressBar = QProgressBar()
        self.progressBar.setVisible(False)
        self.layout.addWidget(self.progressBar)

        # Export options
        export_group = QGroupBox("Export Options")
        export_group.setObjectName("Export Options")
        export_layout = QHBoxLayout()
        
        self.exportDWG = QCheckBox('Export DWG')
        self.exportDWG.setChecked(True)
        self.exportPDF = QCheckBox('Export PDF')
        self.exportPDF.setChecked(True)
        export_layout.addWidget(self.exportDWG)
        export_layout.addWidget(self.exportPDF)
        
        self.exportBtn = QPushButton('Export')
        self.exportBtn.clicked.connect(self.exportFiles)
        self.exportBtn.setEnabled(False)
        
        self.compareBtn = QPushButton('Compare with Reference DXF')
        self.compareBtn.clicked.connect(self.compareWithReference)
        self.compareBtn.setEnabled(False)
        export_layout.addWidget(self.compareBtn)
        
        export_layout.addWidget(self.exportBtn)
        
        export_group.setLayout(export_layout)
        self.layout.addWidget(export_group)

        # Create the export comparison button AFTER setting the layout
        if not hasattr(self, 'exportComparisonBtn'):
            # Create the export comparison button
            self.exportComparisonBtn = QPushButton('Export Comparison Report')
            self.exportComparisonBtn.clicked.connect(self.exportComparisonReport)
            self.exportComparisonBtn.setEnabled(False)  # Initially disabled
            
            # Add it to the export options group
            export_layout.addWidget(self.exportComparisonBtn)

        # Exit button
        self.exitBtn = QPushButton('Exit Application')
        self.exitBtn.setIcon(QIcon.fromTheme("application-exit"))
        self.exitBtn.clicked.connect(self.exitApplication)
        self.exitBtn.setStyleSheet("padding: 8px;")
        
        # Add exit button to layout
        self.layout.addWidget(self.exitBtn)

        self.setLayout(self.layout)

    def mergePcdFrames(self):
        # If we already have PCD files from conversion, use those
        if not self.pcd_files:
            # Otherwise, let the user select PCD files
            options = QFileDialog.Options()
            fileNames, _ = QFileDialog.getOpenFileNames(
                self, "Select PCD Files to Merge", "", 
                "PCD Files (*.pcd);;All Files (*)", 
                options=options
            )
            if fileNames:
                self.pcd_files = fileNames
        
        if self.pcd_files:
            # Ask for output file
            options = QFileDialog.Options()
            output_file, _ = QFileDialog.getSaveFileName(
                self, "Save Merged PCD As", "", 
                "PCD Files (*.pcd);;All Files (*)", 
                options=options
            )
            if output_file:
                self.statusLabel.setText(f"Merging {len(self.pcd_files)} PCD files...")
                self.progressBar.setVisible(True)
                
                # Start merge in a separate thread
                self.pcd_merge_thread = PcdMergeThread(self.pcd_files, output_file, voxel_size=0.01)
                self.pcd_merge_thread.progress.connect(self.updateProgress)
                self.pcd_merge_thread.finished.connect(self.onPcdMergeComplete)
                self.pcd_merge_thread.error.connect(self.onProcessingError)
                self.pcd_merge_thread.start()

    def onPcdMergeComplete(self, output_path):
        self.merged_pcd_path = output_path
        self.progressBar.setVisible(False)
        self.statusLabel.setText(f"Merge complete. Saved to: {os.path.basename(output_path)}")
        
        # Enable the load button
        self.loadBtn.setEnabled(True)
        
        # Show a message with the results
        reply = QMessageBox.question(self, "Merge Complete", 
                               f"Successfully merged PCD files.\nSaved to: {output_path}\n\nDo you want to load this file now?",
                               QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        if reply == QMessageBox.Yes:
            self.point_cloud_path = output_path
            self.startProcessing()

    def loadPointCloudData(self):
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Select Point Cloud Data Scan", "", 
            "Point Cloud Files (*.pcd);;All Files (*)", 
            options=options
        )
        if fileName:
            self.point_cloud_path = fileName
            self.statusLabel.setText(f"Processing: {os.path.basename(fileName)}")
            self.progressBar.setVisible(True)
            self.startProcessing()

    def startProcessing(self):
        self.processing_thread = ProcessingThread(self.point_cloud_path, {})
        self.processing_thread.progress.connect(self.updateProgress)
        self.processing_thread.finished.connect(self.onProcessingComplete)
        self.processing_thread.error.connect(self.onProcessingError)
        self.processing_thread.start()
        
    def updateProgress(self, message):
        self.statusLabel.setText(message)
        
    def onProcessingComplete(self, result):
        self.points_xy, self.polygon = result
        self.updatePreview()
        self.exportBtn.setEnabled(True)
        self.compareBtn.setEnabled(True)
        self.progressBar.setVisible(False)
        self.statusLabel.setText("Processing complete!")
        
    def onProcessingError(self, error_message):
        QMessageBox.critical(self, "Error", f"Processing failed: {error_message}")
        self.progressBar.setVisible(False)
        
    def updatePreview(self):
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.scatter(self.points_xy[:, 0], self.points_xy[:, 1], s=1, c='gray', label='Point Cloud')
        outline_x, outline_y = self.polygon.exterior.xy
        ax.plot(outline_x, outline_y, 'r-', linewidth=2, label='Floor Plan Outline')

        # Calculate area in square meters
        area = self.polygon.area * self.scale_factor ** 2
        ax.set_title(f"Floor Plan (Area: {area:.2f} m²)")
        ax.set_aspect('equal')
        ax.legend()
        self.canvas.draw()

    def compareWithReference(self):
        """Compare the extracted floor plan with a reference DXF file"""
        if not self.polygon:
            QMessageBox.warning(self, "Warning", "No floor plan to compare. Please load and process a point cloud first.")
            return
            
        # Ask user to select reference DXF file
        options = QFileDialog.Options()
        self.reference_dxf_path, _ = QFileDialog.getOpenFileName(
            self, "Select Reference DXF File", "", 
            "DXF Files (*.dxf);;All Files (*)", 
            options=options
        )
        
        if not self.reference_dxf_path:
            return
        
        # Display a note about units instead of asking
        note_dialog = QMessageBox(self)
        note_dialog.setWindowTitle("DXF Units Information")
        note_dialog.setText("The reference DXF file should be in meters (m) for accurate comparison.\n\nIf your DXF is in a different unit, please convert it before uploading.")
        note_dialog.setIcon(QMessageBox.Information)
        note_dialog.exec_()
        
        # Always use meters (no scale factor)
        scale_factor = 1.0
        
        try:
            # First export the current floor plan to a temporary DXF file
            temp_dxf = os.path.join(QDir.tempPath(), "temp_floorplan.dxf")
            export_floor_plan_to_dxf(self.polygon, temp_dxf)
            
            # Show progress
            self.statusLabel.setText("Comparing floor plans...")
            self.progressBar.setVisible(True)
            
            # Run comparison in a separate thread
            self.comparison_thread = ComparisonThread(self.reference_dxf_path, temp_dxf, scale_factor)
            self.comparison_thread.finished.connect(self.onComparisonComplete)
            self.comparison_thread.error_signal.connect(self.onProcessingError)
            self.comparison_thread.start()
        except Exception as e:
            import traceback
            error_msg = f"Error preparing comparison: {str(e)}\n{traceback.format_exc()}"
            QMessageBox.critical(self, "Error", error_msg)
            self.progressBar.setVisible(False)
            self.statusLabel.setText("Comparison failed")
            return

    def onComparisonComplete(self, result):
        """Handle completion of floor plan comparison"""
        reference_polygon, extracted_polygon, metrics, plot_data, report_path = result
        self.progressBar.setVisible(False)
        self.plot_data = plot_data  # Store plot data as an instance attribute
        self.statusLabel.setText("Comparison complete!")
        self.comparison_report_path = report_path  # Store for export
        
        # Show comparison results
        self.comparisonResults.setVisible(True)
        self.comparisonResults.setHtml(f"""
            <h3>Floor Plan Comparison Results</h3>
            <p><b>Overall Similarity Score:</b> {metrics.similarity_score:.1f}%</p>
            
            <h4>Unit Information:</h4>
            <p>Reference DXF units were converted to match extracted floor plan (meters).</p>
            <p>If the comparison looks incorrect, try selecting a different unit when uploading.</p>
            
            <h4>Area Comparison:</h4>
            <ul>
                <li>Reference: {metrics.area_reference:.2f} m²</li>
                <li>Extracted: {metrics.area_extracted:.2f} m²</li>
                <li>Difference: {metrics.area_difference:.2f} m² ({metrics.area_difference_percent:.1f}%)</li>
            </ul>
            
            <h4>Perimeter Comparison:</h4>
            <ul>
                <li>Reference: {metrics.perimeter_reference:.2f} m</li>
                <li>Extracted: {metrics.perimeter_extracted:.2f} m</li>
                <li>Difference: {metrics.perimeter_difference:.2f} m ({metrics.perimeter_difference_percent:.1f}%)</li>
            </ul>
        """)
        
        try:
            # Clear the current figure
            self.figure.clear()

            # Create a figure with two subplots in the main thread
            ax1 = self.figure.add_subplot(121)  # 1 row, 2 cols, first plot
            ax2 = self.figure.add_subplot(122)  # 1 row, 2 cols, second plot

            # Plot 1: Both polygons overlaid
            ref_x, ref_y = plot_data[0], plot_data[1]
            ext_x, ext_y = plot_data[2], plot_data[3]

            ax1.plot(ref_x, ref_y, 'b-', linewidth=2, label='Reference')
            ax1.plot(ext_x, ext_y, 'r--', linewidth=2, label='Extracted')
            ax1.set_title(f"Floor Plan Comparison (Similarity: {metrics.similarity_score:.1f}%)")
            ax1.set_aspect('equal')
            ax1.legend()

            # Plot 2: Metrics summary
            metrics_labels, metrics_values = plot_data[4], plot_data[5]
            ax2.bar(metrics_labels, metrics_values)
            ax2.set_title('Comparison Metrics')
            ax2.set_ylim(0, 100)

            # Update the canvas
            self.figure.tight_layout()
            self.canvas.draw()
            
            # Enable the export comparison button
            self.exportComparisonBtn.setEnabled(True)
            
        except Exception as e:
            QMessageBox.warning(self, "Visualization Error", f"Error displaying comparison: {str(e)}\n\nTry restarting the application.")

    def exportComparisonReport(self):
        """Export the comparison report to a user-selected location"""
        if not hasattr(self, 'plot_data') or not hasattr(self, 'comparison_report_path'):
            QMessageBox.warning(self, "Warning", "No comparison report available. Please run a comparison first.")
            return
            
        options = QFileDialog.Options()
        save_path, _ = QFileDialog.getSaveFileName(
            self, "Save Comparison Report", "", "PDF Files (*.pdf);;All Files (*)", options=options)
        
        if save_path:
            generate_comparison_report_in_main_thread(self.plot_data, save_path)
            QMessageBox.information(self, "Export Complete", f"Comparison report saved to: {save_path}")

    def exportFiles(self):
        if not self.polygon:
            QMessageBox.warning(self, "Warning", "No floor plan to export. Please load and process a point cloud first.")
            return
            
        if not (self.exportDWG.isChecked() or self.exportPDF.isChecked()):
            QMessageBox.warning(self, "Warning", "Please select at least one export format (DWG or PDF).")
            return
            
        export_dir = QFileDialog.getExistingDirectory(
            self, "Select Export Directory"
        )
        if export_dir:
            if self.exportDWG.isChecked():
                export_floor_plan_to_dxf(
                    self.polygon,
                    os.path.join(export_dir, "floorplan.dxf")
                )
            if self.exportPDF.isChecked():
                plot_and_save_floor_plan_pdf(
                    self.points_xy,
                    self.polygon,
                    os.path.join(export_dir, "floorplan.pdf")
                )
            QMessageBox.information(self, "Success", "Export complete!")

    def exitApplication(self):
        """Handles the exit button click with a confirmation dialog"""
        reply = QMessageBox.question(self, 'Exit Confirmation',
                                     'Are you sure you want to exit the application?',
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            QApplication.quit()

if __name__ == "__main__":
    # Create the Qt Application
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = FloorPlanApp()
    window.show()
    
    # Start the event loop
    sys.exit(app.exec_())
