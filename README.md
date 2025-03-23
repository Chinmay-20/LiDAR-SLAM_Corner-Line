# Lidar-Based SLAM System in MATLAB

This project implements a Simultaneous Localization and Mapping (SLAM) system using Lidar data in MATLAB. It processes laser scans to build an occupancy grid map, detect lines and corners, and optimize the robot’s trajectory using a pose graph with loop closure detection.

# Process

1. Lidar Data Processing
2. Reads and preprocesses raw Lidar scans from .mat files.
3. Converts scans into world coordinates for mapping.
4. Feature Extraction
5. PCA-based line detection for extracting structured features.
6. Corner detection for identifying key environmental points.
7. Scan Matching (Fast and Brute-force methods) for pose estimation.
8. Pose Graph Optimization to correct accumulated errors.
9. Loop Closure Detection for reducing trajectory drift.
10. Creates an occupancy grid to represent the environment.
11. Updates map with detected lines and key scans.
12. Real-time plotting of robot trajectory, scan points, and pose graph.

# Project Structure

1. Main Files
main.m:	Entry point for executing the full SLAM pipeline.
helperinitialization.m:	Initializes map and SLAM parameters.
helperreadframe.m"	Reads and processes Lidar scan frames.

2. Core SLAM Components
helperclassposegraph.m:	Implements pose graph optimization.
helperclassnode.m:	Defines nodes (robot poses) in the pose graph.
helperclassedge.m:	Defines edges (constraints) between nodes.
helperloopclosuredetect.m:	Detects loop closures for correcting global map alignment.

3. Mapping & Feature Detection
helperoccugridcreate.m:	Creates an occupancy grid map from Lidar data.
helpermapextractlocalmap.m:	Extracts local map for scan matching.
helpercornerdetection.m:	Identifies corner points in the scan data.

5. Scan Matching & Transformation
helpermatchingfast.m:	Fast scan matching for quick pose estimation.
helpermatchingbrute.m:	Brute-force scan matching for refinement.
helpertransformrobotworld.m:	Transforms robot's local frame to world coordinates.

# Getting Started

Prerequisites
MATLAB (Recommended: R2018b or later).

Installation
1. Clone the repository
2. Open MATLAB and add the project folder to the path:
  addpath(genpath('path_to_project_folder'));

# Running the Full SLAM Pipeline

To execute the SLAM process, run:
main

# Performance

1. Execution Time: The SLAM pipeline processes the dataset in ~55.4 seconds.
2. Scan Matching Accuracy: Achieves 90% success rate in feature alignment.
3. Mapping Precision: Captures key environmental structures with minimal drift.
