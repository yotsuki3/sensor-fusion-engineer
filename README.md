# Sensor Fusion Engineer Nanodegree Program in Udacity

## 1_LidarObstacleDetection
This module performs obstacle detection in a simulated highway environment using LiDAR data. I implemented the segmentation of ground and obstacle point clouds with 3D RANSAC algorithm, Euclidean clustering with KD-Tree data structure, and bounding boxes. In the final result shown below, the ground plane and obstacles (cars and poles) are accurately detected. The process is as follows:
#### Filter point clouds
- Reduce the number of points in cloud to process faster
- Use voxel grid filtering, which creates a cubic grid and filters the cloud by only leaving a single point per voxel cube
- Remove any points outside of box areas

#### Segment the filtered cloud into two parts, ground and obstacles
- Find and filter out point clouds that belong to the ground plane
- For this purpose, implement Random Sample Consensus (RANSAC) algorithm for detecting a plane in a 3D point cloud

#### Cluster the obstacle cloud
- Performe Euclidean clustering by using a nearest neighbor search
- For efficient search, implement a KD-Tree data structure. A KD-Tree is a binary tree that splits points between alternating axes
- Add bounding boxes around clusters

<img src="1_LidarObstacleDetection/media/final_project.gif" width="800" height="400" />



## 2_Camera
This module consits of two projects: "2D_Feature_Matching and 3D_Object_Tracking " and "3D_Object_Tracking".

#### 2D_Feature_Matching
This midterm project performs detailed comparison of feature tracking algorithms. A various detectors (FAST, BRISK, ORB, AKAZE, SIFT) and descriptors (BRIEF, ORB, FREAK, AKAZE, SIFT) were tested to evaluate their performances on keypoints detection.
<img src="2_Camera/media/midterm_project.PNG" width="800" height="150" />

#### 3D_Object_Tracking
The goal of this final project is to estimate the time-to-collision (TTC) using LiDAR and camara data. The process is as follows:
- Implement keypoint detection and descriptor extraction
- Add bounding boxes of objects using YOLO
- Project 3D LiDAR point cloud data onto the bounding box of 2D camera images
- Calculate TTC from two consecutive images
- Compare TTC from camera with TTC from LiDAR.

<img src="2_Camera/media/final_project.gif" width="800" height="400" />

## 3_Radar
The goal of this module is to estimate the range and velocity of a moving object using a radar sensor. The process is as follows:
- Send a Frequency Modulated Continuous Wave (FWCW) radar signals to a moving object
- Construct beat signals based on transmitted and received signals
- Perform 2D FFT on beat signals to generate a range-doppler map
- Implement Constant False Alarm Rate (CFAR) on the range-doppler map to avoid false alarms due to clutter signals that are generally produced by the reflections from the ground, sea, buildings, trees, rain, fog etc.
The following figures show range-doppler maps with and without CFAR. The estimated velocity and range are nearly equal to those set in simulation.

<img src="3_Radar/media/final_project.PNG" width="800" height="300" />

## 4_KalmanFilter
This module performs sensor fusion with LiDAR and Radar data using Unscented Kalman filter (UKF). The process is as follows:
- Detection of each moving car is given
- LiDAR and Radar data are continuously obtained at different sampling rate and fed into UKF pipeline
- UKF performes estimation of the position and velocity of each car through prediction and updating cycles
- The image below depicts a sensor fusion results in a simulated highway environment. The red spheres represent LiDAR detection and the pink arrows represent radar measurements. The green spheres represent the predicted path by UKF.
<img src="4_KalmanFilters/media/final_project.gif" width="800" height="400" />
