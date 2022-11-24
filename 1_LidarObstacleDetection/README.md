# Steps for Lidar obstacle detection

## Filter point clouds
- Reduce the number of points in cloud to process faster
- Use voxel grid filtering, which creates a cubic grid and filters the cloud by only leaving a single point per voxel cube
- Remove any points outside of box areas

## Segment the filtered cloud into two parts, ground and obstacles
- Find and filter out point clouds that belong to the ground plane
- For this purpose, implement Random Sample Consensus (RANSAC) algorithm for detecting a plane in a 3D point cloud

## Cluster the obstacle cloud
- Performe Euclidean clustering by using a nearest neighbor search
- For efficient search, implement a KD-Tree data structure. A KD-Tree is a binary tree that splits points between alternating axes
- Add bounding boxes around clusters

<img src="img/final-project.gif" width="800" height="400" />

