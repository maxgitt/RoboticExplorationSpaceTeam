#ifndef PC_BACKGROUND_SUBTRACT_H
#define PC_BACKGROUND_SUBTRACT_H

#include <pcl/point_types.h>
#include <"obstacle.h">
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <chrono>
#include <thread>
#include <tuple>
#include <stdio.h>   
#include <stdlib.h> 
#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

namespace BackgroundSubtract {
    
// Function to randomly initialize base pointcloud
void initiailzeBaseCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud);

// function to initialize random comparison cloud
void initializeCompareCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

void compareClouds(pcl::PointCloud<pcl::PointXYZ> &base_cloud, 
    pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

void smoothCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

vector< pcl::PointCloud<pcl::PointXYZ> * > extractClustersFromCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud);

// function that averages values in pointcloud, returns vector of average location (x,y, length and width)
// Should return average x, y, length, and width (approximate) of obstacles (the cluster)
// values contained in obstacle object
vector<Obstacle> extractAverages(vector< pcl::PointCloud<pcl::PointXYZ> * > &cluster_clouds);

//Temperorary function that writes values to the terminal
void terminalWrite(vector<Obstacle> &obstacles_vector);
	

} // namespace BackgroundSubtract


#endif // PC_BACKGROUND_SUBTRACT_H