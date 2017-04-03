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
#include "pc_background_subtract.h"
#include <"obstacle.h">


using namespace std;

 /*

 // TODO:
 // Grabber
 // Kinect Error Modelling
 // Extract clusters
  
 */
namespace BackgroundSubtract{ 

  int main (int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZ> base_cloud;

    initiailzeBaseCloud(base_cloud);
      
    
    while(1){
        pcl::PointCloud<pcl::PointXYZ> compare_cloud; 

        initializeCompareCloud(base_cloud, compare_cloud);

        // compare_cloud now has "differences"
        compareClouds(base_cloud, compare_cloud);
        
        // Smoothing kernel on point cloud
        smoothCloud(compare_cloud);

        // Extract clusters ; return vector containing cluster locations
        vector< pcl::PointCloud<pcl::PointXYZ> * > cluster_clouds;
        cluster_clouds = extractClustersFromCloud(compare_cloud);

        vector<Obstacle> obstacles_vector;
        obstacles_vector = extractAverages(cluster_clouds);

        //publish the vector
        //temporarily write into the terminal
        terminalWrite(obstacles_vector);

        // Slow down data rate
        this_thread::sleep_for(chrono::seconds(2)); 
        
    
    }
    
    return (0);
  }

} // namespace BackgroundSubtract

