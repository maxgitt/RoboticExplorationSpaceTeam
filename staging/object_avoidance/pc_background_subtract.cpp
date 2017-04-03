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

namespace BackgroundSubtract{ 

  // Function to randomly initialize base pointcloud
  void initiailzeBaseCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud){
    // Faking point cloud temporarily. 
    // Fill in the cloud data
    base_cloud.width = 5;
    base_cloud.height = 1;
    base_cloud.is_dense = false;
    base_cloud.points.resize (base_cloud.width * base_cloud.height);
    
    int cols = 1;
    
    for (size_t i = 0; i < base_cloud.points.size (); ++i)
    {
      base_cloud.points[i].x = i % cols;
      base_cloud.points[i].y = i / cols;
      base_cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
      cols++;
    }
    
    cols = 1;

    // Write base_cloud to pcd file for testing
    pcl::io::savePCDFileASCII ("base_pcd.pcd", base_cloud);
    cout << "Saved " << base_cloud.points.size () << " data points to base_pcd.pcd." << endl;

    // Write base_cloud to terminal
    cout << "x" << " " << "y" << " " << "z" << endl; // I know this could be one line, but this is easier to read for me. 
    for (size_t i = 0; i < base_cloud.points.size (); ++i){
      cout << " " << base_cloud.points[i].x << " " << base_cloud.points[i].y << " " << base_cloud.points[i].z << endl;
    }
  }

  // function to initialize random comparison cloud
  void initializeCompareCloud(pcl::PointCloud<pcl::PointXYZ> &base_cloud, pcl::PointCloud<pcl::PointXYZ> &compare_cloud){

    compare_cloud.width = 5;
    compare_cloud.height = 1;
    compare_cloud.is_dense = false;
    compare_cloud.points.resize (compare_cloud.width * compare_cloud.height);
          
    for (size_t i = 0; i < compare_cloud.size(); ++i){
      compare_cloud.points[i].x = i % cols; 
      compare_cloud.points[i].y = i / cols;
      compare_cloud.points[i].z = base_cloud.points[i].z + rand() % 3 + (-1); // generates a random fluctuation of +/- 1 or 0
      cols++;
    }

    // Write base_cloud to pcd file for testing
    pcl::io::savePCDFileASCII ("compare_cloud.pcd", compare_cloud);
    cout << "Saved " << base_cloud.points.size () << " data points to compare_cloud.pcd." << endl;
  }

  // function that sets z of compare cloud to the abs difference of the two point cloud values
  void compareClouds(pcl::PointCloud<pcl::PointXYZ> &base_cloud, pcl::PointCloud<pcl::PointXYZ> &compare_cloud){
      
    for (size_t i = 0; i < base_cloud.size(); ++i){
        compare_cloud.points[i].z = fabs(base_cloud.points[i].z - compare_cloud.points[i].z);
    }
  }

  // function to smooth cloud to fill in depth uncertainties
  void smoothCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud){

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
   
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (compare_cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    //test this parameter
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (mls_points);

    //"if the normals and the original dimensions need to be in the same cloud, the fields have to be concatenated."
    // I think this is the way to add back in the calculated normals. Needs to be tested.
    // Possibly rewritten. 
    compare_cloud = compare_cloud + mls_points;
  }

  // function to group, extract clusters from an input point cloud
  // Returns vector of pointers to pointclouds, each representing a cluster
  vector< pcl::PointCloud<pcl::PointXYZ> * > extractClustersFromCloud(pcl::PointCloud<pcl::PointXYZ> &compare_cloud){

    //Initialize kdTree and feed compare cloud into the tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (compare_cloud);

    // Vector that contains the cluster indices
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    //How do we test these values?
    /*
    Be careful setting the right value for setClusterTolerance(). If you take a very small value, 
    it can happen that an actual object can be seen as multiple clusters. On the other hand, if you set the 
    value too high, it could happen, that multiple objects are seen as one cluster. So our recommendation is to 
    just test and try out which value suits your dataset.
    */
    ec.setClusterTolerance (0.02); // 2cm
    // We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points.
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (compare_cloud);
    ec.extract (cluster_indices);

    int j = 0;
    vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ()
    for (it; it != cluster_indices.end (); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

      vector<int>::const_iterator pit = it->indices.begin ()
      for (pit; pit != it->indices.end (); ++pit) {
        cloud_cluster->points.push_back (compare_cloud->points[*pit]); //*
      }
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      //Vector of pointers to point clouds. 
      vector< pcl::PointCloud<pcl::PointXYZ> * > cluster_clouds;

      // Writing values to a PCD file
      // Return vector of point clouds (each representing cluster) ?
      cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
      stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;

      //Push cloud into vector
      cluster_clouds.push_back(cloud_cluster);
    } //Closing for loop iterating through cluster_indices

    return cluster_clouds;

  } // closing extract function

  // function that averages values in pointcloud, returns vector of average location (x,y, length and width)
  // Should return average x, y, length, and width (approximate) of obstacles (the cluster)
  // values contained in obstacle object
  vector<Obstacle> extractAverages(vector< pcl::PointCloud<pcl::PointXYZ> * > cluster_clouds){
    vector<Obstacle> obstacles_vector;

    //Iterate through all of the cluster clouds
    for (auto current_cloud: cluster_clouds) {
      double x_min = current_cloud->points[0].x;
      double y_min = current_cloud->points[0].y;
      double x_max = current_cloud->points[0].x;
      double y_max = current_cloud->points[0].y;
      double x_total = 0;
      double y_total = 0;
      int count = 0;

      //iterate through points in current cloud
      for(size_t i = 0; i < current_cloud->size(); ++i){
        // Reset min if needed
        if(current_cloud->points[i].x < x_min){
          x_min = current_cloud->points[i].x;
        }
        if(current_cloud->points[i].y < y_min){
          y_min = current_cloud->points[i].y;
        }
        //Reset max if needed
        if(current_cloud->points[i].x > x_max){
          x_max = current_cloud->points[i].x;
        }
        if(current_cloud->points[i].y > y_max){
          y_max = current_cloud->points[i].y;
        }

        //Add into total; increment count
        x_total += current_cloud->points[i].x;
        y_total += current_cloud->points[i].y;
        count++;
      }

      // Create obstacle object with relative date & push
      double x_avg = x_total / count;
      double y_avg = y_total / count;
      double width = x_max - x_min;
      double length = y_max - y_min;
      Obstacle currentObstacle(x_avg, y_avg, width, length);
      obstacles_vector.push_back(currentObstacle);


    }


    return obstacles_vector;
  } //closing the averages function

  //Temperorary function that writes values to the terminal
  void terminalWrite(vector<Obstacle> &obstacles_vector){
    int count = 0;
    for(auto i: obstacles_vector){
      double x = i.x;
      double y = i.y;
      double width = i.width;
      double length = i.length;

      cout << "Obstacle number " << count << " located at (" << x << "," << y <<"), with width: " << width << " and length: " << length << endl; 
      count++;
    }
  }



} // Closing namespace