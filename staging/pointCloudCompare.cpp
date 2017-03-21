#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>

using namespace std;

int main(){

	// ------------------------------------------------------------------------------
	// TODO : Understand how voxel resolution works and how to define
	// ------------------------------------------------------------------------------
	// Octree resolution - side length of octree voxels
  	float resolution = 32.0f;

	// Instantiate octree-based point cloud change detection class
  	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

  	// Instantiate a point cloud for base image
	pcl::PointCloud<pcl::PointXYZ> base_cloud;

	// ------------------------------------------------------------------------------
	// TODO : Use grabber to populate depth values into base_cloud
	// ------------------------------------------------------------------------------

	// Add points from cloudA to octree
	octree.setInputCloud(base_cloud);
	octree.addPointsFromInputCloud ();

	//Loop for constantly getting new cloud data to compare to
	while(1){
		
		//Switch to new buffer. Buffers are used for getPointIndicesFromNewVoxels Function
		octree.switchBuffers();

	  	// Instantiate a point cloud for base image
		pcl::PointCloud<pcl::PointXYZ> compare_cloud;

		// ------------------------------------------------------------------------------
		// TODO : Use grabber to populate depth values into compare_cloud
		// ------------------------------------------------------------------------------

		// Add points from cloudA to octree
		octree.setInputCloud(compare_cloud);
		octree.addPointsFromInputCloud ();

		// Vector for voxels present in compare_cloud but not in base_cloud
		vector<int> fullVoxelVector

		// Vector for points that represent points of concern - obstacles
		vector<pair<int, int>> obstaclePointVector;


		// Get vector of point indices from octree voxels which did not exist in previous buffer
  		octree.getPointIndicesFromNewVoxels (fullVoxelVector);

  		// Iterate through vector to output relevant points
  		for (unsigned int i = 0; i < fullVoxelVector.size (); ++i){
  			// Print current point being checked. Remove later. 
  			int current_idx = fullVoxelVector[i]; 
  			int x = cloudB->points[current_idx].x;
  			int y = cloudB->points[current_idx].y;
  			int z = cloudB->points[current_idx].z;
  			cout << i << "# Index:" << current_idx << "  Point: (" << x << ", " << y << ", " << z << ")" << endl;


            // ------------------------------------------------------------------------------
			// TODO : UTest for appropriate value to threshold depth at. Currently using 1. 
            // Check Units. 
			// ------------------------------------------------------------------------------
            if (z >= 2){
            	// Create a pair that represents a point with the x and y value
            	pair<int, int> point;
            	point.first = x;
            	point.second = y;

            	// Push this point into vector
            	obstaclePointVector.push_back(point);
            }
  		}

  		// Print to see what points were identified as obstacles
  		for(unsigned int i = 0; i < obstaclePointVector.size(); ++i){
  			int x = obstaclePointVector[i].first;
  			int y = obstaclePointVector[i].second;

  			cout << "Obstacle detected at: (" << x << ", " << y << ")" << endl;
  		}

  		// Delete current buffer (With compare_cloud data)
  		octree.deleteCurrentBuffer();

  		//Switch back to previous buffer
  		// ------------------------------------------------------------------------------
		// TODO : Test whether this is correct way to switch back to base_cloud buffer
		// ------------------------------------------------------------------------------
  		octree.switchBuffers()




	}


}
