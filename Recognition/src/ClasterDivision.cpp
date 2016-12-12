/*
 * ClasterDivision.cpp
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 */

#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

typedef  pcl::PointCloud<pcl::PointXYZ> Cloud;


#include "ClasterDivision.h"

ClasterDivision::ClasterDivision(ros::NodeHandle* node_ptr) {

	node_ptr_=node_ptr;
	node_ptr_->param("laster_dist_between_points",dist_between_points_,0.02);
	node_ptr_->param("claster_min_points",min_points_,100);
	node_ptr_->param("claster_max_points",max_points_,25000);

}

ClasterDivision::~ClasterDivision() {
	// TODO Auto-generated destructor stub
}

void ClasterDivision::Divide( Cloud::Ptr to_divide_cloud, std::vector<Cloud::Ptr> &clustered_clouds )
{
	// Creating the KdTree object for the search method of the extraction
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (to_divide_cloud);

	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (dist_between_points_); // 2cm
	  ec.setMinClusterSize (min_points_);
	  ec.setMaxClusterSize (max_points_);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (to_divide_cloud);
	  ec.extract (cluster_indices);

	  int j = 0;
	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		
	    Cloud::Ptr cloud_cluster (new Cloud);
	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	      cloud_cluster->points.push_back (to_divide_cloud->points[*pit]); //*
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    std::stringstream ss;
	    clustered_clouds.push_back(cloud_cluster);

	    j++;
	  }
}

void ClasterDivision::SetParameters(int min_points, int max_points, float dist_between_points)
{
	dist_between_points_=dist_between_points;
	min_points_= min_points;
	max_points_=max_points;
}
