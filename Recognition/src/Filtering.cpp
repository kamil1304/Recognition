/*
 * Filtering.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: kamil
 */

#include "Filtering.h"

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


Filtering::Filtering(ros::NodeHandle* node_ptr) {

	node_ptr_=node_ptr;
	node_ptr_->param("filter_voxel_x",voxel_x_,0.006);
	node_ptr_->param("filter_voxel_y",voxel_y_,0.006);
	node_ptr_->param("filter_voxel_z",voxel_z_,0.006);
	node_ptr_->param("filter_floor_min",floor_min_,0.0);
	node_ptr_->param("filter_floor_max",floor_max_,1.15);
	node_ptr_->param("filter_wall_min",wall_min_,-0.2);
	node_ptr_->param("filter_wall_max",wall_max_,1.0);
	node_ptr_->param("filter_out_mean",out_mean_,10);
	node_ptr_->param("filter_out_thres",out_thres_,1.0);

}

Filtering::~Filtering() {
	// TODO Auto-generated destructor stub
}

void Filtering::VoxelGrid(pcl::PCLPointCloud2ConstPtr cloud_ptr, pcl::PCLPointCloud2ConstPtr &cloud_floor_ptr)
{
	 pcl::PCLPointCloud2 cloud_filtered;

	 pcl::VoxelGrid<pcl::PCLPointCloud2> sort;
	 sort.setInputCloud (cloud_ptr);
	 sort.setLeafSize (voxel_x_, voxel_y_, voxel_z_);
	 sort.filter (cloud_filtered);

	 pcl::PCLPointCloud2* cloud_floor_ptr2 = new pcl::PCLPointCloud2;
	 *cloud_floor_ptr2 =  cloud_filtered;
	 pcl::PCLPointCloud2ConstPtr cloud_floor_ptr_temp(cloud_floor_ptr2);
	 cloud_floor_ptr=cloud_floor_ptr_temp;
}
void Filtering::SetVoxel( float voxel_x, float voxel_y, float voxel_z)
{
	voxel_x_=voxel_x;
	voxel_y_=voxel_y;
	voxel_z_=voxel_z;
}

void Filtering::FloorRemove(pcl::PCLPointCloud2ConstPtr cloud_floor_ptr, pcl::PCLPointCloud2ConstPtr &cloud_wall_ptr )
{
		pcl::PCLPointCloud2 cloud_out_floor;

		 pcl::PassThrough<pcl::PCLPointCloud2> filter;
		 filter.setInputCloud (cloud_floor_ptr);
		 filter.setFilterFieldName ("z");
		 filter.setFilterLimits (floor_min_, floor_max_);
		 filter.filter(cloud_out_floor);

		 pcl::PCLPointCloud2* cloud_wall_ptr2 = new pcl::PCLPointCloud2;
		 *cloud_wall_ptr2 =  cloud_out_floor;
		 pcl::PCLPointCloud2ConstPtr cloud_wall_ptr_temp(cloud_wall_ptr2);
		 cloud_wall_ptr=cloud_wall_ptr_temp;
}

void Filtering::SetFloor( float floor_min, float floor_max)
{
	floor_min_=floor_min;
	floor_max_=floor_max;
}

void Filtering::WallRemove(pcl::PCLPointCloud2ConstPtr cloud_wall_ptr, pcl::PCLPointCloud2ConstPtr &cloud_outliers_ptr)
{
	 pcl::PCLPointCloud2 cloud_out_wall;

	 pcl::PassThrough<pcl::PCLPointCloud2> filter;
	 filter.setInputCloud (cloud_wall_ptr);
	 filter.setFilterFieldName ("y");
	 filter.setFilterLimits (wall_min_, wall_max_);
	 filter.filter(cloud_out_wall);

	 pcl::PCLPointCloud2* cloud_outliers_ptr2 = new pcl::PCLPointCloud2;
	 *cloud_outliers_ptr2 =  cloud_out_wall;
	 pcl::PCLPointCloud2ConstPtr cloud_outliers_ptr_temp(cloud_outliers_ptr2);
	 cloud_outliers_ptr=cloud_outliers_ptr_temp;
}

void Filtering::SetWall( float wall_min, float wall_max)
{
	wall_min_=wall_min;
	wall_max_=wall_max;
}

void Filtering::OutliersRemove(pcl::PCLPointCloud2ConstPtr cloud_outliers_ptr,pcl::PCLPointCloud2ConstPtr &cloud_filtered )
{
	 pcl::PCLPointCloud2 cloud_outliers;

	  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> outliers;
	  outliers.setInputCloud (cloud_outliers_ptr);
	  outliers.setMeanK (10);
	  outliers.setStddevMulThresh (1.0);
	  outliers.filter (cloud_outliers);

	  pcl::PCLPointCloud2* cloud_outliers_ptr2 = new pcl::PCLPointCloud2;
	   *cloud_outliers_ptr2 =  cloud_outliers;
	   pcl::PCLPointCloud2ConstPtr cloud_outliers_ptr_temp(cloud_outliers_ptr2);
	  cloud_filtered=cloud_outliers_ptr_temp;
}
void Filtering::SetOutliers( int  out_mean, double out_thres)
{
	out_mean_=out_mean;
	out_thres_=out_thres;
}
