/*
 * Filtering.h
 *
 *  Created on: Nov 20, 2015
 *      Author: kamil
 *    Class enables to filer  PointCloud using different methods
 *    Main taks is do decrease amount of points which increase
 *    next operations on the PointCloud
 */

#ifndef MY_PCL_TUTORIAL_SRC_FILTERING_H_
#define MY_PCL_TUTORIAL_SRC_FILTERING_H_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


class Filtering {

private:
	//parameters fo voxel grid
	double voxel_x_;
	double voxel_y_;
	double voxel_z_;

	//parameters for floor removal
	double floor_min_;
	double floor_max_;

	//parameters for wall removal
	double wall_min_;
	double wall_max_;

	//parameters for outliers removal
	int out_mean_;
	double out_thres_;

	ros::NodeHandle* node_ptr_;

public:
	 // set default class variables
	Filtering(ros::NodeHandle* node_ptr);
	virtual ~Filtering();

	// decrease amount of points by downsampling a PointCloud
	void VoxelGrid (pcl::PCLPointCloud2ConstPtr cloud_ptr, pcl::PCLPointCloud2ConstPtr &cloud_floor_ptr);

	//remove all points which are beneath and above some threshold, it causes floor and ceiling removal
	void FloorRemove(pcl::PCLPointCloud2ConstPtr cloud_floor_ptr, pcl::PCLPointCloud2ConstPtr &cloud_wall_ptr );

	//remove all points which are further or closer than  some threshold, it causes wall removal
	void WallRemove(pcl::PCLPointCloud2ConstPtr cloud_wall_ptr, pcl::PCLPointCloud2ConstPtr &cloud_outliers_ptr);

	// remove outlires which lies further than specific distance
	void OutliersRemove(pcl::PCLPointCloud2ConstPtr cloud_outliers_ptr,pcl::PCLPointCloud2ConstPtr &cloud_filtered);

	// Methods enable set class variables
	void SetVoxel( float voxel_x, float voxel_y, float voxel_z);
	void SetFloor( float floor_min, float floor_max);
	void SetWall( float wall_min, float wall_max);
	void SetOutliers( int  out_mean, double out_thres);
};

#endif /* MY_PCL_TUTORIAL_SRC_FILTERING_H_ */
