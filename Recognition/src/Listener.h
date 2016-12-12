/*
 * Listener.h
 *
 *  Created on: Nov 22, 2015
 *      Author: kamil
 *  Listener enables to gather data from topic from ROS and operate on it
 */

#ifndef MY_PCL_TUTORIAL_SRC_LISTENER_H_
#define MY_PCL_TUTORIAL_SRC_LISTENER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <vector>

#include "Filtering.h"
#include "ClasterDivision.h"
#include "Recognition.h"
#include "Rospublish.h"

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::Histogram<153> SpinImage;

class Listener {

private:
	// filtered cloud ready to  use for  searching algorithm
	Cloud::Ptr final_cloud_;

	// ros objects adresses
	Rospublish*  pub_array_ptr_;
	ros::NodeHandle* node_ptr_;
	ros::Publisher* publisher_;
	pcl::visualization::CloudViewer* viewer_ptr_;

	// All  operations with filtering are done here
	void AllFiltering(pcl::PCLPointCloud2ConstPtr &cloud_ptr, pcl::PCLPointCloud2ConstPtr &filter_result);

public:
	//name of saved file
	std::string save_name_;

	// number of saved file
	int file_number_;

	// vector of all PointCloud of  descriptors
	std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > *all_clouds_spin_image_;

	// Constructor  assign  values to class variables
	Listener(ros::Publisher* pub, pcl::visualization::CloudViewer* viewer,
			 std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > *all_clouds_spin_image,
			 Rospublish*  pub_array_ptr,
			 ros::NodeHandle* node_ptr);
	virtual ~Listener();

	// callback function is a main function of Listener, which  gather information from topic (  which is represented as a shared pointer)
	// and  operates on it
	void callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

	// method display POintCloud
	void DisplayResivedStream(Cloud::Ptr cloud_ptr);

	//Save set of PointClouds
	void SaveCloud(Cloud &cloud_def);
};

#endif /* MY_PCL_TUTORIAL_SRC_LISTENER_H_ */
