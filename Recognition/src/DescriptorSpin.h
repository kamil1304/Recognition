/*
 * Descriptors.h
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 *    Class enables to use SpinImage descriptor
 */

#ifndef MY_PCL_TUTORIAL_SRC_DESCRIPTORS_H_
#define MY_PCL_TUTORIAL_SRC_DESCRIPTORS_H_
#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>
#include <ros/ros.h>
#include "DescriptorsTypes.h"

typedef pcl::Histogram<153> SpinImage;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;


class DescriptorSpin: public DescriptorsTypes <SpinImage>
{

private:
	double radius_normals_;
	double radius_suport_cylinder_;

public:
	// Constructor put default  values to class variables
	DescriptorSpin(ros::NodeHandle* node_ptr);
	virtual ~DescriptorSpin();

	// Create descriptors for PointCloud
	virtual pcl::PointCloud<SpinImage>::Ptr  DescriptorCreate (Cloud::Ptr cloud_to_change) override;

	// Create vector of PointClouds of a descriptors
	virtual void CreateClouds(std::vector<std::vector<Cloud::Ptr> > all_point_clouds,
							std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > &all_clouds) override;

	//Enable to set  class variables
	void SetRadiuses(float radius_normals, float radius_suport_cylinder);
};

#endif /* MY_PCL_TUTORIAL_SRC_DESCRIPTORS_H_ */
