/*
 * DescriptorsTypes.h
 *
 *  Created on: Jul 5, 2016
 *      Author: kamil
 *   Class enables to create different types of descriptors
 *    for PointClouds
 */

#ifndef MY_PCL_TUTORIAL_SRC_DESCRIPTORS_H_
#define MY_PCL_TUTORIAL_SRC_DESCRIPTORS_H_

#include <ros/ros.h>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

template <class T>
class DescriptorsTypes {

public:
	DescriptorsTypes();
	virtual ~DescriptorsTypes();

protected:
	ros::NodeHandle* node_ptr_;


public:
	virtual pcl::PointCloud<T>::Ptr  DescriptorCreate(Cloud::Ptr cloud_to_change)=0;

		// Create vector of PointClouds of a descriptors
	virtual void CreateClouds(std::vector<std::vector<Cloud::Ptr> > all_point_clouds,
								std::vector<std::vector<pcl::PointCloud<T>::Ptr> > &all_clouds )=0;
};

#endif /* MY_PCL_TUTORIAL_SRC_DESCRIPTORS_H_ */
