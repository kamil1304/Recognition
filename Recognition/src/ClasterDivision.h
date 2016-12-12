/*
 * ClasterDivision.h
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 *  Class contains methods to cluster PointClouds
 *
 */

#ifndef MY_PCL_TUTORIAL_SRC_CLASTERDIVISION_H_
#define MY_PCL_TUTORIAL_SRC_CLASTERDIVISION_H_

#include <ros/ros.h>
#include <vector>
typedef  pcl::PointCloud<pcl::PointXYZ> Cloud;

class ClasterDivision {


private:

	//minimal distance between points to treat them as a cluster
	double dist_between_points_;

	// min and max amount of points in one cluster
	int min_points_;
	int max_points_;

	ros::NodeHandle* node_ptr_;

public:
	// put default values  to  class variables
	ClasterDivision(ros::NodeHandle* node_ptr);
	virtual ~ClasterDivision();

	// load PointCloud and divide it to as many clasters as parameters allow
	void Divide(Cloud::Ptr to_divide_cloud, std::vector<Cloud::Ptr> &clustered_clouds );

	//  Set  client's parameters for class variables
	void SetParameters(int min_points, int max_points, float dist_between_points);
};

#endif /* MY_PCL_TUTORIAL_SRC_CLASTERDIVISION_H_ */
