/*
 * Recognition.h
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 *
 *   Class enables to find resemblance  between two PointClouds
 */

#ifndef MY_PCL_TUTORIAL_SRC_RECOGNITION_H_
#define MY_PCL_TUTORIAL_SRC_RECOGNITION_H_
#include <ros/ros.h>

#include "DescriptorSpin.h"


class Recognition {

private:
	// amount of point to check their similarity
	int points_to_check_;

	// similarity threshold <1,-1> between two points
	double threshold_similarity_;

	// amount of similar points
	int thres_simil_point_;

	ros::NodeHandle* node_ptr_;


public:
	// set  default class variables
	Recognition(ros::NodeHandle* node_ptr);
	virtual ~Recognition();

	// Method take certain amount of points and check if they appear in  provided models
	// method return a number of one of provided objects in which PointCloud from sensor was found.
	// Method return -1 when  object seen by sensor is not found in default models
	int PointToCheck(std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > &all_clouds_spin_image, Cloud::Ptr &Tthe_cloud);

	// Method  look for match between one descriptor and provided PointCloud
	void FindMatch(SpinImage  descriptor1, std:: vector<int> &objects,
					std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > &all_clouds_spin_image);

	// Methods enable to change default class variables
	void SetPointsToSee(int points_to_check);
	void SetThreshold( double threshold_similarity, int thres_simil_point);
};

#endif /* MY_PCL_TUTORIAL_SRC_RECOGNITION_H_ */
