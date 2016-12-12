/*
 * Rospublish.h
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 *    Class create a markers and allow to publish them in certian position
 *
 */

#ifndef MY_PCL_TUTORIAL_SRC_ROSPUBLISH_H_
#define MY_PCL_TUTORIAL_SRC_ROSPUBLISH_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

typedef  pcl::PointCloud<pcl::PointXYZ> Cloud;

class Rospublish {

private:
	//position of a cube
	double cube_x_;
	double cube_y_;
	double cube_z_;

	//dimensions of a cube
	double cube_width_;
	double cube_depth_;
	double cube_height_;

	//transparency of a cube
	double transparency_;

	// vector storing markers
	std::vector<visualization_msgs::Marker > marker_vector_;

	// amount of markers to remove
	int markers_remove_;

	ros::NodeHandle* node_ptr_;

	void DeleteMarkers(ros::Publisher &pub, int markers_remove);

public:
	// constructor take default values of  class variables
	Rospublish( ros::NodeHandle* node_ptr);
	virtual ~Rospublish();

	//method set  certain value of position and dimension of a cube
	void SetCubeValues(Cloud::Ptr &cloud_cube);

	// method  set a color of a cube depending on what kind of object sensor sees
	void CreateCube(int color);

	//  Method display whole array of markers
	void DisplayMarkerArray(ros::Publisher &pub);

};

#endif /* MY_PCL_TUTORIAL_SRC_ROSPUBLISH_H_ */
