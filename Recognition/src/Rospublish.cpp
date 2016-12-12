/*
 * Rospublish.cpp
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 */

#include "Rospublish.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <Eigen/Dense>
#include <pcl/pcl_base.h>
#include "pcl/common/common.h"


typedef  pcl::PointCloud<pcl::PointXYZ> Cloud;

Rospublish::Rospublish(ros::NodeHandle* node_ptr) {

	node_ptr_= node_ptr;
	node_ptr_->param("publish_cube_x", cube_x_,0.0);
	node_ptr_->param("publish_cube_y", cube_y_,0.0);
	node_ptr_->param("publish_cube_z", cube_z_,0.0);
	node_ptr_->param("publish_cube_width", cube_width_,0.40);
	node_ptr_->param("publish_cube_depth", cube_depth_,0.40);
	node_ptr_->param("publish_cube_height", cube_height_,0.40);
	node_ptr_->param("publish_transparency", transparency_,0.30);
	node_ptr_->param("publish_cube_x", cube_x_,0.0);

	markers_remove_=0;
}

Rospublish::~Rospublish() {
	// TODO Auto-generated destructor stub
}

void Rospublish::SetCubeValues(Cloud::Ptr &cloud_cube)
{
	pcl::PointXYZ max, min;
	pcl::getMinMax3D(*cloud_cube,max, min);

	cube_width_=fabs(max.x-min.x);
	cube_depth_=fabs(max.y-min.y);
	cube_height_=fabs(max.z-min.z);
	cube_x_=(max.x+min.x)/2+cube_width_/2;
	cube_y_=(max.y+min.y)/2;
	cube_z_=(max.z+min.z)/2;
}

void Rospublish::DisplayMarkerArray(ros::Publisher &pub)
{
	visualization_msgs::MarkerArray marker;
	marker.markers.resize(marker_vector_.size());

	for(int i=0; i<marker_vector_.size(); i++)
	{
	// set  marker's options
	marker.markers[i].action= visualization_msgs::Marker::DELETE;
	marker.markers[i].header.frame_id = marker_vector_.at(i).header.frame_id;
	marker.markers[i].header.stamp = marker_vector_.at(i).header.stamp;
	marker.markers[i].ns=marker_vector_.at(i).ns;
	marker.markers[i].id=marker_vector_.at(i).id;
	marker.markers[i].type = marker_vector_.at(i).type;
	marker.markers[i].action = marker_vector_.at(i).action;

	// set a position and orientation
	marker.markers[i].pose.position.x = marker_vector_.at(i).pose.position.x;
	marker.markers[i].pose.position.y = marker_vector_.at(i).pose.position.y;
	marker.markers[i].pose.position.z = marker_vector_.at(i).pose.position.z;;
	marker.markers[i].pose.orientation.x = marker_vector_.at(i).pose.orientation.x;
	marker.markers[i].pose.orientation.y = marker_vector_.at(i).pose.orientation.y;
	marker.markers[i].pose.orientation.z = marker_vector_.at(i).pose.orientation.z;
	marker.markers[i].pose.orientation.w = marker_vector_.at(i).pose.orientation.w;

	// scale the marker
	marker.markers[i].scale.x = marker_vector_.at(i).scale.x;
	marker.markers[i].scale.y = marker_vector_.at(i).scale.y;
	marker.markers[i].scale.z = marker_vector_.at(i).scale.z;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.markers[i].color.r = marker_vector_.at(i).color.r;
	marker.markers[i].color.g = marker_vector_.at(i).color.g;
	marker.markers[i].color.b = marker_vector_.at(i).color.b;
	marker.markers[i].color.a = marker_vector_.at(i).color.a;
	marker.markers[i].lifetime = marker_vector_.at(i).lifetime;

	}
	DeleteMarkers(pub, markers_remove_);
	pub.publish(marker);
	markers_remove_=marker_vector_.size();
	marker_vector_.clear();

}

void Rospublish::DeleteMarkers(ros::Publisher &pub, int markers_remove)
{
	visualization_msgs::MarkerArray marker;
	marker.markers.resize(markers_remove);

	for(int i=0; i<markers_remove; i++)
	{
		marker.markers[i].id=i+1;
		marker.markers[i].action= visualization_msgs::Marker::DELETE;
		marker.markers[i].header.frame_id = "camera_depth_optical_frame";
		marker.markers[i].header.stamp = ros::Time::now();
		marker.markers[i].type = visualization_msgs::Marker::CUBE;
		marker.markers[i].ns="my_space";
	}
	  pub.publish(marker);
}


void Rospublish::CreateCube(int color)
{
		visualization_msgs::Marker marker;

		//set  marker options
	    marker.header.frame_id = "camera_depth_optical_frame";
	    marker.header.stamp = ros::Time::now();
	    marker.ns="my_space";
	    marker.id=marker_vector_.size()+1;
	    marker.type = visualization_msgs::Marker::CUBE;
	    marker.action = visualization_msgs::Marker::ADD;
	 	marker.lifetime = ros::Duration();

	    // set a position and orientation
	    marker.pose.position.x = cube_x_;
	    marker.pose.position.y = cube_y_;
	    marker.pose.position.z = cube_z_;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    // scale the marker
	    marker.scale.x = cube_width_;
	    marker.scale.y = cube_depth_;
	    marker.scale.z = cube_height_;

	    // Set the color
	   switch(color)
	    {

	   case -1: //unknown
		   marker.color.r = 1.0f;
		   marker.color.g = 0.0f;
		   marker.color.b = 0.0f;
	    	break;

	   case 0: //human
		   marker.color.r = 0.0f;
		   marker.color.g = 1.0f;
		   marker.color.b = 0.0f;
			break;

	   case 1: //duck
		   marker.color.r = 0.0f;
		   marker.color.g = 0.0f;
		   marker.color.b = 1.0f;
			break;
	    }
	    marker.color.a = transparency_;

	    marker_vector_.push_back(marker);
}
