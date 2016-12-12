/*
 * Descriptors.cpp
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 */

#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>
#include <ros/ros.h>
#include "DescriptorSpin.h"

typedef pcl::Histogram<153> SpinImage;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;


DescriptorSpin::DescriptorSpin(ros::NodeHandle* node_ptr) {

	node_ptr_=node_ptr;
	node_ptr_->param("descriptors_radius_normals",radius_normals_,0.14);
	node_ptr_->param("descriptors_radius_suport_cylinder",radius_suport_cylinder_,0.14);
}

DescriptorSpin::~DescriptorSpin() {
	// TODO Auto-generated destructor stub
}

pcl::PointCloud<SpinImage>::Ptr  DescriptorSpin::DescriptorCreate(Cloud::Ptr cloud_to_change)
{
		// Object for storing the normals.
	   	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	   	// Object for storing the spin image for each point.
	   	pcl::PointCloud<SpinImage>::Ptr descriptors(new pcl::PointCloud<SpinImage>());

	   	// Estimate the normals.
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
		normal_estimation.setInputCloud(cloud_to_change);
		normal_estimation.setRadiusSearch(radius_normals_);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normal_estimation.setSearchMethod(kdtree);
		normal_estimation.compute(*normals);

		// Spin image estimation object.
		pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, SpinImage> sie;

		sie.setInputCloud(cloud_to_change);
		sie.setInputNormals(normals);

		// Radius of the support cylinder.
		sie.setRadiusSearch(radius_suport_cylinder_);
		sie.setImageWidth(8);
		sie.compute(*descriptors);

	   	return descriptors;
}
void DescriptorSpin::SetRadiuses(float radius_normals, float radius_suport_cylinder)
{
	radius_normals_=radius_normals;
	radius_suport_cylinder_=radius_suport_cylinder;
}


void DescriptorSpin::CreateClouds(std::vector<std::vector<Cloud::Ptr> > all_point_clouds,
								  std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > &all_clouds)
{
	for (int i=0; i<all_point_clouds.size(); i++)
	{
		std::vector<pcl::PointCloud<SpinImage>::Ptr> temp_obj;
		pcl::PointCloud<SpinImage>::Ptr temp_cloud_spin;
		for(int j=0; j<all_point_clouds.at(i).size(); j++)
		{
			temp_cloud_spin=SpinImageCreate(all_point_clouds.at(i).at(j));
			temp_obj.push_back(temp_cloud_spin);
		}
		all_clouds_spin_image.push_back(temp_obj);
	}
}
