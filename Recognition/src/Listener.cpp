/*
 * Listener.cpp
 *
 *  Created on: Nov 22, 2015
 *      Author: kamil
 */

#include "Listener.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <vector>
#include <unistd.h>

#include "Filtering.h"
#include "ClasterDivision.h"
#include "Rospublish.h"
#include "Iocloud.h"
#include "Recognition.h"

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::Histogram<153> SpinImage;

Listener::Listener(ros::Publisher* pub,  pcl::visualization::CloudViewer* viewer,
					std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > *all_clouds_spin,
					Rospublish*  pub_array_ptr,
					ros::NodeHandle* node_ptr)
{
	publisher_=pub;
	viewer_ptr_=viewer;
	all_clouds_spin_image_=all_clouds_spin;
	pub_array_ptr_=pub_array_ptr;
	node_ptr_=node_ptr;
	node_ptr_->param<std::string>("listener_save_name",save_name_,"test");
	node_ptr_->param("listener_file_number",file_number_, 0);
}

Listener::~Listener() {
	// TODO Auto-generated destructor stub
}

void Listener::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud_resived = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr unfiltered_ptr(cloud_resived);

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud_resived);


	//Filtering
	pcl::PCLPointCloud2ConstPtr filtered_result;
	AllFiltering(unfiltered_ptr,filtered_result);

	Cloud cloud_def;
	pcl::fromPCLPointCloud2(*filtered_result, cloud_def);
	final_cloud_.reset (new Cloud (cloud_def));

	DisplayResivedStream(final_cloud_);

	//  cluster division
	std::vector<Cloud::Ptr> clustered_clouds;
	ClasterDivision clasters(node_ptr_);
	clasters.Divide(final_cloud_,clustered_clouds);

	std::vector<int> detected;
	Recognition find_obj(node_ptr_);
	for (int i=0; i<clustered_clouds.size(); i++)
	{
		int numer;
		numer=find_obj.PointToCheck(*all_clouds_spin_image_,clustered_clouds.at(i));
		detected.push_back(numer);
		pub_array_ptr_->SetCubeValues(clustered_clouds.at(i));
		pub_array_ptr_->CreateCube(detected.at(i));

	}
	pub_array_ptr_->DisplayMarkerArray(*publisher_);
}
void Listener::SaveCloud(Cloud &cloud_def)
{
	//to save a pointcloud
	Iocloud save;
	save.SavePcd(cloud_def,save_name_,file_number_);
	file_number_++;
	cout<<"You have 10 s to change a pose of an object"<<endl;
	usleep(10000000);

}

void Listener::AllFiltering(pcl::PCLPointCloud2ConstPtr &cloud_ptr, pcl::PCLPointCloud2ConstPtr &filter_result)
{
		Filtering filter(node_ptr_);

		//  voxelgrid filter
		pcl::PCLPointCloud2ConstPtr voxel_result;
		filter.VoxelGrid(cloud_ptr, voxel_result);

		// floor remove filter
		pcl::PCLPointCloud2ConstPtr floor_result;
		filter.FloorRemove(voxel_result,floor_result);

		 // wall remove filter
		pcl::PCLPointCloud2ConstPtr wall_result;
		filter.WallRemove(floor_result,wall_result);

		// outliers remove filter
		filter.OutliersRemove(wall_result,filter_result);
}

void Listener::DisplayResivedStream(Cloud::Ptr cloud_ptr)
{
   viewer_ptr_->showCloud(cloud_ptr);
}
