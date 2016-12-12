/*
	 Short description:
	Main class, here is created ROS node,
	advertiser & publisher. Calling this class,
	user can detect objects using PrimeSens sensor.
	Default objects are loaded in the begining of program by Iocloud
	and compared with  PointCloud sent by sensor.

*/

	#include <ros/ros.h>
    #include <sensor_msgs/PointCloud2.h>
	#include <visualization_msgs/Marker.h>
	#include <visualization_msgs/MarkerArray.h>
    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl/point_cloud.h>
	#include <pcl/visualization/cloud_viewer.h>
	#include <string>
	#include <vector>

	#include "DescriptorSpin.h"
	#include "Iocloud.h"
	#include "Rospublish.h"
	#include "Listener.h"

	typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
	typedef pcl::Histogram<153> SpinImage;

    ros::Publisher pub;

   int
   main (int argc, char** argv)
   {
	  // Initialize ROS and name node
	  ros::init (argc, argv, "objects_recognition");
	  ros::NodeHandle node;
	  ros::NodeHandle* node_ptr=&node;



	  std::string file_to_load;
	  std::string publish_topic_name;
	  std::string subscribed_topic;
	  std::string viewer_name;
	  node.param<std::string>("file_to_load",file_to_load,"list.txt");
	  node.param<std::string>("publish_topic_name",publish_topic_name,"visualization_markers");
	  node.param<std::string>("subscribed_topic",subscribed_topic,"/camera/depth_registered/points");
	  node.param<std::string>("viewer_name",viewer_name,"Cloud viewer");

	  // create of used vectors
	  std::vector<std::vector<Cloud::Ptr> > all_point_clouds;
	  std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > all_clouds_spin_image;

	  // viewer creation
	  pcl::visualization::CloudViewer viewer(viewer_name);

	  // create pointers
	  ros::Publisher* publisher=&pub;
	  pcl::visualization::CloudViewer* viewer_ptr=&viewer;

	  //load pcd files
	  Iocloud parse;
	  all_point_clouds=parse.LoadPcd(file_to_load);

	  // create descriptors of pcd files
	  DescriptorSpin spin_descriptors(node_ptr);
	  spin_descriptors.CreateClouds(all_point_clouds,all_clouds_spin_image);

      // create Listener class which contain callback
      Rospublish  pub_array(node_ptr);
      Rospublish*  pub_array_ptr=&pub_array;
      Listener listener(publisher, viewer_ptr,&all_clouds_spin_image, pub_array_ptr,node_ptr);

      // subscriber
      ros::Subscriber sub = node.subscribe (subscribed_topic, 1, &Listener::callback, &listener);

      //publisher
      pub = node.advertise<visualization_msgs::MarkerArray>(publish_topic_name, 10);

      ros::spin ();

   }
