/*
 * Iocloud.h
 *
 *  Created on: Nov 20, 2015
 *      Author: kamil
 *  Class contain methods enables to input and output PointCloud
 *  such as save and load
 */

#ifndef MY_PCL_TUTORIAL_SRC_IOCLOUD_H_
#define MY_PCL_TUTORIAL_SRC_IOCLOUD_H_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

typedef  pcl::PointCloud<pcl::PointXYZ> Cloud;

class Iocloud {

public:

	Iocloud();
	virtual ~Iocloud();

	//method enable to save and load Pcd files
	void SavePcd(Cloud &cloud, std::string address, int num_file );
	std::vector<std::vector<Cloud::Ptr> > LoadPcd( std::string  objects_list);
};

#endif /* MY_PCL_TUTORIAL_SRC_IOCLOUD_H_ */
