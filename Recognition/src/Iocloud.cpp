/*
 * Iocloud.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: kamil
 */
#include "Iocloud.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <fstream>

typedef  pcl::PointCloud<pcl::PointXYZ> Cloud;

Iocloud::Iocloud() {
	// TODO Auto-generated destructor stub
}

Iocloud::~Iocloud() {
	// TODO Auto-generated destructor stub
}

void Iocloud::SavePcd(Cloud &cloud, std::string address, int num_file )
   {
	//  std::string address="cuptest_";
	  std::string name;
	  name.append(address);
	  std::ostringstream ss ;
	  ss<< num_file ;
	  std::string number = ss.str();
	  name.append(number);
	  name.append(".pcd");
	  pcl::io::savePCDFileASCII (name, cloud);
	  std::cout<<"file saved: "<<name<<std::endl;
	  std::cout<<cloud.size()<<"  saved a cloud of points"<<std::endl;
   }

std::vector<std::vector<Cloud::Ptr> > Iocloud::LoadPcd( std::string objects_list)
   {

		std::vector<std::vector<Cloud::Ptr> > all_point_clouds;
		std::vector<std::string> objects;
	    std::string object_name;
	   std::ifstream myfile (objects_list.c_str());

	    if (myfile.is_open())
	    {

	      while ( getline (myfile,object_name) )
	      {
	    	  objects.push_back(object_name);
	      }
	      myfile.close();
	    }
	    else std::cout << "Unable to open file"<<std::endl;

		for (int i=0; i<objects.size();i++)
				std::cout<<"Loaded object: "<<objects.at(i)<<std::endl;

	   for(int i=0; i<objects.size();i++)
	   {
		    bool end = true;
		    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> one_object;

		    int j=0;
		    std::string address="objects/";
			address.append(objects.at(i));
			address.append("/");
			address.append(objects.at(i));
			address.append("_");
			 do
		   {
				std::string path;
				path.append(address);
				std::ostringstream ss ;
				ss<< j ;
				std::string number = ss.str();
				path.append(number);
				path.append(".pcd");

			    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

				if (pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud) == -1)
				{
				  PCL_ERROR ("Couldn't read the file, which doesn't exist. \n");
				  end = false;
				}
				else
				one_object.push_back(cloud);

				 j++;
		   }while (end);
			 all_point_clouds.push_back(one_object);
	   }
	   return all_point_clouds;
   }

