/*
 * Recognition.cpp
 *
 *  Created on: Nov 21, 2015
 *      Author: kamil
 */

#include "Recognition.h"

#include <ros/ros.h>
#include <vector>

#include "Histogramoper.h"


typedef pcl::Histogram<153> SpinImage;

Recognition::Recognition(ros::NodeHandle* node_ptr) {

	node_ptr_=node_ptr;
	node_ptr_->param("recognition_points_to_check", points_to_check_,180);
	node_ptr_->param("recognition_threshold_similarity",threshold_similarity_,0.993);
	node_ptr_->param("recognition_thres_simil_point",thres_simil_point_,35);
}

Recognition::~Recognition() {
	// TODO Auto-generated destructor stub
}

int Recognition::PointToCheck(std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > &all_clouds_spin_image, Cloud::Ptr &the_cloud)
   {

	   DescriptorSpin spinImage(node_ptr_);
	   pcl::PointCloud<SpinImage>::Ptr one_spin_ptr;
	   std::vector<int> objects_bins(all_clouds_spin_image.size(),0);
	   one_spin_ptr=spinImage.DescriptorCreate(the_cloud);

	   int i;
	   for(int y=0; y<points_to_check_;y++)
	   {
		i=rand()% one_spin_ptr->size();
	    FindMatch(one_spin_ptr->at(i),objects_bins,all_clouds_spin_image);
	   }

	   std::vector<int>::iterator result;
	   result = std::max_element(objects_bins.begin(), objects_bins.end());

	   int  position=std::distance(objects_bins.begin(), result);
	   int matches=objects_bins.at(position);

	   if (matches>=thres_simil_point_)
	   	return position;
	   else
		   return -1;
   }

void Recognition::SetPointsToSee(int points_to_check)
{
	points_to_check_=points_to_check;
}

void Recognition::FindMatch(SpinImage  descriptor1, std:: vector<int> &objects,
								  std::vector<std::vector<pcl::PointCloud<SpinImage>::Ptr> > &all_clouds_spin_image)
  {
	   float norm;
	   Histogramoper < SpinImage > enorm;

	   std::vector<int>::iterator result;
	   int max=0;
	   int max_value=0;
	   int i=0;
	   while( i<all_clouds_spin_image.size() && max_value<thres_simil_point_)
		{
		   int j=0;
			while(j<all_clouds_spin_image.at(i).size()  && max_value<thres_simil_point_)
			{
				int k=0;
				while  ( k<all_clouds_spin_image.at(i).at(j)->size() && max_value<thres_simil_point_ )
				{
					norm=enorm.NormComputingSpinImage(descriptor1,all_clouds_spin_image.at(i).at(j)->at(k) );
					if (norm>threshold_similarity_)
					objects.at(i)++;

					result=std::max_element(objects.begin(), objects.end());
					max= std::distance(objects.begin(), result);
					max_value=objects.at(max);
				 k++;
				}
			j++;
			}
		i++;
		}

  }
void Recognition::SetThreshold( double threshold_similarity, int thres_simil_point)
{
	threshold_similarity_=threshold_similarity;
	thres_simil_point_=thres_simil_point;
}
