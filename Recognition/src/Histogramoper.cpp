/*
 * Histogramoper.cpp
 *
 *  Created on: Nov 20, 2015
 *      Author: kamil
 */

#include "Histogramoper.h"

#include <pcl/features/spin_image.h>
#include <vector>

typedef pcl::Histogram<153> SpinImage;

template< class T>
Histogramoper<T>::Histogramoper() {
	// TODO Auto-generated constructor stub

}
template< class T>
Histogramoper<T>::~Histogramoper() {
	// TODO Auto-generated destructor stub
}
template< class T>
float Histogramoper<T>::NormComputingSpinImage(T  descriptor1, T descriptor2)
{
   float aver1=0;
   float aver2=0;

   for (int i=0; i<descriptor1.descriptorSize(); i++)
   {
	   aver1=aver1+descriptor1.histogram[i];
	   aver2=aver2+descriptor2.histogram[i];
   }
   aver1=aver1/descriptor1.descriptorSize();
   aver2=aver2/descriptor2.descriptorSize();

   float sum_nom=0;
   float sum1=0,sum2=0;
   float a,b;

   for (int i=0; i<descriptor1.descriptorSize(); i++)
   {
	   a=(descriptor1.histogram[i]-aver1);
	   b=(descriptor2.histogram[i]-aver2);
	   sum_nom=sum_nom+a*b;
	   sum1=sum1+a*a;
	   sum2=sum2+b*b;
   }
   float comparison=sum_nom/sqrt(sum1*sum2);

   return comparison;

}

