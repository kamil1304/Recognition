/*
 * Histogramoper.h
 *
 *  Created on: Nov 20, 2015
 *      Author: kamil
 *     Class contain methods  which can operate on histograms
 *
 */

#ifndef MY_PCL_TUTORIAL_SRC_HISTOGRAMOPER_H_
#define MY_PCL_TUTORIAL_SRC_HISTOGRAMOPER_H_

#include <pcl/features/spin_image.h>
#include <vector>

typedef pcl::Histogram<153> SpinImage;

template< class T>
class Histogramoper {
public:
	Histogramoper();
	virtual ~Histogramoper();

	// Computing norm  of two histograms
	float NormComputingSpinImage(T  descriptor1, T descriptor2);

	virtual void NormComputingDifferentDescriptor()=0;
};

#endif /* MY_PCL_TUTORIAL_SRC_HISTOGRAMOPER_H_ */
