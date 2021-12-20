#pragma once

#include <iostream>
#include <math.h>
#include "Eigen.h"
using namespace std;


float calculatePreprocessedValue(int p, int q, Matrix2f& source, float sigma_s, float sigma_r)
{
	float result=0;
	float factor=0;
	int rows = source.rows();
	int cols = source.cols();
	for (int i = 0; i != rows; i++)
	{
		for (int j = 0; j != cols; j++)
		{
			float pixelPosNormSquare = (pow(i - p, 2) + pow(j - q, 2));
			float pixelValueNormSquare = (pow(source(p,q) - source(i,j), 2));
			result += exp(-pixelPosNormSquare / pow(sigma_s, 2)) * exp(-pixelValueNormSquare / pow(sigma_r, 2)) * source(i,j);
			
			factor += exp(-pixelPosNormSquare / pow(sigma_s, 2)) * exp(-pixelValueNormSquare / pow(sigma_r, 2));
			
		}
	}
	
	return result / factor;
}

namespace kinect_fusion {


void BilateralFilter(Matrix2f& source, Matrix2f& destination, float sigma_s, float sigma_r)
{
	int rows = source.rows();
	int cols = source.cols();
	for (int i = 0; i != rows; i++)
	{
		for (int j = 0; j != cols; j++)
		{
			destination(i,j) = calculatePreprocessedValue(i,j, source, sigma_s, sigma_r);
		}
	}
}
} // namespace kinect_fusion
