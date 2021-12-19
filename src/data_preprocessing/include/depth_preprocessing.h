#pragma once

#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

namespace kinect_fusion {

void depthMapPreprocessing(float* source, float* result )
{
	bilateralFilter(source, result, 4, 2, 2);
}
} // namespace kinect_fusion
