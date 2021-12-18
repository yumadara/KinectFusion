#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;


void depthMapPreprocessing(float* source, float* result )
{
	bilateralFilter(source, result, 4, 2, 2);
}

