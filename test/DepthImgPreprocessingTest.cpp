#include <gtest/gtest.h>
#include "depth_preprocessing.h"
// Demonstrate some basic assertions.
TEST(DepthImgPreprocessingTest, BasicAssertions) {
	// Expect two strings not to be equal.
	Matrix2f source;
	Matrix2f destination;
	source(0,0) = 1;
	source(0,1) = 0.5;
	source(1,0) = 0.2;
	source(1,1) = 1;
	kinect_fusion::BilateralFilter(source, destination, 1, 1);
	cout << destination(0,0);
	EXPECT_TRUE(destination(0,0)<0.816 && destination(0,0)>0.815);
}