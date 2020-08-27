#ifndef OPTIMIZE_FEATS
#define OPTIMIZE_FEATS

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


void optimizePerformance( cv::Mat& points3D_t0, cv::Mat& points3D_t1, cv::Mat& rotation, cv::Mat& translation, int& kCorners );


#endif
