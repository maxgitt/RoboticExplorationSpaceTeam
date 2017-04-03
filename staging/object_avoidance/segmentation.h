#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <opencv2/opencv.hpp>

namespace Segmentation{

void segmentDepthImage(const cv::Mat& image, 
  std::vector<pair<int, int>>& obstacles);

    

}; // namespace Segmentation

#endif // SEGMENTATION_H