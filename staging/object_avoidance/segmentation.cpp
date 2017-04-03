#include <opencv2/opencv.hpp>

#include "segmentation.h"

namespace Segmentation{

void segmentDepthImage(const cv::Mat& image, 
  std::vector<pair<int, int>>& obstacles){

    cv::Mat grayscaleMat (image.size(), CV_8U);

    cv::Mat binaryMat(grayscaleMat.size(), grayscaleMat.type());

    cv::threshold(grayscaleMat, binaryMat, 100, 255, cv::THRESH_BINARY);

    // Dilate a bit the dist image
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
    dilate(binaryMat, binaryMat, kernel1);

    // Find total markers
    std::vector< std::vector<cv::Point> > contours;
    findContours(binaryMat, contours, cv::CV_RETR_EXTERNAL, cv::CV_CHAIN_APPROX_SIMPLE);


}

} // namespace Segmentation