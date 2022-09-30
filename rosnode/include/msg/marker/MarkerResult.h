#pragma once

#include <cstdint>
#include "../common/error/ErrorCode.h"
#include <opencv2/opencv.hpp>
namespace marker_msgs
{
  using MResult =  error_msgs::ErrorCode;
  using MResultState = error_msgs::ErrorCode;
  struct MarkerResult 
  {
    std::uint64_t timeStamp;
    MResult res;
    MResultState status;
    //marker的id
    int tagID;
    //left_bottom->right_bottom->right_up->left_up, center
    double p[5][2];
    //transform from camera to marker, Tmc \in SE(3)
    cv::Mat tmc;
    error_msgs::ErrorCode errorCode; //内部使用
  };
}
