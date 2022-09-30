#pragma once

#include <cstdint>
#include <string>
#include "../../msg/common/error/ErrorMessage.h"
#include <opencv2/opencv.hpp>

namespace marker_srvs
{
  struct MarkerResultSrv
  {
    using MarkerResult =  error_msgs::ErrorMessage;
    using MarkerResultState = error_msgs::ErrorMessage;
    struct Request
    {
      std::uint64_t timeStamp;
      int status; // 0: start, 1:end
    } request;
    struct Response
    {
      std::uint64_t timeStamp;
      MarkerResult res;
      MarkerResultState status;
      //marker的id
      int tagID;
      //left_bottom->right_bottom->right_up->left_up, center
      double p[5][2];
      //transform from camera to marker, Tmc \in SE(3)
      cv::Mat tmc;
    } response;
  };
}

