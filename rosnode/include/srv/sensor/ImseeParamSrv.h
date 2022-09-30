  #pragma once

  #include <iostream>
  #include <map>
  #include "math.h"
  namespace sensor_srvs {
  
  struct IMUParameter {
    double _a_max;
    double _g_max;
    double _sigma_g_c;
    double _sigma_a_c;
    double _sigma_bg;
    double _sigma_ba;
    double _sigma_gw_c;
    double _sigma_aw_c;
    double _tau;  // time relationship param
    double _g;
    double _a0[4];
    double _T_BS[16];
    double _Acc[12];  //加速度计的补偿参数
    double _Gyr[12];  //陀螺仪的补偿参数
  };
  struct ModuleInfo {
    char _id[32];
    char _designer[32];
    char _fireware_version[32];
    char _hardware_version[32];
    char _lens[32];
    char _imu[32];
    char _viewing_angle[32];
    char _baseline[32];
  };
  struct SDebugSet {
    bool s_d_receive;
    bool s_d_result;
    bool s_d_track;
    bool s_d_lba;
    bool s_d_map;
    bool s_d_lpc;
    bool s_d_opt;
  };
  struct SlamParameter {
    int _numKeyframes;             //关键帧数量 =5
    int _numImuFrames;             //相邻imu帧数量 =3
    int _ceres_minIterations;      //最小优化次数 =3
    int _ceres_maxIterations;      //最大优化次数 =4
    double _ceres_timeLimit;       //=3.5000000000000003e-02
    int detection_threshold;       //角点阈值 =30
    int detection_octaves;         //=0
    int detection_maxNoKeypoints;  //最大角点数量    = 100
    bool displayImages;            //是否显示slam界面  = true
  };

  enum Resolution : uint8_t { RES_640X400, RES_1280X800, LAST };



  // flash 中存储的原始数据
  struct CameraParameterRaw {
    double _TSC[16];  //
    /** 4X4 matrix from camera to imu */
    int _width;
    /** width */
    int _height;
    /** height */
    double _focal_length[2];
    /** fx,fy */
    double _principal_point[2];
    /** cx,cy */
    double _R[9];
    /** Rectification matrix (stereo cameras only)
       A rotation matrix aligning the camera coordinate system to the ideal
      stereo image plane so that epipolar lines in both stereo images are
      parallel. */

    double _P[12];
    /** Projection/camera matrix
           [fx'  0  cx' Tx]
      P = [ 0  fy' cy' Ty]
          [ 0   0   1   0] */
    double _K[9];
    /** Intrinsic camera matrix for the raw (distorted) images.
           [fx  0 cx]
      K = [ 0 fy cy]
          [ 0  0  1] */
    double _D[4];
    /** The distortion parameters, size depending on the distortion model.
       For us, the 4 parameters are: (k1, k2, t1, t2).*/
  };


  // 需要从flash中读取数据后进行格式转换
  struct CameraParameter {
    double _TSC[16];  //
    /** 4X4 matrix from camera to imu */
    int _width;
    /** width */
    int _height;
    /** height */
    double _focal_length[2];
    /** fx,fy */
    double _principal_point[2];
    /** cx,cy */
    double _R[9];
    /** Rectification matrix (stereo cameras only)
       A rotation matrix aligning the camera coordinate system to the ideal
      stereo image plane so that epipolar lines in both stereo images are
      parallel. */

    double _P[12];
    /** Projection/camera matrix
           [fx'  0  cx' Tx]
      P = [ 0  fy' cy' Ty]
          [ 0   0   1   0] */
    double _K[9];
    /** Intrinsic camera matrix for the raw (distorted) images.
           [fx  0 cx]
      K = [ 0 fy cy]
          [ 0  0  1] */
    double _D[4];
    /** The distortion parameters, size depending on the distortion model.
       For us, the 4 parameters are: (k1, k2, t1, t2).*/
      
    float b;
    float bf;
  };

  struct ImseeRawParam {
    size_t size;
    uint8_t param[2048];
  };
  struct ImseeParamSrv {
    ImseeParamSrv() {}
    struct Request {
      int sensorId;   // 传感器ID 
      std::uint64_t timeStamp;
    } request;

    struct Response {
      bool result;  // true 成功， false 失败           首先要判断获取参数是否成功
      int sensorId;  // 传感器ID
      std::map<Resolution, CameraParameter> _left_camera;
      std::map<Resolution, CameraParameter> _right_camera;
      int _camera_channel = 1;
      double _baseline = 0.12;  // 基线 单位 m
      IMUParameter _imu;
      ModuleInfo _device;
      SlamParameter _slam;
      sensor_srvs::SDebugSet _s_debug;
      std::uint64_t timeStamp;
    } response;
  };



}  // namespace sensor_srvs
