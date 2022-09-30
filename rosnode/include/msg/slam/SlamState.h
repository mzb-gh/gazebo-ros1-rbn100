#pragma once
#include <cstdint>

namespace slam_msgs {

enum SlamSystemState {
    SLAM_SYSTEM_INIT_SUCCESS = 0,  //系统初始化成功--------->机器人正常工作
    OK = 2,          //紧耦合正常--------->机器人正常工作
    LOOP_SUCCESS = 3,    //发现回环
    RELOCATION_SUCCESS = 4,    //重定位成功
    MAPSAVE_SUCCESS = 6,       //保存地图成功
    MAPLOAD_SUCCESS = 7,       //加载地图成功
    SLAM_RESET_FINISHED = 8,   // SLAM重置成功
    SLAM_FINISHED_SUCCESS = 9,        // SLAM完成
    SLAM_STOP_REC = 10,        // SLAM停止接收数据
    SLAM_RESTART_REC = 11,     // SLAM重新接收数据
    SLAM_RESAVE_MAP = 12       // SLAM判断当前地图需要更新

};

struct SlamState {
    std::uint64_t s_timestamp; 
    SlamSystemState s_slamstate;
};

}  // namespace slam_msgs

