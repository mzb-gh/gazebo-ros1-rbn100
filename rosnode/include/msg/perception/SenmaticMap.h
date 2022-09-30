//
// Created by guest on 2022/3/21.
//
#pragma once
#include <string>
#include "Object.h"

namespace perception_msgs
{
    struct SenmaticMap
    {
        enum Status
        {
            NO_UPDATE = 0, UPDATE
        };

        std::vector <Object> objects;
        bool finalUpdate = false;    //  final update befor the task close;
        Status status = NO_UPDATE;
    };
}




