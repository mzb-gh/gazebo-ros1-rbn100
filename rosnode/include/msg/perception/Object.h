//
// Created by guest on 2022/3/21.
//
#pragma once

namespace perception_msgs
{
    using LOCATION_TYPE = double;

    struct Location
    {
        LOCATION_TYPE x;
        LOCATION_TYPE y;
        LOCATION_TYPE z;
    };

    struct Object
    {
        enum StatusOperate
        {
            INITED_DL = 0, UPDATE_DL, //  [dl]
            INITED_SLAM, UPDATE_SLAM, //  [slam]
        };
        enum ClassID
        {
            UNKNOWN = 0, PERSON, ESCALATOR, HANDRAILS, PERSON_DUMMY
        };

        enum Status
        {
            ENABLE = 0, DISABLE
        };

        int id = -1;    // object ID
        ClassID classID = UNKNOWN; // senmatic id
        // convex for object; one point for person, two point for escalator
        std::vector <Location> locationWorld; // convex
        std::uint64_t time;
        StatusOperate statusOperator; // the status of operator
        signed long int nodeID;       // node id for slam [slam]
        bool history = true; // is load from offline map [slam]
        Status status = DISABLE; // for navigation
    };



}

