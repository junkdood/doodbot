#pragma once

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"


#include "doodbot/Hardware.h"
#include "doodbot/solver.h"
#include "doodbot/player.h"

class doodbot{
    public:
    doodbot(int argc, char **argv);
    ~doodbot();

    private:
    Interface* dobot;
    ros::NodeHandle n;
};