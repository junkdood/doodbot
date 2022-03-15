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


    void draw_board();


    void moveto_offline(DM destination);
    void log_pose();
    void log_board();
    bool moving();

    private:
    void callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    double distance(Pose pose0, Pose pose1);

    Interface* dobot;
    DirectCollocationSolver* solver;
    KalmanFilter* filter;

    ros::NodeHandle n;
    ros::Subscriber msg_sub;

    int board[3][3];
};