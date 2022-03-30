#pragma once

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "doodbot/Gamepad.h"


#include "doodbot/Hardware.h"
#include "doodbot/solver.h"
#include "doodbot/player.h"



class Doodbot{
    public:
    Doodbot(int argc, char **argv);
    ~Doodbot();


    void draw_board();
    void draw_X(double i, double j);
    void draw_O(double i, double j);
    void letsplay();

    void playgamepad();
    
    void log_pose();
    void log_board();
    

    private:
    void moveSto_offline(DM destination);
    void moveC_offline(double circleX, double circleY);
    void moveG_speed(double x, double y, double z, double r);
    void endEffector(float r,bool effector);
    void defaultPose();
    bool moving();
    int gameOver();
    void callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void gamepadcallback(const doodbot::Gamepad::ConstPtr& msg);
    double distance(Pose pose0, Pose pose1);

    Interface* dobot;
    DirectCollocationSolver* solver;
    KalmanFilter* filter;
    ChessPlayer* player;

    ros::NodeHandle n;
    ros::Subscriber msg_sub;

    int board[3][3];
    int boardcacheO[3][3];
    int boardcacheX[3][3];

    //棋盘左上角的点和格子宽度
    // [190,15,-60,0][190,-15,-60,0][160,15,-60,0][160,-15,-60,0]
    const double boardX = 190;
    const double boardY = 15;
    const double boardStep = 30;
    //第一个棋子的位置
    const double chessX = boardX + boardStep/2;
    const double chessY = boardY + boardStep/2;


    //定义笔不碰到和碰到的高度
    const double penUp = -40;
    const double penDown = -55;

    //常用变量(用之前一定要自己初始化！)
    State initialState, finalState, AEKFq;
    PathCost path;
    DM sol_state, sol_control;
    Pose pose;
    bool ok;
    
};