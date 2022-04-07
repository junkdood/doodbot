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


    void draw_board();//画棋盘
    void draw_X(double i, double j);//在ij位置画X
    void draw_O(double i, double j);//在ij位置画O
    void letsplay();//对弈主逻辑

    void playgamepad();//手柄控制
    
    void log_pose();//输出位姿
    void log_board();//输出棋盘
    

    private:
    void moveSto_offline(DM destination);//直线
    void moveC_offline(double circleX, double circleY);//圆圈
    void moveG_speed(double x, double y, double z, double r);//手柄控制，发送速度
    void endEffector(float r,bool effector);//末端控制器，未实现
    void defaultPose();//重置默认位置
    bool moving();//判断机械臂是否在移动
    int gameOver();//判断游戏是否结束，可以考虑整合到player里面
    void callback(const std_msgs::Int32MultiArray::ConstPtr& msg);//用于获取棋盘状态的回调函数
    void gamepadcallback(const doodbot::Gamepad::ConstPtr& msg);//由于获取手柄信息的回调函数
    double distance(Pose pose0, Pose pose1);//计算位姿直接距离

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