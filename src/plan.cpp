#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "dobot/Board.h"

#include "dobot/Hardware.h"
#include "dobot/solver.h"
#include "dobot/player.h"



void msgCallback(const dobot::Board::ConstPtr &msg){
    std::cout << "msg->grid_size=" << msg->grid_size << std::endl;
    ChessPlayer player;
    player.SetState(1);
    player.PlayChess();
    std::cout << "location=" << player.Getlocation() << std::endl;
}

int main(int argc, char **argv){
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Time::init();
    

    // Hardware_Interface test(argv[1]);
    


    // control test
    // JOGJointParams init_Params = test.Get_Params();
    // ROS_INFO("\nvelocity1:%f\nacceleration1:%f\n", init_Params.velocity[0],init_Params.acceleration[0]);
    // JOGCommonParams init_Ratio = test.Get_Ratio();
    // ROS_INFO("\nvelocityR:%f\naccelerationR:%f\n", init_Ratio.velocityRatio,init_Ratio.accelerationRatio);
    // Pose init_pose = test.Get_Pose();
    // ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", init_pose.x, init_pose.y, init_pose.z, init_pose.jointAngle[0], init_pose.jointAngle[1], init_pose.jointAngle[2], init_pose.jointAngle[3]);
    // test.Send_Ctrl_Cmd(4000, Joint1_n);
    // ros::Duration(1.0).sleep();
    // Pose final_pose = test.Get_Pose();
    // ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", final_pose.x, final_pose.y, final_pose.z, final_pose.jointAngle[0], final_pose.jointAngle[1], final_pose.jointAngle[2], final_pose.jointAngle[3]);
    


    //pose test
    // while(ros::ok){
    //     Pose init_pose = test.Get_Pose();
    //     float my_jointAngle[4] = {0};
    //     float my_x = 0;
    //     float my_y = 0;
    //     float my_z = 0;

    //     test.xyz_to_jointAngle(init_pose.x, init_pose.y, init_pose.z, my_jointAngle);
    //     test.jointAngle_to_xyz(init_pose.jointAngle, my_x, my_y, my_z);

    //     printf("x: %f  %f\n",init_pose.x, my_x);
    //     printf("y: %f  %f\n",init_pose.y, my_y);
    //     printf("z: %f  %f\n",init_pose.z, my_z);

    //     printf("J1: %f  %f\n",init_pose.jointAngle[0], my_jointAngle[0]);
    //     printf("J2: %f  %f\n",init_pose.jointAngle[1], my_jointAngle[1]);
    //     printf("J3: %f  %f\n",init_pose.jointAngle[2], my_jointAngle[2]);

    //     printf("=============================================================\n");

    //     ros::Duration(0.1).sleep();
    // }


    //key ctrl test
    // test.Key_Ctrl();


    


    //solver
    // Settings settings;
    // settings.phaseLength = 10;
    // settings.time = 1;
    // settings._costWeights.control = 1;
    // settings.solverVerbosity = 0;
    // settings.ipoptLinearSolver = "mumps";
    

    // arm_model model = {138, 135, 147};
    // double RPD=acos(-1)/180;
    // constraint_value constraint = {-90 * RPD, 90 * RPD, 0 * RPD, 85 * RPD, -10 * RPD, 90 * RPD, -60 * RPD, 50 * RPD, -15 * RPD, 15 * RPD};

    // State initialState, finalState;
    // initialState.state = {75, 0, 0, 0};
    // finalState.state = {80, 10, 0, 0};

    // DirectCollocationSolver solver(model, constraint);
    // solver.setupProblemColloc(settings);
    // bool ok = solver.solveColloc(initialState, finalState);
    // DM sol_state, sol_control;
    // if(ok) solver.getSolutionColloc(sol_state, sol_control);
    // std::cout<<"collocation\n";
    // std::cout << "state:\n" << sol_state << "\ncontrol:\n" << sol_control << std::endl;

    //check
    // auto xyz_to_jointAngle=[](float x, float y, float z, float jointAngle[4]){
    //     double _RPD=acos(-1)/180;
    //     double _DPR=180/acos(-1);
    //     double _l0 = 138;
    //     double _l1 = 135;
    //     double _l2 = 147;
    //     double _l1_2 = _l1*_l1;
    //     double _l2_2 = _l2*_l2;
    //     double r_2 = x*x + y*y;
    //     double d_2 = r_2 + z*z;
    //     double d = sqrt(d_2);
    //     jointAngle[0]=asin(y/sqrt(r_2));
    //     jointAngle[1]=acos(z/d) - acos((d_2 + _l1_2 - _l2_2)/(2*_l1*d));
    //     jointAngle[2]=acos((d_2 + _l2_2 - _l1_2)/(2*_l2*d)) - asin(z/d);
    //     return;
    // };
    // float jointAngle[4] = {0};
    // for(int i = 0; i <  settings.phaseLength; ++i){
    //     xyz_to_jointAngle(sol_state(0,i).scalar(),sol_state(1,i).scalar(),sol_state(2,i).scalar(),jointAngle);
    //     std::cout << jointAngle[0]<< ", " << jointAngle[1]<< ", " << jointAngle[2] << std::endl;

    // }


    //专门与dobot交互的对象
    // Hardware_Interface dobot_interface(argv[1]);
    Simulator_Interface dobot_interface;

    //求解器的基础设置
    Settings settings;
    settings.phaseLength = 50;
    settings.time = 5;
    settings._costWeights.control = 1;
    settings.solverVerbosity = 0;
    settings.ipoptLinearSolver = "mumps";

    //获取dobot的物理模型
    arm_model model = {dobot_interface._l0, dobot_interface._l1, dobot_interface._l2};
    double RPD=dobot_interface._RPD;
    constraint_value constraint = {-90 * RPD, 90 * RPD, 0 * RPD, 85 * RPD, -10 * RPD, 90 * RPD, -60 * RPD, 50 * RPD, -15 * RPD, 15 * RPD};

    //用dobot物理模型创建一个求解器
    DirectCollocationSolver solver(model, constraint);
    //配置求解器
    solver.setupProblemColloc(settings);

    
    State initialState, finalState, Kalman;
    //设置开始状态，读取xyz坐标
    Pose init_pose = dobot_interface.Get_Pose();
    initialState.state = {init_pose.x, init_pose.y, init_pose.z, 0};
    ROS_INFO("\nx:%f\ny:%f\nz:%f\n", init_pose.x, init_pose.y, init_pose.z);

    //设置目标点
    finalState.state = {init_pose.x + 5, init_pose.y, init_pose.z, 0};

    //设置卡尔曼滤波器的参数
    Kalman.state = {0, 0, 0, 0};

    //求解
    bool ok = solver.solveColloc(initialState, finalState, Kalman);

    //获取求解结果，即控制量
    DM sol_state, sol_control;
    if(ok) solver.getSolutionColloc(sol_state, sol_control);

    //发送控制
    for(int i = 0; i <  settings.phaseLength; ++i){
        dobot_interface.Send_Ctrl_Cmd(sol_control(0,i).scalar(), sol_control(1,i).scalar(), sol_control(2,i).scalar(), sol_control(3,i).scalar(), settings.time / (double)settings.phaseLength);
    }
    
    //检查最后是否到达
    ros::Duration(1.0).sleep();
    Pose final_pose = dobot_interface.Get_Pose();
    ROS_INFO("\nx:%f\ny:%f\nz:%f\n", final_pose.x, final_pose.y, final_pose.z);


    
    //再规划一次
    init_pose = dobot_interface.Get_Pose();
    initialState.state = {init_pose.x, init_pose.y, init_pose.z, 0};
    ROS_INFO("\nx:%f\ny:%f\nz:%f\n", init_pose.x, init_pose.y, init_pose.z);
    finalState.state = {init_pose.x, init_pose.y + 5, init_pose.z, 0};
    ok = solver.solveColloc(initialState, finalState, Kalman);
    if(ok) solver.getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  settings.phaseLength; ++i){
        dobot_interface.Send_Ctrl_Cmd(sol_control(0,i).scalar(), sol_control(1,i).scalar(), sol_control(2,i).scalar(), sol_control(3,i).scalar(), settings.time / (double)settings.phaseLength);
    }
    ros::Duration(1.0).sleep();
    final_pose = dobot_interface.Get_Pose();
    ROS_INFO("\nx:%f\ny:%f\nz:%f\n", final_pose.x, final_pose.y, final_pose.z);


    //vis
    // ros::Subscriber msg_sub = n.subscribe("vis_msg", 100, msgCallback);
    // ros::spin();
    
    return 0; 
}
