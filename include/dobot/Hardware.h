#pragma once

#include <ros/ros.h>
#include <DobotDll.h>

#include<bits/stdc++.h>
#include <termios.h>


enum joint_set{IDLE, Joint1_p, Joint1_n, Joint2_p, Joint2_n, Joint3_p, Joint3_n, Joint4_p, Joint4_n};

class Hardware_Interface{
    public:
    Hardware_Interface(char* port, unsigned int timeout = 3000);
    ~Hardware_Interface();

    void Set_Home();
    Pose Get_Pose();
    void Set_Params(JOGJointParams params);
    JOGJointParams Get_Params();
    void Set_Ratio();
    JOGCommonParams Get_Ratio();
    void Send_Ctrl_Cmd(uint32_t duration, joint_set target_joint);
    void Send_Ctrl_Cmd(float j0, float j1, float j2, float j3, double dt);

    void Key_Ctrl();

    void xyz_to_jointAngle(float x, float y, float z, float jointAngle[4]);

    void jointAngle_to_xyz(float jointAngle[4], float &x, float &y, float &z);

    
    const double _RPD=acos(-1)/180;
    const double _DPR=180/acos(-1);
    const double _l0 = 138;
    const double _l1 = 135;
    const double _l2 = 147;

    private:
    const double _l1_2 = _l1*_l1;
    const double _l2_2 = _l2*_l2;


    // -90 < jointAngle[0] < 90
    // 0 < jointAngle[1] < 85
    // -10 < jointAngle[2] < 90
    // jointAngle[1] - 50 < jointAngle[2] < jointAngle[1] + 60 //手测
};

class Simulator_Interface{
    public:
    Simulator_Interface(float x = 75, float y = 0, float z = 0);
    ~Simulator_Interface();

    void Set_Home();
    Pose Get_Pose();
    void Send_Ctrl_Cmd(float j0, float j1, float j2, float j3, double dt);
    void xyz_to_jointAngle(float x, float y, float z, float jointAngle[4]);
    void jointAngle_to_xyz(float jointAngle[4], float &x, float &y, float &z);

    
    const double _RPD=acos(-1)/180;
    const double _DPR=180/acos(-1);
    const double _l0 = 138;
    const double _l1 = 135;
    const double _l2 = 147;

    private:
    bool check_valid();
    float _sim_jointAngle[4];
    float _home_jointAngle[4];
    const double _l1_2 = _l1*_l1;
    const double _l2_2 = _l2*_l2;


    // -90 < jointAngle[0] < 90
    // 0 < jointAngle[1] < 85
    // -10 < jointAngle[2] < 90
    // 50 > jointAngle[1] - jointAngle[2] > -60 //手测
};
