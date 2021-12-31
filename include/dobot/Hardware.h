#pragma once

#include <ros/ros.h>
#include <DobotDll.h>


enum joint_set{IDLE, Joint1_p, Joint1_n, Joint2_p, Joint2_n, Joint3_p, Joint3_n, Joint4_p, Joint4_n};

class Hardware_Interface{
    public:
    Hardware_Interface(char* port, unsigned int timeout = 3000);
    ~Hardware_Interface();

    void Set_Home();
    Pose Get_Pose();
    JOGJointParams Get_Params();
    void Set_Ratio();
    JOGCommonParams Get_Ratio();
    void Send_Ctrl_Cmd(int duration, joint_set target_joint);

    private:
    double _RPD=M_PI/180;
};