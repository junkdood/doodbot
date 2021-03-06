#pragma once

#include <DobotDll.h>

#include<bits/stdc++.h>
#include <unistd.h>

double generateGaussianNoise(double mu, double sigma);
enum joint_set{IDLE, Joint1_p, Joint1_n, Joint2_p, Joint2_n, Joint3_p, Joint3_n, Joint4_p, Joint4_n};

class Interface{
    public:
    Interface(){};
    ~Interface(){};

    //常用接口规范
    virtual void Set_Home() = 0;//重置位置
    virtual Pose Get_Pose() = 0;//获取位置
    virtual void Send_Ctrl_Cmd(float j0, float j1, float j2, float j3, double dt) = 0;//标准控制，发送机械臂速度
    virtual void Send_CP_Cmd(float x, float y, float z, float r) = 0;//连续运动轨迹功能，绝对位置模式
    virtual void Send_CP_Cmd_0(float x, float y, float z, float r) = 0;//连续运动轨迹功能，相对位置模式
    virtual void Send_END_Cmd(float r,bool effector) = 0;//末端控制。未完成

    //物理模型相关
    void xyz_to_jointAngle(float x, float y, float z, float r, float jointAngle[4]);//笛卡尔坐标转关节坐标
    void jointAngle_to_xyz(float jointAngle[4], float &x, float &y, float &z, float &r);//关节坐标转笛卡尔坐标
    
    //常量
    const double _RPD=acos(-1)/180;//角度转弧度
    const double _DPR=180/acos(-1);//弧度转角度
    const double _l0 = 138;//底座高度
    const double _l1 = 135;//大臂长度
    const double _l2 = 147;//小臂长度

    //机械臂约束
    const double j0_min = -90 * _RPD;
    const double j0_max = 90 * _RPD;
    const double j1_min = 0 * _RPD;
    const double j1_max = 85 * _RPD;
    const double j2_min = -10 * _RPD;
    const double j2_max = 90 * _RPD;
    const double j1_sub_j2_min = -60 * _RPD;
    const double j1_sub_j2_max = 50 * _RPD;

    //控制速度约束
    const double v_min = -1 * _RPD;
    const double v_max = 1 * _RPD;

    protected:
    const double _l1_2 = _l1*_l1;
    const double _l2_2 = _l2*_l2;
};

class Hardware_Interface : public Interface{
    public:
    Hardware_Interface(char* port, unsigned int timeout = 3000);
    ~Hardware_Interface();

    void Set_Home();
    Pose Get_Pose();
    void Set_Params(JOGJointParams params);
    JOGJointParams Get_Params();
    void Set_Ratio();
    JOGCommonParams Get_Ratio();
    void Send_joint_Cmd(uint32_t duration, joint_set target_joint);
    void Send_Ctrl_Cmd(float j0, float j1, float j2, float j3, double dt);
    void Send_CP_Cmd(float x, float y, float z, float r);
    void Send_CP_Cmd_0(float x, float y, float z, float r);
    void Send_END_Cmd(float r,bool effector);
};

class Simulator_Interface : public Interface{
    public:
    Simulator_Interface(float x = 80, float y = 10, float z = 0, float r = 0);
    ~Simulator_Interface();

    void Set_Home();
    Pose Get_Pose();
    void Send_Ctrl_Cmd(float j0, float j1, float j2, float j3, double dt);
    void Send_CP_Cmd(float x, float y, float z, float r);
    void Send_CP_Cmd_0(float x, float y, float z, float r);
    void Send_END_Cmd(float r,bool effector){};

    private:
    bool isValid();
    float _sim_jointAngle[4];
    float _home_jointAngle[4];
};
