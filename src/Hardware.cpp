#include "dobot/Hardware.h"


Hardware_Interface::Hardware_Interface(char* port, unsigned int timeout){
    int result = ConnectDobot(port, 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            throw "Connect Failed";
            return;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            getchar();
            throw "Connect Failed";
            return;
        break;
        default:
        break;
    }

    if(SetCmdTimeout(timeout)!=DobotCommunicate_NoError){
        DisconnectDobot();
        throw "Communicate Failed";
    }
    ros::Time::init();
    return;
}

Hardware_Interface::~Hardware_Interface(){
    DisconnectDobot();
}

void Hardware_Interface::Set_Home(){
    HOMECmd cmd;
    uint64_t queuedCmdIndex;
    if(SetHOMECmd(&cmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
}

Pose Hardware_Interface::Get_Pose(){
    Pose pose;
    if(GetPose(&pose)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    return pose;
}

JOGJointParams Hardware_Interface::Get_Params(){
    JOGJointParams params;
    if(GetJOGJointParams(&params)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    return params;
}

void Hardware_Interface::Set_Ratio(){
    
}

JOGCommonParams Hardware_Interface::Get_Ratio(){
    JOGCommonParams params;
    if(GetJOGCommonParams(&params)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    return params;
}

void Hardware_Interface::Send_Ctrl_Cmd(uint32_t duration, joint_set target_joint){


    if(SetQueuedCmdStopExec()!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }

    if(SetQueuedCmdClear()!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }

    JOGCmd jcmd;
    uint64_t queuedCmdIndex;
    jcmd.isJoint = true;
    jcmd.cmd = target_joint;
    if(SetJOGCmd(&jcmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }

    WAITCmd Wcmd;
    Wcmd.timeout = duration;
    if(SetWAITCmd(&Wcmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }

    // ros::Duration(duration).sleep();

    jcmd.cmd = IDLE;
    if(SetJOGCmd(&jcmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }

    if(SetQueuedCmdStartExec()!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    uint64_t executedCmdIndex;
    do{
        GetQueuedCmdCurrentIndex(&executedCmdIndex);
    }while(executedCmdIndex < queuedCmdIndex);

    return;
}

void Hardware_Interface::xyz_to_jointAngle(float x, float y, float z, float (&jointAngle)[4]){
    double r_2 = x*x + y*y;
    double d_2 = r_2 + z*z;
    double d = sqrt(d_2);
    jointAngle[0]=asin(y/sqrt(r_2));
    jointAngle[1]=acos(z/d) - acos((d_2 + _l1_2 - _l2_2)/(2*_l1*d));
    jointAngle[2]=acos((d_2 + _l2_2 - _l1_2)/(2*_l2*d)) - asin(z/d);
    jointAngle[0] = jointAngle[0]*_DPR;
    jointAngle[1] = jointAngle[1]*_DPR;
    jointAngle[2] = jointAngle[2]*_DPR;
    return;
}

void Hardware_Interface::jointAngle_to_xyz(float jointAngle[4], float &x, float &y, float &z){
    double j0 = jointAngle[0]*_RPD;
    double j1 = jointAngle[1]*_RPD;
    double j2 = jointAngle[2]*_RPD;
    double r = _l1*sin(j1) + _l2*cos(j2);
    x = r*cos(j0);
    y = r*sin(j0);
    z = _l1*cos(j1) - _l2*sin(j2);
    return;
}