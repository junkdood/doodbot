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

}

void Hardware_Interface::Set_Ratio(){

}

JOGCommonParams Hardware_Interface::Get_Ratio(){

}

void Hardware_Interface::Send_Ctrl_Cmd(int duration, joint_set target_joint){

}