#include "dobot/Hardware.h"

Hardware_Interface::Hardware_Interface(char* port, unsigned int timeout){
    int result = ConnectDobot(port, 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            std::cout << "Dobot not found!" << std::endl;
            throw "Connect Failed";
            return;
        break;
        case DobotConnect_Occupied:
            std::cout << "Invalid port name or Dobot is occupied by other application!" << std::endl;
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
    pose.r *= _RPD;
    pose.jointAngle[0] *= _RPD;
    pose.jointAngle[1] *= _RPD;
    pose.jointAngle[2] *= _RPD;
    pose.jointAngle[3] *= _RPD;
    return pose;
}

void Hardware_Interface::Set_Params(JOGJointParams params){
    uint64_t queuedCmdIndex;
    if(SetJOGJointParams(&params, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    return;
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

void Hardware_Interface::Send_Ctrl_Cmd(float j0, float j1, float j2, float j3, double dt){

    PTPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = PTPMOVJANGLEINCMode;
    cmd.x = j0 * dt * _DPR;
    cmd.y = j1 * dt * _DPR;
    cmd.z = j2 * dt * _DPR;
    cmd.r = j3 * dt * _DPR;
    if(SetPTPCmd(&cmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    usleep (100000);
}


void Hardware_Interface::xyz_to_jointAngle(float x, float y, float z, float r, float jointAngle[4]){
    double R_2 = x*x + y*y;
    double d_2 = R_2 + z*z;
    double d = sqrt(d_2);
    jointAngle[0]=asin(y/sqrt(R_2));
    jointAngle[1]=acos(z/d) - acos((d_2 + _l1_2 - _l2_2)/(2*_l1*d));
    jointAngle[2]=acos((d_2 + _l2_2 - _l1_2)/(2*_l2*d)) - asin(z/d);
    jointAngle[3] = r;
    return;
}

void Hardware_Interface::jointAngle_to_xyz(float jointAngle[4], float &x, float &y, float &z, float &r){
    double R = _l1*sin(jointAngle[1]) + _l2*cos(jointAngle[2]);
    x = R*cos(jointAngle[0]);
    y = R*sin(jointAngle[0]);
    z = _l1*cos(jointAngle[1]) - _l2*sin(jointAngle[2]);
    r = jointAngle[3];
    return;
}






//#############################################################################



Simulator_Interface::Simulator_Interface(float x, float y, float z, float r){
    xyz_to_jointAngle(x, y, z, r, _home_jointAngle);
    Set_Home();
    return;
}

Simulator_Interface::~Simulator_Interface(){
    return;
}

void Simulator_Interface::Set_Home(){
    _sim_jointAngle[0] = _home_jointAngle[0];
    _sim_jointAngle[1] = _home_jointAngle[1];
    _sim_jointAngle[2] = _home_jointAngle[2];
    _sim_jointAngle[3] = _home_jointAngle[3];
    return;
}

Pose Simulator_Interface::Get_Pose(){
    Pose pose;
    jointAngle_to_xyz(_sim_jointAngle, pose.x, pose.y, pose.z, pose.r);
    pose.jointAngle[0] = _sim_jointAngle[0];
    pose.jointAngle[1] = _sim_jointAngle[1];
    pose.jointAngle[2] = _sim_jointAngle[2];
    pose.jointAngle[3] = _sim_jointAngle[3];

    pose.x += generateGaussianNoise(0,sqrt(0.0001));
    pose.y += generateGaussianNoise(0,sqrt(0.0001));
    pose.z += generateGaussianNoise(0,sqrt(0.0001));
    pose.r += generateGaussianNoise(0,sqrt(0.0001));
    return pose;
}

void Simulator_Interface::Send_Ctrl_Cmd(float j0, float j1, float j2, float j3, double dt){
    _sim_jointAngle[0] += j0*dt;
    _sim_jointAngle[1] += j1*dt;
    _sim_jointAngle[2] += j2*dt;
    _sim_jointAngle[3] += j3*dt;

    _sim_jointAngle[0] += generateGaussianNoise(0,sqrt(0.00001));
    _sim_jointAngle[1] += generateGaussianNoise(0,sqrt(0.00001));
    _sim_jointAngle[2] += generateGaussianNoise(0,sqrt(0.00001));
    _sim_jointAngle[3] += generateGaussianNoise(0,sqrt(0.00001));
}

void Simulator_Interface::xyz_to_jointAngle(float x, float y, float z, float r, float jointAngle[4]){
    double R_2 = x*x + y*y;
    double d_2 = R_2 + z*z;
    double d = sqrt(d_2);
    jointAngle[0]=asin(y/sqrt(R_2));
    jointAngle[1]=acos(z/d) - acos((d_2 + _l1_2 - _l2_2)/(2*_l1*d));
    jointAngle[2]=acos((d_2 + _l2_2 - _l1_2)/(2*_l2*d)) - asin(z/d);
    jointAngle[3] = r;
    return;
}

void Simulator_Interface::jointAngle_to_xyz(float jointAngle[4], float &x, float &y, float &z, float &r){
    double R = _l1*sin(jointAngle[1]) + _l2*cos(jointAngle[2]);
    x = R*cos(jointAngle[0]);
    y = R*sin(jointAngle[0]);
    z = _l1*cos(jointAngle[1]) - _l2*sin(jointAngle[2]);
    r = jointAngle[3];
    return;
}

bool Simulator_Interface::isValid(){
    return -90 < _sim_jointAngle[0] < 90 && 0 < _sim_jointAngle[1] < 85 && -10 < _sim_jointAngle[2] < 90 && 50 > _sim_jointAngle[1] - _sim_jointAngle[2] > -60;
}


double generateGaussianNoise(double mu, double sigma){
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;
 
    static double z0, z1;
    static bool generate;
    generate = !generate;
 
    if (!generate)
       return z1 * sigma + mu;
 
    double u1, u2;
    do{
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
    }while ( u1 <= epsilon );
 
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}