#include "dobot/Hardware.h"

static struct termios ori_attr, cur_attr;

static __inline 
int tty_reset(void){
    if (tcsetattr(STDIN_FILENO, TCSANOW, &ori_attr) != 0)
            return -1;

    return 0;
}


static __inline
int tty_set(void){
        
    if ( tcgetattr(STDIN_FILENO, &ori_attr) )
            return -1;
    
    memcpy(&cur_attr, &ori_attr, sizeof(cur_attr) );
    cur_attr.c_lflag &= ~ICANON;
    cur_attr.c_lflag &= ~ECHO;
    cur_attr.c_cc[VMIN] = 1;
    cur_attr.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &cur_attr) != 0)
            return -1;

    return 0;
}

static __inline
int kbhit(void) {               
    fd_set rfds;
    struct timeval tv;
    int retval;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec  = 0;
    tv.tv_usec = 0;

    retval = select(1, &rfds, NULL, NULL, &tv);

    if (retval == -1) {
            perror("select()");
            return 0;
    } else if (retval)
            return 1;
    else
            return 0;
    return 0;
}




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
    ros::Duration(0.1).sleep();
}

void Hardware_Interface::Key_Ctrl(){
    tty_set();

    JOGJointParams Initparams = Get_Params();
    JOGJointParams Stopparams = Initparams;
    Stopparams.velocity[0] = 0;
    Stopparams.velocity[1] = 0;
    Stopparams.velocity[2] = 0;
    Set_Params(Stopparams);

    JOGJointParams J1_p_Params = Stopparams;
    J1_p_Params.velocity[0] = 15;
    JOGJointParams J2_p_Params = Stopparams;
    J2_p_Params.velocity[1] = 15;
    JOGJointParams J3_p_Params = Stopparams;
    J3_p_Params.velocity[2] = 15;

    JOGCmd jcmd;
    uint64_t queuedCmdIndex;
    jcmd.isJoint = true;
    jcmd.cmd = Joint1_p;
    if(SetJOGCmd(&jcmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }

    jcmd.cmd = Joint2_p;
    if(SetJOGCmd(&jcmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }

    jcmd.cmd = Joint3_p;
    if(SetJOGCmd(&jcmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    bool flag = true;
    char temp = 'w';
    while(flag){
        usleep (50000);
        if(kbhit()){
            temp=getchar(); 
            switch(temp) {
                case 'q':
                    ROS_INFO("q");
                    Set_Params(J1_p_Params);
                break;
                case 'w':
                    ROS_INFO("w");
                break;
                case 'a':
                    ROS_INFO("a");
                    Set_Params(J2_p_Params);
                break;
                case 's':
                    ROS_INFO("s");
                break;
                case 'z':
                    ROS_INFO("z");
                    Set_Params(J3_p_Params);
                break;
                case 'x':
                    ROS_INFO("x");
                break;
                case 'f':
                    flag = false;
                    ROS_INFO("f");
                break;
                default:
                    ROS_INFO("DEFAULT:%c", temp);
                    Set_Params(Stopparams);
                break;
            }
        }
        else{
            ROS_INFO("Empty");
            Pose init_pose = Get_Pose();
            ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", init_pose.x, init_pose.y, init_pose.z, init_pose.jointAngle[0], init_pose.jointAngle[1], init_pose.jointAngle[2], init_pose.jointAngle[3]);
        }
    }
    jcmd.cmd = IDLE;
    if(SetJOGCmd(&jcmd, true, &queuedCmdIndex)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    Set_Params(Initparams);
    tty_reset();
}

void Hardware_Interface::xyz_to_jointAngle(float x, float y, float z, float jointAngle[4]){
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