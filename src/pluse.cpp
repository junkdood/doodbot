#include "ros/ros.h"
#include "DobotDll.h"

int main(int argc, char **argv){
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            getchar();
            return -3;
        break;
        default:
        break;
    }

    ros::init(argc, argv, "pluse");

    Pose pose;
    if(GetPose(&pose)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", pose.x, pose.y, pose.z, pose.jointAngle[0], pose.jointAngle[1], pose.jointAngle[2], pose.jointAngle[3]);

    IOPWM PWM;
    if(GetIOPWM(&PWM)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    ROS_INFO("\nfrequency:%f\ndutyCycle:%f\n",PWM.frequency, PWM.dutyCycle);

    PluseCmd cmd;
    cmd.j1=0;
    cmd.j2=0;
    cmd.j3=500;
    cmd.j4=0;
    cmd.e1=0;
    cmd.e2=0;
    uint64_t queuedCmdIndex;
    int res = SendPluseEx(&cmd);
    if (res == DobotCommunicate_NoError) {
        ROS_INFO("Success");
    }
    else{
        ROS_INFO("Faild, error code: %d",res);
    }

    if(GetPose(&pose)!=DobotCommunicate_NoError){
        throw "Communicate Failed";
    }
    ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", pose.x, pose.y, pose.z, pose.jointAngle[0], pose.jointAngle[1], pose.jointAngle[2], pose.jointAngle[3]);


    DisconnectDobot();

    return 0;
}