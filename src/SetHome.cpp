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

    ros::init(argc, argv, "SetHome");

    HOMECmd cmd;
    uint64_t queuedCmdIndex;
    int res = SetHOMECmd(&cmd, true, &queuedCmdIndex);
    if (res == DobotCommunicate_NoError) {
        ROS_INFO("Success");
    }
    else{
        ROS_INFO("Faild, error code: %d",res);
    }

    DisconnectDobot();

    return 0;
}