#include "ros/ros.h"
#include <cstdlib>
#include "dobot/SetCmdTimeout.h"
#include "dobot/GetPose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    // SetCmdTimeout
    ros::ServiceClient client;
    ros::service::waitForService("/DobotServer/SetCmdTimeout");
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv;
    srv.request.timeout = 3000;
    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }
    do{
        client = n.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
        dobot::GetPose srv;
        if (client.call(srv) == false) {
            ROS_ERROR("Failed to call GetPose.");
            return -1;
        }
        ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", srv.response.x, srv.response.y, srv.response.z, srv.response.jointAngle[0], srv.response.jointAngle[1], srv.response.jointAngle[2], srv.response.jointAngle[3]);
        ros::Duration(0.5).sleep();
    }while(true);
    return 0;
}
