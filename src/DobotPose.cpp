#include "ros/ros.h"
#include <cstdlib>
#include "doodbot/SetCmdTimeout.h"
#include "doodbot/GetPose.h"
#include "doodbot/GetHOMEParams.h"
#include "doodbot/SetHOMEParams.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    // SetCmdTimeout
    ros::ServiceClient client;
    client = n.serviceClient<doodbot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    client.waitForExistence();
    doodbot::SetCmdTimeout srv;
    srv.request.timeout = 3000;
    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }
    do{
        client = n.serviceClient<doodbot::GetPose>("/DobotServer/GetPose");
        doodbot::GetPose srv;
        if (client.call(srv) == false) {
            ROS_ERROR("Failed to call GetPose.");
            return -1;
        }
        ROS_INFO("\nx:%f\ny:%f\nz:%f\nr:%f\njointAngle:\n%f\n%f\n%f\n%f\n", srv.response.x, srv.response.y, srv.response.z, srv.response.r, srv.response.jointAngle[0], srv.response.jointAngle[1], srv.response.jointAngle[2], srv.response.jointAngle[3]);
        
        // client = n.serviceClient<doodbot::GetHOMEParams>("/DobotServer/GetHOMEParams");
        // doodbot::GetHOMEParams srv1;
        // if (client.call(srv1) == false) {
        //     ROS_ERROR("Failed to call GetHOMEParams.");
        //     return -1;
        // }
        // ROS_INFO("\nx:%f\ny:%f\nz:%f\n", srv1.response.x, srv1.response.y, srv1.response.z);
        
        ros::Duration(0.5).sleep();
    }while(ros::ok());
    return 0;
}
