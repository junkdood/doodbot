#include "ros/ros.h"
#include "std_msgs/String.h"
#include "doodbot/SetCmdTimeout.h"
#include "doodbot/SetQueuedCmdClear.h"
#include "doodbot/SetQueuedCmdStartExec.h"
#include "doodbot/SetQueuedCmdForceStopExec.h"
#include "doodbot/GetDeviceVersion.h"

#include "doodbot/SetEndEffectorParams.h"
#include "doodbot/SetPTPJointParams.h"
#include "doodbot/SetPTPCoordinateParams.h"
#include "doodbot/SetPTPJumpParams.h"
#include "doodbot/SetPTPCommonParams.h"
#include "doodbot/SetPTPCmd.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    ros::ServiceClient client;

    // SetCmdTimeout
    client = n.serviceClient<doodbot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    client.waitForExistence();
    doodbot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<doodbot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    doodbot::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<doodbot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    doodbot::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<doodbot::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    doodbot::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<doodbot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    doodbot::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    // Set PTP joint parameters
    do {
        client = n.serviceClient<doodbot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        doodbot::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<doodbot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        doodbot::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<doodbot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        doodbot::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<doodbot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        doodbot::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);

    client = n.serviceClient<doodbot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    doodbot::SetPTPCmd srv;

    while (ros::ok()) {
        // The first point
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 200;
            srv.request.y = 0;
            srv.request.z = 0;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);


        // The first point
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 250;
            srv.request.y = 0;
            srv.request.z = 0;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);
  
        ros::spinOnce();
        break;
    }

    return 0;
}

