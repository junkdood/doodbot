#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "dobot/Board.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "vis");
    ros::NodeHandle n;
    ros::Publisher msg_pub = n.advertise<dobot::Board>("vis_msg", 1000);
    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok()){
        dobot::Board msg;
        msg.header.stamp = ros::Time::now();
        msg.grid_size = 40.0;
        msg_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
