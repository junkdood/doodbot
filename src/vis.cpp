#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"
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
        msg.grid_size = 20.0;
        msg.board_pose.position.x = 75.0;
        msg.board_pose.position.y = 75.0;
        msg.board_pose.position.z = -100.0;
        msg.board_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        for(int i = 0;i < 1;i++){
            geometry_msgs::Point point;
            point.x = 105.0;
            point.y = 25.0;
            point.z = -100.0;
            msg.chess_Point.push_back(point);
            msg.chess_type.push_back(dobot::Board::white);
        }
        msg_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
