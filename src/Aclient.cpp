#include <ros/ros.h>
#include <iostream>
#include <roscpp_tutorials/TwoInts.h>
using namespace std;
int main(int argc, char *argv[]) {
    ros::init(argc,argv,"client_node");
    ros::NodeHandle nodeHandle;
    ros::ServiceClient client = nodeHandle.serviceClient<roscpp_tutorials::TwoInts>("my_service");
    client.waitForExistence();
    roscpp_tutorials::TwoInts twoInts;
    twoInts.request.b = 10;
    twoInts.request.a = 20;
    while(ros::ok()) {
        if(client.call(twoInts)) {
            cout << "Result: " << twoInts.response.sum << endl;
        } else {
            ROS_ERROR("Failed to call service");
        }
        ros::spinOnce();
    }
}