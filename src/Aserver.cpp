#include <ros/ros.h>
#include <roscpp_tutorials/TwoInts.h>
 
bool callback(roscpp_tutorials::TwoInts::Request &request, roscpp_tutorials::TwoInts::Response &response) {
    response.sum = request.a + request.b;
    return true;
}
 
int main(int argc, char *argv[]) {
 
    ros::init(argc, argv, "service_node");
    ros::NodeHandle nodeHandle;
    const ros::ServiceServer &server = nodeHandle.advertiseService("my_service", callback);
    ros::spin();
 
}