#include "dobot/Hardware.h"

int main(int argc, char **argv){
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    ros::init(argc, argv, "test");
    

    Hardware_Interface test(argv[1]);

    JOGJointParams init_Params = test.Get_Params();
    ROS_INFO("\nvelocity1:%f\nacceleration1:%f\n", init_Params.velocity[0],init_Params.acceleration[0]);
    JOGCommonParams init_Ratio = test.Get_Ratio();
    ROS_INFO("\nvelocityR:%f\naccelerationR:%f\n", init_Ratio.velocityRatio,init_Ratio.accelerationRatio);

    Pose init_pose = test.Get_Pose();
    ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", init_pose.x, init_pose.y, init_pose.z, init_pose.jointAngle[0], init_pose.jointAngle[1], init_pose.jointAngle[2], init_pose.jointAngle[3]);
    test.Send_Ctrl_Cmd(4000, Joint1_n);
    ros::Duration(1.0).sleep();
    Pose final_pose = test.Get_Pose();
    ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", final_pose.x, final_pose.y, final_pose.z, final_pose.jointAngle[0], final_pose.jointAngle[1], final_pose.jointAngle[2], final_pose.jointAngle[3]);
    

    
    return 0;
}
