#include "dobot/Hardware.h"
#include "dobot/solver.h"


void xyz_to_jointAngle(float x, float y, float z, float jointAngle[4]){
    double _RPD=acos(-1)/180;
    double _DPR=180/acos(-1);
    double _l0 = 138;
    double _l1 = 135;
    double _l2 = 147;
    double _l1_2 = _l1*_l1;
    double _l2_2 = _l2*_l2;
    double r_2 = x*x + y*y;
    double d_2 = r_2 + z*z;
    double d = sqrt(d_2);
    jointAngle[0]=asin(y/sqrt(r_2));
    jointAngle[1]=acos(z/d) - acos((d_2 + _l1_2 - _l2_2)/(2*_l1*d));
    jointAngle[2]=acos((d_2 + _l2_2 - _l1_2)/(2*_l2*d)) - asin(z/d);
    return;
}

int main(int argc, char **argv){
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    ros::init(argc, argv, "test");
    

    // Hardware_Interface test(argv[1]);
    


    // control test
    // JOGJointParams init_Params = test.Get_Params();
    // ROS_INFO("\nvelocity1:%f\nacceleration1:%f\n", init_Params.velocity[0],init_Params.acceleration[0]);
    // JOGCommonParams init_Ratio = test.Get_Ratio();
    // ROS_INFO("\nvelocityR:%f\naccelerationR:%f\n", init_Ratio.velocityRatio,init_Ratio.accelerationRatio);
    // Pose init_pose = test.Get_Pose();
    // ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", init_pose.x, init_pose.y, init_pose.z, init_pose.jointAngle[0], init_pose.jointAngle[1], init_pose.jointAngle[2], init_pose.jointAngle[3]);
    // test.Send_Ctrl_Cmd(4000, Joint1_n);
    // ros::Duration(1.0).sleep();
    // Pose final_pose = test.Get_Pose();
    // ROS_INFO("\nx:%f\ny:%f\nz:%f\njointAngle:\n%f\n%f\n%f\n%f\n", final_pose.x, final_pose.y, final_pose.z, final_pose.jointAngle[0], final_pose.jointAngle[1], final_pose.jointAngle[2], final_pose.jointAngle[3]);
    


    //pose test
    // while(ros::ok){
    //     Pose init_pose = test.Get_Pose();
    //     float my_jointAngle[4] = {0};
    //     float my_x = 0;
    //     float my_y = 0;
    //     float my_z = 0;

    //     test.xyz_to_jointAngle(init_pose.x, init_pose.y, init_pose.z, my_jointAngle);
    //     test.jointAngle_to_xyz(init_pose.jointAngle, my_x, my_y, my_z);

    //     printf("x: %f  %f\n",init_pose.x, my_x);
    //     printf("y: %f  %f\n",init_pose.y, my_y);
    //     printf("z: %f  %f\n",init_pose.z, my_z);

    //     printf("J1: %f  %f\n",init_pose.jointAngle[0], my_jointAngle[0]);
    //     printf("J2: %f  %f\n",init_pose.jointAngle[1], my_jointAngle[1]);
    //     printf("J3: %f  %f\n",init_pose.jointAngle[2], my_jointAngle[2]);

    //     printf("=============================================================\n");

    //     ros::Duration(0.1).sleep();
    // }


    //key ctrl test
    // test.Key_Ctrl();


    



    Settings co_settings;
    co_settings.phaseLength = 10;
    co_settings.time = 1;
    co_settings._costWeights.control = 1;
    co_settings.solverVerbosity = 0;
    co_settings.ipoptLinearSolver = "mumps";

    State initialState, finalState;
    initialState.state = {75, 0, 0, 0};
    finalState.state = {80, 10, 0, 0};

    arm_model model = {138, 135, 147};
    double RPD=acos(-1)/180;
    constraint_value constraint = {-90 * RPD, 90 * RPD, 0 * RPD, 85 * RPD, -10 * RPD, 90 * RPD, -60 * RPD, 50 * RPD, -15 * RPD, 15 * RPD};

    DirectCollocationSolver co_solver(model, constraint);
    co_solver.setupProblemColloc(co_settings);
    bool co_ok = co_solver.solveColloc(initialState, finalState);
    DM co_sol_state, co_sol_control;
    if(co_ok) co_solver.getSolutionColloc(co_sol_state, co_sol_control);
    std::cout<<"collocation\n";
    std::cout << "state:\n" << co_sol_state << "\ncontrol:\n" << co_sol_control << std::endl;

    //check
    float jointAngle[4] = {0};
    for(int i = 0; i <  co_settings.phaseLength; ++i){
        xyz_to_jointAngle(co_sol_state(0,i).scalar(),co_sol_state(1,i).scalar(),co_sol_state(2,i).scalar(),jointAngle);
        std::cout << jointAngle[0]<< ", " << jointAngle[1]<< ", " << jointAngle[2] << std::endl;

    }
    
    return 0;
}
