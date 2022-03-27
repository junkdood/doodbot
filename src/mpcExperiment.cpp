#include "ros/ros.h"

#include "doodbot/Hardware.h"
#include "doodbot/solver.h"

double distance(DM state, DM state_goal){
    return sqrt(mtimes((state-state_goal).T(),state-state_goal).scalar());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "exp");
    ros::NodeHandle n;
    ros::Time::init();

    ros::Rate loop_rate(10);

    Simulator_Interface dobot;

    Settings settings;
    settings.phaseLength = 200;
    settings.time = 50;
    settings._costWeights.control = 0;
    settings._costWeights.path = 1;
    settings.solverVerbosity = 0;
    settings.ipoptLinearSolver = "mumps";

    arm_model model = {dobot._l0, dobot._l1, dobot._l2};
    constraint_value constraint = {dobot.j0_min, dobot.j0_max, dobot.j1_min, dobot.j1_max, dobot.j2_min, dobot.j2_max, dobot.j1_sub_j2_min, dobot.j1_sub_j2_max, dobot.v_min, dobot.v_max};

    DirectCollocationSolver solver(model, constraint);
    solver.setupProblemColloc(settings);

    KalmanFilter filter(model, settings.time / (double)settings.phaseLength);   

    State initialState, finalState, AEKFq;
    PathCost path = {1, 0, 0, 0, 0};
    DM sol_state, sol_control;
    Pose pose;
    bool ok;


    pose = dobot.Get_Pose();
    filter.reset({pose.x, pose.y, pose.z, pose.r});
    initialState.state = {pose.x, pose.y, pose.z, pose.r};
    finalState.state = {pose.x + 50, pose.y + 60, pose.z, pose.r};



    while(distance(initialState.state, finalState.state) > 1){
        AEKFq.state = filter.getq();
        ok = solver.solveColloc(initialState, finalState, AEKFq, path);
        if(ok) solver.getSolutionColloc(sol_state, sol_control);

        dobot.Send_Ctrl_Cmd(sol_control(0,0).scalar(), sol_control(1,0).scalar(), sol_control(2,0).scalar(), sol_control(3,0).scalar(), settings.time / (double)settings.phaseLength);   
        
        pose = dobot.Get_Pose();

        DM X = filter.AEKF_unity(sol_control(Slice(),0), {pose.x, pose.y, pose.z, pose.r});
        // ROS_INFO("\nx:%f\ny:%f\nz:%f\nr:%f\n", pose.x, pose.y, pose.z, pose.r);
        ROS_INFO("\nKFx:%f\nKFy:%f\nKFz:%f\nFr:%f\n", X(0).scalar(), X(1).scalar(), X(2).scalar(), X(3).scalar());

        initialState.state = X;
        loop_rate.sleep();
    }

}