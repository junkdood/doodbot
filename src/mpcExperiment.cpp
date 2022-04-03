#include "ros/ros.h"

#include "doodbot/Hardware.h"
#include "doodbot/solver.h"

std::ofstream track("./src/doodbot/MPCdata/track.csv",std::ios::out);
std::ofstream goal("./src/doodbot/MPCdata/goal.csv",std::ios::out);

double distance(DM state, DM state_goal){
    return sqrt(mtimes((state-state_goal).T(),state-state_goal).scalar());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "exp");
    ros::NodeHandle n;
    ros::Time::init();

    ros::Rate loop_rate(10);

    Simulator_Interface dobot;

    double cost_threshold = 1;
    // double cost_threshold = 0.000075;
    // double cost_threshold = 0.000050;
    // double cost_threshold = 0.000025;
    // double cost_threshold = 0;
    Settings settings;
    settings.phaseLength = 200;
    settings.time = 50;
    settings._costWeights.control = (double)1 - cost_threshold;
    settings._costWeights.path = cost_threshold;
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


    track<<"x,"<<"y,"<<"z,"<<"r"<<std::endl;
    goal<<"x,"<<"y,"<<"z,"<<"r"<<std::endl;


    pose = dobot.Get_Pose();
    filter.reset({pose.x, pose.y, pose.z, pose.r});
    initialState.state = {pose.x, pose.y, pose.z, pose.r};
    finalState.state = {pose.x + 50, pose.y + 60, pose.z, pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver.solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver.getSolutionColloc(sol_state, sol_control);
    // for(int i = 0; i <  sol_state.size2(); ++i){
    //     goal<<std::setprecision(12)<<sol_state(0,i).scalar()<<","<<std::setprecision(12)<<sol_state(1,i).scalar()<<","<<std::setprecision(12)<<sol_state(2,i).scalar()<<","<<std::setprecision(12)<<sol_state(3,i).scalar()<<std::endl;
    // }
    for(int i = 0; i <  sol_control.size2() - 1; ++i){
        pose = dobot.Get_Pose();
        goal<<std::setprecision(12)<<pose.x<<","<<std::setprecision(12)<<pose.y<<","<<std::setprecision(12)<<pose.z<<","<<std::setprecision(12)<<pose.r<<std::endl;
        dobot.Send_Ctrl_Cmd(sol_control(0,i).scalar(), sol_control(1,i).scalar(), sol_control(2,i).scalar(), sol_control(3,i).scalar(), settings.time / (double)settings.phaseLength);
    }
    pose = dobot.Get_Pose();
    goal<<std::setprecision(12)<<pose.x<<","<<std::setprecision(12)<<pose.y<<","<<std::setprecision(12)<<pose.z<<","<<std::setprecision(12)<<pose.r<<std::endl;


    // while(distance(initialState.state, finalState.state) > 1){
    //     AEKFq.state = filter.getq();
    //     ok = solver.solveColloc(initialState, finalState, AEKFq, path);
    //     if(ok) solver.getSolutionColloc(sol_state, sol_control);

    //     dobot.Send_Ctrl_Cmd(sol_control(0,0).scalar(), sol_control(1,0).scalar(), sol_control(2,0).scalar(), sol_control(3,0).scalar(), settings.time / (double)settings.phaseLength);   
        
    //     pose = dobot.Get_Pose();

    //     DM X = filter.AEKF_unity(sol_control(Slice(),0), {pose.x, pose.y, pose.z, pose.r});
    //     // ROS_INFO("\nx:%f\ny:%f\nz:%f\nr:%f\n", pose.x, pose.y, pose.z, pose.r);
    //     ROS_INFO("\nKFx:%f\nKFy:%f\nKFz:%f\nFr:%f\n", X(0).scalar(), X(1).scalar(), X(2).scalar(), X(3).scalar());
    //     track<<std::setprecision(12)<<X(0).scalar()<<","<<std::setprecision(12)<<X(1).scalar()<<","<<std::setprecision(12)<<X(2).scalar()<<","<<std::setprecision(12)<<X(3).scalar()<<std::endl;

    //     initialState.state = X;
    //     loop_rate.sleep();
    // }


    track.close();
    goal.close();
    return 0;
}