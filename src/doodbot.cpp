#include "doodbot/doodbot.h"

doodbot::doodbot(int argc, char **argv){
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application mode (portName)");
        throw "usage wrong";
    }
    if(strcmp("sim", argv[1]) == 0){
        dobot = new Simulator_Interface();
    }
    else if(strcmp("real", argv[1]) == 0){
        if (argc < 3) {
            ROS_ERROR("[USAGE]Application real portName");
            throw "usage wrong";
        }
        dobot = new Hardware_Interface(argv[2]);
    }
    else{
        ROS_ERROR("[ERROR]mode not right");
        throw "mode not right";
    }

    Settings settings;
    settings.phaseLength = 20;
    settings.time = 5;
    settings._costWeights.control = 0;
    settings._costWeights.path = 1;
    settings.solverVerbosity = 0;
    settings.ipoptLinearSolver = "mumps";

    arm_model model = {dobot->_l0, dobot->_l1, dobot->_l2};
    constraint_value constraint = {dobot->j0_min, dobot->j0_max, dobot->j1_min, dobot->j1_max, dobot->j2_min, dobot->j2_max, dobot->j1_sub_j2_min, dobot->j1_sub_j2_max, dobot->v_min, dobot->v_max};

    solver = new DirectCollocationSolver(model, constraint);
    solver->setupProblemColloc(settings);

    filter = new KalmanFilter(model, settings.time / (double)settings.phaseLength);    

    for(int i = 0; i < 3; i++){
        for(int j = 0;j < 3;j++){
            board[i][j] = 0;
        }
    }

    msg_sub = n.subscribe("OXstate", 100, &doodbot::callback, this);
}

doodbot::~doodbot(){
    delete dobot;
    delete solver;
    delete filter;
}

void doodbot::draw_board(){
    // [190,15,-60,0][190,-15,-60,0][160,15,-60,0][160,-15,-60,0]

    moveSto_offline({190,45,-40,0});
    moveSto_offline({190,45,-60,0});
    moveSto_offline({190,-45,-60,0});
    moveSto_offline({190,-45,-40,0});
    log_pose();

    moveSto_offline({160,-45,-40,0});
    moveSto_offline({160,-45,-60,0});
    moveSto_offline({160,45,-60,0});
    moveSto_offline({160,45,-40,0});
    log_pose();

    moveSto_offline({130,15,-40,0});
    moveSto_offline({130,15,-60,0});
    moveSto_offline({220,15,-60,0});
    moveSto_offline({220,15,-40,0});
    log_pose();

    moveSto_offline({220,-15,-40,0});
    moveSto_offline({220,-15,-60,0});
    moveSto_offline({130,-15,-60,0});
    moveSto_offline({130,-15,-40,0});
    log_pose();
}

void doodbot::draw_X_test(){
    moveSto_offline({175, 0, -40, 0});
    moveSto_offline({185, 10, -40, 0});
    moveSto_offline({185, 10, -60, 0});
    moveSto_offline({165, -10, -60, 0});
    moveSto_offline({165, -10, -40, 0});
    moveSto_offline({165, 10, -40, 0});
    moveSto_offline({165, 10, -60, 0});
    moveSto_offline({185, -10, -60, 0});
    moveSto_offline({185, -10, -40, 0});
}

void doodbot::draw_O_test(){
    moveSto_offline({175, 0, -40, 0});
    moveSto_offline({175, 10, -40, 0});
    moveSto_offline({175, 10, -60, 0});
    moveC(175, 0);
    moveSto_offline({175, 10, -40, 0});
}

void doodbot::moveSto_offline(DM destination){
    Pose init_pose = dobot->Get_Pose();
    PathCost path = {1, 0, 0, 0, 0};

    State initialState, finalState, AEKFq;
    initialState.state = {init_pose.x, init_pose.y, init_pose.z, init_pose.r};
    finalState.state = destination;
    AEKFq.state = DM::zeros(4);

    DM sol_state, sol_control;
    bool ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());
}

void doodbot::moveC(double circleX, double circleY){
    Pose init_pose = dobot->Get_Pose();
    double circleR = sqrt(pow(init_pose.x - circleX, 2) + pow(init_pose.y - circleY, 2));
    PathCost path = {0, 1, circleX, circleY, circleR};

    State initialState, finalState, AEKFq;
    initialState.state = {init_pose.x, init_pose.y, init_pose.z, init_pose.r};
    finalState.state = {circleX + (circleY - init_pose.y), circleY - (circleX - init_pose.x), init_pose.z, init_pose.r};
    AEKFq.state = DM::zeros(4);

    DM sol_state, sol_control;
    bool ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());


    initialState.state = {circleX + (circleY - init_pose.y), circleY - (circleX - init_pose.x), init_pose.z, init_pose.r};
    finalState.state = {circleX + (circleX - init_pose.x), circleY + (circleY - init_pose.y), init_pose.z, init_pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());


    initialState.state = {circleX + (circleX - init_pose.x), circleY + (circleY - init_pose.y), init_pose.z, init_pose.r};
    finalState.state = {circleX - (circleY - init_pose.y), circleY + (circleX - init_pose.x), init_pose.z, init_pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());


    initialState.state = {circleX - (circleY - init_pose.y), circleY + (circleX - init_pose.x), init_pose.z, init_pose.r};
    finalState.state = {init_pose.x, init_pose.y, init_pose.z, init_pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());
}

void doodbot::log_pose(){
    Pose pose = dobot->Get_Pose();
    ROS_INFO("\nx:%f\ny:%f\nz:%f\nr:%f\n", pose.x, pose.y, pose.z, pose.r);
}

void doodbot::log_board(){
    if(board[0][0] == NOBOARD){
        std::cout<<"No board recognized!"<<std::endl;
        return;
    }
    for(int i = 0; i < 3; i++){
        for(int j = 0;j < 3;j++){
            if(board[i][j] == EMPTY){
                std::cout<<"-"<<" ";
            }
            else if(board[i][j] == O){
                std::cout<<"O"<<" ";
            }
            else if(board[i][j] == X){
                std::cout<<"X"<<" ";
            }
            else{
                std::cout<<board[i][j]<<" ";
            }
        }
        std::cout<<std::endl;
    }
}

bool doodbot::moving(){
    Pose pose0 = dobot->Get_Pose();
    ros::Duration(0.1).sleep();
    Pose pose1 = dobot->Get_Pose();
    return distance(pose0, pose1) > 0.0001;
}

void doodbot::callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
    ROS_INFO("callback");
    for(int i = 0; i < 3; i++){
        for(int j = 0;j < 3;j++){
            board[i][j] = msg->data.at(i*3+j);
        }
    }
}

double doodbot::distance(Pose pose0, Pose pose1){
    return sqrt(pow(pose0.x-pose1.x,2) + pow(pose0.y-pose1.y,2) + pow(pose0.z-pose1.z, 2) + pow(pose0.r - pose1.r, 2));
}