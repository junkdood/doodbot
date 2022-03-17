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

    player = new ChessPlayer();

    for(int i = 0; i < 3; i++){
        for(int j = 0;j < 3;j++){
            board[i][j] = 0;
        }
    }

    msg_sub = n.subscribe("OXstate", 1, &doodbot::callback, this);
}

doodbot::~doodbot(){
    delete dobot;
    delete solver;
    delete filter;
    delete player;
}

void doodbot::letsplay(){
    draw_board();
    while(board[0][0]==NOBOARD){
        ROS_INFO("No board recognized!");
        ros::spinOnce();
    }
    printf("who first, you or doodbot?\n'x' for you\n'o' for doobot\n");
    char first;
    bool flag = true;
    while(1){
        int t = scanf("%c",&first);
        if(first=='x'){
            flag = true;
            break;
        }
        else if(first=='o'){
            flag = false;
            break;
        }
        printf("\nYou press a wrong letter\nplease enter again:");
    }
    while(!gameOver()){
        if(flag){
            printf("press ENTER button to go on after you play your turn\n");
            getchar();
            getchar();
        }
        else{
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
            log_board();
            
            player->SetState(board);
            player->PlayChess();
            draw_O(player->GetI(), player->GetJ());
        }


        flag = !flag;
    }
    if(gameOver() == O){
        printf("doodbot wins!");
    }
    else if(gameOver() == X){
        printf("you wins!");
    }
    else{
        printf("no winer!");
    }
}



void doodbot::draw_board(){
    moveSto_offline({boardX, boardY + boardStep, penUp, 0});
    moveSto_offline({boardX, boardY + boardStep, penDown, 0});
    moveSto_offline({boardX, boardY - 2*boardStep, penDown, 0});
    moveSto_offline({boardX, boardY - 2*boardStep, penUp, 0});

    moveSto_offline({boardX - boardStep, boardY - 2*boardStep, penUp, 0});
    moveSto_offline({boardX - boardStep, boardY - 2*boardStep, penDown, 0});
    moveSto_offline({boardX - boardStep, boardY + boardStep, penDown, 0});
    moveSto_offline({boardX - boardStep, boardY + boardStep, penUp, 0});

    moveSto_offline({boardX - 2*boardStep, boardY, penUp, 0});
    moveSto_offline({boardX - 2*boardStep, boardY, penDown, 0});
    moveSto_offline({boardX + boardStep, boardY, penDown, 0});
    moveSto_offline({boardX + boardStep, boardY, penUp,0});

    moveSto_offline({boardX + boardStep, boardY - boardStep, penUp, 0});
    moveSto_offline({boardX + boardStep, boardY - boardStep, penDown, 0});
    moveSto_offline({boardX - 2*boardStep, boardY - boardStep, penDown, 0});
    moveSto_offline({boardX - 2*boardStep, boardY - boardStep, penUp, 0});

    defaultPose();
}

void doodbot::draw_X(double i, double j){
    double x = chessX - i*boardStep;
    double y = chessY - j*boardStep;
    double size = 10;
    moveSto_offline({x, y, penUp, 0});
    moveSto_offline({x+size, y+size, penUp, 0});
    moveSto_offline({x+size, y+size, penDown, 0});
    moveSto_offline({x-size, y-size, penDown, 0});
    moveSto_offline({x-size, y-size, penUp, 0});
    moveSto_offline({x-size, y+size, penUp, 0});
    moveSto_offline({x-size, y+size, penDown, 0});
    moveSto_offline({x+size, y-size, penDown, 0});
    moveSto_offline({x+size, y-size, penUp, 0});

    defaultPose();
}

void doodbot::draw_O(double i, double j){
    double x = chessX - i*boardStep;
    double y = chessY - j*boardStep;
    double size = 10;
    moveSto_offline({x, y, penUp, 0});
    moveSto_offline({x, y+size, penUp, 0});
    moveSto_offline({x, y+size, penDown, 0});
    moveC_offline(x, y);
    moveSto_offline({x, y+size, penUp, 0});

    defaultPose();
}


void doodbot::log_pose(){
    pose = dobot->Get_Pose();
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


void doodbot::moveSto_offline(DM destination){
    pose = dobot->Get_Pose();
    path = {1, 0, 0, 0, 0};

    initialState.state = {pose.x, pose.y, pose.z, pose.r};
    finalState.state = destination;
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());
}

void doodbot::moveC_offline(double circleX, double circleY){
    pose = dobot->Get_Pose();
    double circleR = sqrt(pow(pose.x - circleX, 2) + pow(pose.y - circleY, 2));
    path = {0, 1, circleX, circleY, circleR};

    initialState.state = {pose.x, pose.y, pose.z, pose.r};
    finalState.state = {circleX + (circleY - pose.y), circleY - (circleX - pose.x), pose.z, pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());


    initialState.state = {circleX + (circleY - pose.y), circleY - (circleX - pose.x), pose.z, pose.r};
    finalState.state = {circleX + (circleX - pose.x), circleY + (circleY - pose.y), pose.z, pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());


    initialState.state = {circleX + (circleX - pose.x), circleY + (circleY - pose.y), pose.z, pose.r};
    finalState.state = {circleX - (circleY - pose.y), circleY + (circleX - pose.x), pose.z, pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());


    initialState.state = {circleX - (circleY - pose.y), circleY + (circleX - pose.x), pose.z, pose.r};
    finalState.state = {pose.x, pose.y, pose.z, pose.r};
    AEKFq.state = DM::zeros(4);

    ok = solver->solveColloc(initialState, finalState, AEKFq, path);
    if(ok) solver->getSolutionColloc(sol_state, sol_control);
    for(int i = 0; i <  sol_state.size2(); ++i){
        dobot->Send_CP_Cmd(sol_state(0,i).scalar(), sol_state(1,i).scalar(), sol_state(2,i).scalar(), sol_state(3,i).scalar());
    }
    // ros::Duration(2).sleep();
    while(moving());
}

void doodbot::defaultPose(){
    moveSto_offline({75, 0, 0, 0});
}

bool doodbot::moving(){
    Pose pose0 = dobot->Get_Pose();
    ros::Duration(0.1).sleep();
    Pose pose1 = dobot->Get_Pose();
    return distance(pose0, pose1) > 0.0001;
}

int doodbot::gameOver(){
    bool full = true;
    for(int i = 0; i < 3; i++){
        for(int j = 0;j < 3;j++){
            if(board[i][0]==EMPTY){
                full = false;
            }
        }
    }
    if(full)return -1;

	for(int i=0;i<3;i++)
	{
		if(board[i][0]==O && board[i][1]==O && board[i][2]==O) return O;
		if(board[i][0]==X && board[i][1]==X && board[i][2]==X) return X;
	}

	for(int i=0;i<3;i++)
	{
		if(board[0][i]==O && board[1][i]==O && board[2][i]==O) return O;
		if(board[0][i]==X && board[1][i]==X && board[2][i]==X) return X;
	}

	if((board[0][0]==O&&board[1][1]==O&&board[2][2]==O)||(board[2][0]==O&&board[1][1]==O&&board[0][2]==O)) return O;
    if((board[0][0]==X&&board[1][1]==X&&board[2][2]==X)||(board[2][0]==X&&board[1][1]==X&&board[0][2]==X)) return X;

	return 0;
}

void doodbot::callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
    // ROS_INFO("callback");
    for(int i = 0; i < 3; i++){
        for(int j = 0;j < 3;j++){
            int t = msg->data.at(i*3+j);
            if(t!=NOBOARD){
                if(board[i][j] == NOBOARD || board[i][j] == EMPTY){
                    board[i][j] = msg->data.at(i*3+j);
                }
            }
        }
    }
}

double doodbot::distance(Pose pose0, Pose pose1){
    return sqrt(pow(pose0.x-pose1.x,2) + pow(pose0.y-pose1.y,2) + pow(pose0.z-pose1.z, 2) + pow(pose0.r - pose1.r, 2));
}