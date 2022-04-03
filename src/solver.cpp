#include "doodbot/solver.h"

DirectCollocationSolver::DirectCollocationSolver(const arm_model& _model, const constraint_value& _constraint){
    model = _model;
    solverState = SolverState::NOT_INITIALIZED;
    solution = nullptr;
    minJ0 = opti.parameter();
    maxJ0 = opti.parameter();
    minJ1 = opti.parameter();
    maxJ1 = opti.parameter();
    minJ2 = opti.parameter();
    maxJ2 = opti.parameter();
    minJ1subJ2 = opti.parameter();
    maxJ1subJ2 = opti.parameter();
    minV = opti.parameter();
    maxV = opti.parameter();

    T = opti.parameter();

    initialStateParameters = opti.parameter(4);
    finalStateParameters = opti.parameter(4);
    AEKFqParameters = opti.parameter(4);

    straightWParameters = opti.parameter();
    circleWParameters = opti.parameter();
    circleXParameters = opti.parameter();
    circleYParameters = opti.parameter();
    circleRParameters = opti.parameter();
    

    opti.set_value(minJ0, _constraint.j0_min);
    opti.set_value(maxJ0, _constraint.j0_max);
    opti.set_value(minJ1, _constraint.j1_min);
    opti.set_value(maxJ1, _constraint.j1_max);
    opti.set_value(minJ2, _constraint.j2_min);
    opti.set_value(maxJ2, _constraint.j2_max);
    opti.set_value(minJ1subJ2, _constraint.j1_sub_j2_min);
    opti.set_value(maxJ1subJ2, _constraint.j1_sub_j2_max);
    opti.set_value(minV, _constraint.v_min);
    opti.set_value(maxV, _constraint.v_max);
}

Function DirectCollocationSolver::getSystemDynamics(){
    MX X = MX::sym("x", 4); //状态
    MX V = MX::sym("v", 4); //控制量
    MX dt = MX::sym("dt");
    MX A(4, 1); //状态的导数(就是速度)

    MX J(4, 1);
    J(0) = asin(X(1)/sqrt(pow(X(0),2) + pow(X(1),2)));
    J(1) = acos(X(2)/sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2))) - acos(((pow(X(0),2) + pow(X(1),2) + pow(X(2),2)) + pow(model.l1,2) - pow(model.l2,2))/(2*model.l1*sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2))));
    J(2) = acos(((pow(X(0),2) + pow(X(1),2) + pow(X(2),2)) + pow(model.l2,2) - pow(model.l1,2))/(2*model.l2*sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2)))) - asin(X(2)/sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2)));
    J(3) = X(3);

    A(0) = -X(1)*V(0) + model.l1*cos(J(0))*cos(J(1))*V(1) - model.l2*cos(J(0))*sin(J(2))*V(2);
    A(1) = X(0)*V(0) + model.l1*sin(J(0))*cos(J(1))*V(1) - model.l2*sin(J(0))*sin(J(2))*V(2);
    A(2) = -model.l1*sin(J(1))*V(1) - model.l2*cos(J(2))*V(2);
    A(3) = V(3);

    return Function("dynamics", {X, V, dt}, {A});
}
void DirectCollocationSolver::setOptColloc(){
    casadi_int phaseLength = static_cast<casadi_int> (settings.phaseLength);
    opti.set_value(T, settings.time);
    casadi_int N = 2 * phaseLength;
    X = opti.variable(4, N + 1);
    A = opti.variable(4, N + 1);
    V = opti.variable(4, N + 1);

    opti.subject_to(X(Slice(), 0) == initialStateParameters);
    opti.subject_to(X(Slice(), N) == finalStateParameters);

    MX dT = T / phaseLength;
    MX f_curr, f_mid, f_next;
    MX costFunction = 0;
    costWeights w = settings._costWeights;
    for(casadi_int k = 0; k < N; ++k){
        if(k % 2 == 0 && k + 2 <= N){
            f_curr = MX::vertcat(systemDynamics({X(Slice(), k), V(Slice(), k), dT})) + AEKFqParameters;
            f_next = MX::vertcat(systemDynamics({X(Slice(), k + 2), V(Slice(), k + 2), dT})) + AEKFqParameters;
            f_mid = MX::vertcat(systemDynamics({X(Slice(), k + 1), V(Slice(), k + 1), dT})) + AEKFqParameters;
            opti.subject_to(X(Slice(), k + 2) == X(Slice(), k) + dT * (f_curr + 4 * f_mid + f_next) / 6);
            
            //尽量走直线
            // costFunction += w.path * straightWParameters * mtimes((X(Slice(),k+2)-X(Slice(),k)).T(), (X(Slice(),k+2)-X(Slice(),k)));
            costFunction += w.path * straightWParameters * mtimes((X(Slice(),N)-X(Slice(),k)).T(), (X(Slice(),N)-X(Slice(),k)));

            //走圆的时候另外两个自由度尽量不变
            costFunction += w.path * circleWParameters * mtimes((X(Slice(2,4),k+2)-X(Slice(2,4),k)).T(), (X(Slice(2,4),k+2)-X(Slice(2,4),k)));
            costFunction += w.path * circleWParameters * pow(sqrt(pow(X(0,k) - circleXParameters, 2) + pow(X(1,k) - circleYParameters, 2)) - circleRParameters, 2);

            //控制量尽可能平均
            costFunction += w.control * dT / 6 * ( (mtimes(V(Slice(),k).T(),V(Slice(),k))/4) + 4 * (mtimes(V(Slice(),k+1).T(),V(Slice(),k+1))/4) + (mtimes(V(Slice(),k+2).T(),V(Slice(),k+2))/4));
        }
        else if(k % 2 != 0){
            opti.subject_to(X(Slice(), k) == 0.5 * (X(Slice(), k - 1) + X(Slice(), k + 1)) + dT * (f_curr - f_next) / 8);
            opti.subject_to(V(Slice(), k) == 0.5 * (V(Slice(), k - 1) + V(Slice(), k + 1)));

        }
        opti.subject_to(minJ0 <= asin(X(1, k)/sqrt(pow(X(0,k),2) + pow(X(1,k),2))) <= maxJ0);
        opti.subject_to(minJ1 <= acos(X(2, k)/sqrt(pow(X(0,k),2) + pow(X(1,k),2) + pow(X(2,k),2))) - acos(((pow(X(0,k),2) + pow(X(1,k),2) + pow(X(2,k),2)) + pow(model.l1,2) - pow(model.l2,2))/(2*model.l1*sqrt(pow(X(0,k),2) + pow(X(1,k),2) + pow(X(2,k),2)))) <= maxJ1);
        opti.subject_to(minJ2 <= acos(((pow(X(0,k),2) + pow(X(1,k),2) + pow(X(2,k),2)) + pow(model.l2,2) - pow(model.l1,2))/(2*model.l2*sqrt(pow(X(0,k),2) + pow(X(1,k),2) + pow(X(2,k),2)))) - asin(X(2, k)/sqrt(pow(X(0,k),2) + pow(X(1,k),2) + pow(X(2,k),2))) <= maxJ2);
        opti.subject_to(minV <= V(0, k) <= maxV);
        opti.subject_to(minV <= V(1, k) <= maxV);
        opti.subject_to(minV <= V(2, k) <= maxV);
        opti.subject_to(minV <= V(3, k) <= maxV);
    }   
    opti.subject_to(minJ0 <= asin(X(1, N)/sqrt(pow(X(0,N),2) + pow(X(1,N),2))) <= maxJ0);
    opti.subject_to(minJ1 <= acos(X(2, N)/sqrt(pow(X(0,N),2) + pow(X(1,N),2) + pow(X(2,N),2))) - acos(((pow(X(0,N),2) + pow(X(1,N),2) + pow(X(2,N),2)) + pow(model.l1,2) - pow(model.l2,2))/(2*model.l1*sqrt(pow(X(0,N),2) + pow(X(1,N),2) + pow(X(2,N),2)))) <= maxJ1);
    opti.subject_to(minJ2 <= acos(((pow(X(0,N),2) + pow(X(1,N),2) + pow(X(2,N),2)) + pow(model.l2,2) - pow(model.l1,2))/(2*model.l2*sqrt(pow(X(0,N),2) + pow(X(1,N),2) + pow(X(2,N),2)))) - asin(X(2, N)/sqrt(pow(X(0,N),2) + pow(X(1,N),2) + pow(X(2,N),2))) <= maxJ2);
    opti.subject_to(minV <= V(0, N) <= maxV);
    opti.subject_to(minV <= V(1, N) <= maxV);
    opti.subject_to(minV <= V(2, N) <= maxV);
    opti.subject_to(minV <= V(3, N) <= maxV);
    opti.minimize(costFunction);
}
bool DirectCollocationSolver::setupProblemColloc(const Settings& _settings){
    settings = _settings;
    systemDynamics = getSystemDynamics();

    setOptColloc();

    casadi::Dict casadiOptions;
    casadi::Dict ipoptOptions;

    casadiOptions["expand"] = true;//Replace MX with SX expressions in problem formulation, speed up
    unsigned long solverVerbosity = settings.solverVerbosity;
    if (solverVerbosity) {
        casadi_int ipoptVerbosity = static_cast<long long>(solverVerbosity - 1);
        ipoptOptions["print_level"] = ipoptVerbosity;
        casadiOptions["print_time"] = true;
        casadiOptions["bound_consistency"] = false; 
    } else {
        ipoptOptions["print_level"] = 0;
        casadiOptions["print_time"] = false;
        //casadiOptions["bound_consistency"] = false;
        //ipoptOptions["fixed_variable_treatment"] = "make_constraint";
    }
    ipoptOptions["linear_solver"] = settings.ipoptLinearSolver;

    opti.solver("ipopt", casadiOptions, ipoptOptions);

    solverState = SolverState::PROBLEM_SET;

    return true;
}

void DirectCollocationSolver::setParametersValue(const State& initialState, const State& finalState, const State& AEKFq, const PathCost& path){
    opti.set_value(initialStateParameters, initialState.state);
    opti.set_value(finalStateParameters, finalState.state);
    opti.set_value(AEKFqParameters, AEKFq.state);

    opti.set_value(straightWParameters, path.straightW);
    opti.set_value(circleWParameters, path.circleW);
    opti.set_value(circleXParameters, path.circleX);
    opti.set_value(circleYParameters, path.circleY);
    opti.set_value(circleRParameters, path.circleR);
}

bool DirectCollocationSolver::solveColloc(const State& initialState, const State& finalState, const State& AEKFq, const PathCost& path){
    if(solverState == SolverState::NOT_INITIALIZED){
        throw std::runtime_error("problem not initialized");
        return false;
    }
    setParametersValue(initialState, finalState, AEKFq, path);
    casadi_int npoints = 2 * static_cast<casadi_int> (settings.phaseLength);
    DM initPos = DM::zeros(4,1);
    DM finalPos = DM::zeros(4,1);
    initPos = initialState.state(Slice());
    finalPos = finalState.state(Slice());
    DM interpolatedPosition(4, 1);
    DM linSpacePoints = DM::linspace(0, 1, npoints + 1);
    for(casadi_int k = 0; k < npoints + 1; ++k){
        /*
        initial guess, pos using linear interpolatation from init to final, 
        vel and control all set zero
        */
        interpolatedPosition = initPos + linSpacePoints(k) * (finalPos - initPos);
        opti.set_initial(X(Slice(), k), interpolatedPosition);
    }
    for(casadi_int k = 0; k < npoints; ++k){
        opti.set_initial(V(Slice(), k), 0);
    }
    
    solverState = SolverState::PROBLEM_SET;
    
    try{
        solution = std::make_unique<casadi::OptiSol>(opti.solve());
    }catch(std::exception &e){
        opti.debug().show_infeasibilities(1e-5);
        std::cerr << "error while solving the optimization" << std::endl;
        std::cerr << "Details:\n " << e.what() << std::endl;
        return false;
    }
    solverState = SolverState::PROBLEM_SOLVED;
    std::cout << "\nsolve success\n\n";
    return true;
}
void DirectCollocationSolver::getSolutionColloc(DM& state, DM& control){
    
    DM state_all = solution -> value(X);
    DM control_all = solution -> value(V);
    int a = 0;
    state = DM::zeros(4, settings.phaseLength + 1);
    control = DM::zeros(4, settings.phaseLength + 1);
    for(int i = 0; i < 2 * settings.phaseLength + 1; ++i){
        if(i % 2 == 0){
            std::cout <<control_all(Slice(), i) << "\n";
            state(Slice(),a) = DM::vertcat({state_all(Slice(), i)});
            control(Slice(),a) = control_all(Slice(), i);
            a++;
        }
    }
}


KalmanFilter::KalmanFilter(const arm_model& _model, double dt){
    model = _model;
    dT = dt;
    reset(DM::zeros(4));
    
}

void KalmanFilter::reset(DM X){
    //模型噪声协方差，自适应后好像没用了
    Q = DM::zeros(4, 4);
    Q(0, 0) = 0.001;
    Q(1, 1) = 0.001;
    Q(2, 2) = 0.001;
    Q(3, 3) = 0.001;

    //观测噪声协方差
    R = DM::zeros(4, 4);
    // R(0, 0) = 0.001;
    // R(1, 1) = 0.001;
    // R(2, 2) = 0.001;
    // R(3, 3) = 0.001;

    //EKF初始置零
    x_cal_pre = X;
    pk_pre = DM::zeros(4, 4);
    pk_p = DM::zeros(4, 4);

    //自适应参数初始化
    b = 0.98;
    t = 0;
    q = DM::zeros(4);
    r = DM::zeros(4);
    X_AEKF = X;
    P_AEKF = DM::zeros(4, 4);
}

DM KalmanFilter::g(DM X, DM V){
    DM J(4, 1);
    DM A(4, 1);

    J(0) = asin(X(1)/sqrt(pow(X(0),2) + pow(X(1),2)));
    J(1) = acos(X(2)/sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2))) - acos(((pow(X(0),2) + pow(X(1),2) + pow(X(2),2)) + pow(model.l1,2) - pow(model.l2,2))/(2*model.l1*sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2))));
    J(2) = acos(((pow(X(0),2) + pow(X(1),2) + pow(X(2),2)) + pow(model.l2,2) - pow(model.l1,2))/(2*model.l2*sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2)))) - asin(X(2)/sqrt(pow(X(0),2) + pow(X(1),2) + pow(X(2),2)));
    J(3) = X(3);

    A(0) = -X(1)*V(0) + model.l1*cos(J(0))*cos(J(1))*V(1) - model.l2*cos(J(0))*sin(J(2))*V(2);
    A(1) = X(0)*V(0) + model.l1*sin(J(0))*cos(J(1))*V(1) - model.l2*sin(J(0))*sin(J(2))*V(2);
    A(2) = -model.l1*sin(J(1))*V(1) - model.l2*cos(J(2))*V(2);
    A(3) = V(3);

    return X + A * dT;
}

DM KalmanFilter::Jg(DM X, DM V){
    double eps = 0.001;
    DM A = DM::zeros(4, 4);

    DM temp = g(X, V);
    DM newX = X;
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            newX = X;
            newX(j)+=eps;
            A(i,j) = (g(newX, V) - temp)(i) / eps;//Xi 对 Xj的偏导
        }
    }
    return A;
}

DM KalmanFilter::h(DM X){
    return X;
}

DM KalmanFilter::Jh(DM X){
    return DM::eye(4);
}

void KalmanFilter::Predict(DM control){
    x_pred = g(x_cal_pre, control);
    DM J_g = Jg(x_cal_pre, control);
    pk_p = mtimes(mtimes(J_g,pk_pre),J_g.T()) + Q;    
}

void KalmanFilter::Update(DM y_meas){
    DM J_h = Jh(x_pred);
    DM tmp = mtimes(mtimes(J_h,pk_p),J_h.T()) + R;
    DM k = mtimes(mtimes(pk_p,J_h.T()),inv(tmp));
    x_cal_pre = x_pred + mtimes(k, (y_meas - h(x_pred)));
    pk_pre = mtimes((DM::eye(4) - mtimes(k, J_h)), pk_p);
} 

DM KalmanFilter::getCal(){
    return h(x_cal_pre);
}

DM KalmanFilter::AEKF_unity(DM control, DM y_meas){
    

    DM X_AEKF_cache = g(X_AEKF, control);
    X_AEKF = X_AEKF_cache + q;
    DM J_g = Jg(X_AEKF, control);
    DM P_AEKF_cache = mtimes(mtimes(J_g,P_AEKF),J_g.T());
    P_AEKF = P_AEKF_cache + Q;

    DM J_h = Jh(X_AEKF);
    DM tmp_cache = mtimes(mtimes(J_h,P_AEKF),J_h.T());
    DM tmp = tmp_cache + R;
    DM k = mtimes(mtimes(P_AEKF,J_h.T()),inv(tmp));
    DM epsilon_cache = y_meas - h(X_AEKF);
    DM epsilon = epsilon_cache - r;
    X_AEKF = X_AEKF + mtimes(k, epsilon);
    P_AEKF = mtimes((DM::eye(4) - mtimes(k, J_h)), P_AEKF);

    //自适应参数更新
    double d = (1.0 - b)/(1.0 - pow(b, t + 1));
    q = (1 - d)*q + d * (X_AEKF - X_AEKF_cache);
    Q = (1 - d)*Q + d * (mtimes(mtimes(mtimes(k, epsilon), epsilon.T()), k.T()) + P_AEKF - P_AEKF_cache);
    // r = (1 - d)*r + d * epsilon_cache;
    // R = (1 - d)*R + d * (mtimes(mtimes(mtimes((DM::eye(4) - mtimes(k, J_h)), epsilon), epsilon.T()), (DM::eye(4) - mtimes(k, J_h)).T()) + tmp_cache);
    t++;
    std::cout<<q.get_str()<<std::endl;

    return h(X_AEKF);
}

DM KalmanFilter::getq(){
    return q;
}