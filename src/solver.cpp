#include "dobot/solver.h"

using S1 = casadi::Slice;

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

    A(0) = V(0);//too hard
    A(1) = V(1);
    A(2) = V(2);
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

    opti.subject_to(X(S1(), 0) == initialStateParameters);
    opti.subject_to(X(S1(), N) == finalStateParameters);

    MX dT = T / phaseLength;
    MX f_curr, f_mid, f_next;
    MX costFunction = 0;
    costWeights w = settings._costWeights;
    for(casadi_int k = 0; k < N; ++k){
        if(k % 2 == 0 && k + 2 <= N){
            f_curr = MX::vertcat(systemDynamics({X(S1(), k), V(S1(), k), 0}));
            f_next = MX::vertcat(systemDynamics({X(S1(), k + 2), V(S1(), k + 2), 0}));
            f_mid = MX::vertcat(systemDynamics({X(S1(), k + 1), V(S1(), k + 1), 0}));
            opti.subject_to(X(S1(), k + 2) == X(S1(), k) + dT * (f_curr + 4 * f_mid + f_next) / 6);
            
            costFunction += w.control * dT / 6 * ( ((pow(V(0,k),2)+pow(V(1,k),2)+pow(V(2,k),2)+pow(V(3,k),2))/4) + 4 * ((pow(V(0,k+1),2)+pow(V(1,k+1),2)+pow(V(2,k+1),2)+pow(V(3,k+1),2))/4) + ((pow(V(0,k+2),2)+pow(V(1,k+2),2)+pow(V(2,k+2),2)+pow(V(3,k+2),2))/4));
        }
        else if(k % 2 != 0){
            opti.subject_to(X(S1(), k) == 0.5 * (X(S1(), k - 1) + X(S1(), k + 1)) + dT * (f_curr - f_next) / 8);
            opti.subject_to(V(S1(), k) == 0.5 * (V(S1(), k - 1) + V(S1(), k + 1)));

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

void DirectCollocationSolver::setParametersValue(const State& initialState, const State& finalState){
    opti.set_value(initialStateParameters, initialState.state);
    opti.set_value(finalStateParameters, finalState.state);
}

bool DirectCollocationSolver::solveColloc(const State& initialState, const State& finalState){
    if(solverState == SolverState::NOT_INITIALIZED){
        throw std::runtime_error("problem not initialized");
        return false;
    }
    setParametersValue(initialState, finalState);
    casadi_int npoints = 2 * static_cast<casadi_int> (settings.phaseLength);
    DM initPos = DM::zeros(4,1);
    DM finalPos = DM::zeros(4,1);
    initPos = initialState.state(S1());
    finalPos = finalState.state(S1());
    DM interpolatedPosition(4, 1);
    DM linSpacePoints = DM::linspace(0, 1, npoints + 1);
    for(casadi_int k = 0; k < npoints + 1; ++k){
        /*
        initial guess, pos using linear interpolatation from init to final, 
        vel and control all set zero
        */
        interpolatedPosition = initPos + linSpacePoints(k) * (finalPos - initPos);
        opti.set_initial(X(S1(), k), interpolatedPosition);
    }
    for(casadi_int k = 0; k < npoints; ++k){
        opti.set_initial(V(S1(), k), 0);
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
            //std::cout <<"i\n" << i << "\n"<<control_all(S1(), i) << "\n\n";
            state(S1(),a) = DM::vertcat({state_all(S1(), i)});
            control(S1(),a) = control_all(S1(), i);
            a++;
        }
    }
}
