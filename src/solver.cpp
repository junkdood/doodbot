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
}

Function DirectCollocationSolver::getSystemDynamics(){
    return systemDynamics;
}
void DirectCollocationSolver::setOptColloc(){
    
}
bool DirectCollocationSolver::setupProblemColloc(const Settings& _settings){

    return true;
}
bool DirectCollocationSolver::solveColloc(const State& initialState, const State& finalState){
    return true;
}
void DirectCollocationSolver::getSolutionColloc(DM& state, DM& control){

}
