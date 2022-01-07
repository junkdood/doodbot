#pragma once

#include "dobot/common.h"

using casadi::DM;
using casadi::MX;
using casadi::Function;
using casadi::Opti;

class DirectCollocationSolver{
public:
    DirectCollocationSolver(const arm_model& _model, const constraint_value& _constraint);
    ~DirectCollocationSolver(){ };
    bool setupProblemColloc(const Settings& _settings);
    bool solveColloc(const State& initialState, const State& finalState);
    void getSolutionColloc(DM& state, DM& control);
private:
    Function systemDynamics;
    Function getSystemDynamics();
    void setOptColloc();


    arm_model model;
    Settings settings;

    enum SolverState{NOT_INITIALIZED=0, PROBLEM_SET, PROBLEM_SOLVED};
    SolverState solverState;

    casadi::Opti opti;
    std::unique_ptr<casadi::OptiSol> solution;
    casadi::MX initialStateParameters, finalStateParameters;
    casadi::MX X, A, U, T;
    casadi::MX minJ0, maxJ0, minJ1, maxJ1, minJ2, maxJ2, minJ1subJ2, maxJ1subJ2;
};