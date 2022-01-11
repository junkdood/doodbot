#pragma once

#include <casadi/casadi.hpp>

typedef struct{
    double control;
}costWeights;

typedef struct{
    int phaseLength;
    double time;
    costWeights _costWeights;
    double solverVerbosity;
    /*
    ipopt's available linear solvers = {"ma27", "ma57", "ma77", "ma86", "ma97", "pardiso", "wsmp", "mumps"};
    used for it's interior point method
    */
    std::string ipoptLinearSolver;
}Settings;

typedef struct {
    casadi::DM state = casadi::DM::zeros(4,1);
}State;

typedef struct{
    double l0;
    double l1;
    double l2;
}arm_model;

typedef struct{
    double j0_min;
    double j0_max;
    double j1_min;
    double j1_max;
    double j2_min;
    double j2_max;
    double j1_sub_j2_min;
    double j1_sub_j2_max;
    double v_min;
    double v_max;
}constraint_value;