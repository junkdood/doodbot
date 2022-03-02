#pragma once

#include "dobot/common.h"

using casadi::DM;
using casadi::MX;
using casadi::Function;
using casadi::Opti;
using casadi::Slice;

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
    void setParametersValue(const State& initialState, const State& finalState);


    arm_model model;
    Settings settings;

    enum SolverState{NOT_INITIALIZED=0, PROBLEM_SET, PROBLEM_SOLVED};
    SolverState solverState;

    Opti opti;
    std::unique_ptr<casadi::OptiSol> solution;
    MX initialStateParameters, finalStateParameters;
    MX X, A, V, T;
    MX minJ0, maxJ0, minJ1, maxJ1, minJ2, maxJ2, minJ1subJ2, maxJ1subJ2, minV, maxV;
};

class KalmanFilter{
    public:
    KalmanFilter(const arm_model& _model, double dt);
    ~KalmanFilter(){};

    //初始化
    void reset(DM X);

    //KF核心过程
    void Predict(DM control);
    void Update(DM y_meas);

    //获取预测
    DM getCal();

    DM g(DM X, DM V);
    DM Jg(DM X, DM V);
    DM h(DM X);
    DM Jh(DM X);

    private:
    //模型相关
    arm_model model;
    Function systemDynamics;
    Function getSystemDynamics();
    double dT;

    //Kalman相关
    DM Q;
    DM R;
    DM x_cal_pre; //上一个最终预测值
    DM pk_pre; //上一个pk，即估计与真值协方差
    DM pk_p; //预测与真值协方差

    DM x_pred; //systemDynamics估计的值
};