#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "DobotDll.h"

/*
 * Cmd timeout
 */
#include "doodbot/SetCmdTimeout.h"

bool SetCmdTimeoutService(doodbot::SetCmdTimeout::Request &req, doodbot::SetCmdTimeout::Response &res)
{
    res.result = SetCmdTimeout(req.timeout);

    return true;
}

void InitCmdTimeoutServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetCmdTimeout", SetCmdTimeoutService);
    serverVec.push_back(server);
}

/*
 * Device information
 */
#include "doodbot/GetDeviceSN.h"
#include "doodbot/SetDeviceName.h"
#include "doodbot/GetDeviceName.h"
#include "doodbot/GetDeviceVersion.h"

bool GetDeviceSNService(doodbot::GetDeviceSN::Request &req, doodbot::GetDeviceSN::Response &res)
{
    char deviceSN[256];

    res.result = GetDeviceSN(deviceSN, sizeof(deviceSN));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceSN;
        res.deviceSN.data = ss.str();
    }

    return true;
}

bool SetDeviceNameService(doodbot::SetDeviceName::Request &req, doodbot::SetDeviceName::Response &res)
{
    res.result = SetDeviceName(req.deviceName.data.c_str());

    return true;
}

bool GetDeviceNameService(doodbot::GetDeviceName::Request &req, doodbot::GetDeviceName::Response &res)
{
    char deviceName[256];

    res.result = GetDeviceName(deviceName, sizeof(deviceName));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceName;
        res.deviceName.data = ss.str();
    }

    return true;
}

bool GetDeviceVersionService(doodbot::GetDeviceVersion::Request &req, doodbot::GetDeviceVersion::Response &res)
{
    uint8_t majorVersion, minorVersion, revision;

    res.result = GetDeviceVersion(&majorVersion, &minorVersion, &revision);
    if (res.result == DobotCommunicate_NoError) {
        res.majorVersion = majorVersion;
        res.minorVersion = minorVersion;
        res.revision = revision;
    }

    return true;
}

void InitDeviceInfoServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/GetDeviceSN", GetDeviceSNService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetDeviceName", SetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetDeviceName", GetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetDeviceVersion", GetDeviceVersionService);
    serverVec.push_back(server);
}

/*
 * Pose
 */
#include "doodbot/GetPose.h"

bool GetPoseService(doodbot::GetPose::Request &req, doodbot::GetPose::Response &res)
{
    Pose pose;

    res.result = GetPose(&pose);
    if (res.result == DobotCommunicate_NoError) {
        res.x = pose.x;
        res.y = pose.y;
        res.z = pose.z;
        res.r = pose.r;
        for (int i = 0; i < 4; i++) {
            res.jointAngle.push_back(pose.jointAngle[i]);
        }
    }

    return true;
}

void InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/GetPose", GetPoseService);
    serverVec.push_back(server);
}

/*
 * Alarms
 */
#include "doodbot/GetAlarmsState.h"
#include "doodbot/ClearAllAlarmsState.h"

bool GetAlarmsStateService(doodbot::GetAlarmsState::Request &req, doodbot::GetAlarmsState::Response &res)
{
    uint8_t alarmsState[128];
    uint32_t len;

    res.result = GetAlarmsState(alarmsState, &len, sizeof(alarmsState));
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < len; i++) {
            res.alarmsState.push_back(alarmsState[i]);
        }
    }

    return true;
}

bool ClearAllAlarmsStateService(doodbot::ClearAllAlarmsState::Request &req, doodbot::ClearAllAlarmsState::Response &res)
{
    res.result = ClearAllAlarmsState();

    return true;
}

void InitAlarmsServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/GetAlarmsState", GetAlarmsStateService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/ClearAllAlarmsState", ClearAllAlarmsStateService);
    serverVec.push_back(server);
}

/*
 * HOME
 */
#include "doodbot/SetHOMEParams.h"
#include "doodbot/GetHOMEParams.h"
#include "doodbot/SetHOMECmd.h"

bool SetHOMEParamsService(doodbot::SetHOMEParams::Request &req, doodbot::SetHOMEParams::Response &res)
{
    HOMEParams params;
    uint64_t queuedCmdIndex;

    params.x = req.x;
    params.y = req.y;
    params.z = req.z;
    params.r = req.r;

    res.result = SetHOMEParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetHOMEParamsService(doodbot::GetHOMEParams::Request &req, doodbot::GetHOMEParams::Response &res)
{
    HOMEParams params;

    res.result = GetHOMEParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.x = params.x;
        res.y = params.y;
        res.z = params.z;
        res.r = params.r;
    }

    return true;
}

bool SetHOMECmdService(doodbot::SetHOMECmd::Request &req, doodbot::SetHOMECmd::Response &res)
{
    HOMECmd cmd;
    uint64_t queuedCmdIndex;

    res.result = SetHOMECmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitHOMEServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetHOMEParams", SetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetHOMEParams", GetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetHOMECmd", SetHOMECmdService);
    serverVec.push_back(server);
}

/*
 * End effector
 */
#include "doodbot/SetEndEffectorParams.h"
#include "doodbot/GetEndEffectorParams.h"
#include "doodbot/SetEndEffectorLaser.h"
#include "doodbot/GetEndEffectorLaser.h"
#include "doodbot/SetEndEffectorSuctionCup.h"
#include "doodbot/GetEndEffectorSuctionCup.h"
#include "doodbot/SetEndEffectorGripper.h"
#include "doodbot/GetEndEffectorGripper.h"

bool SetEndEffectorParamsService(doodbot::SetEndEffectorParams::Request &req, doodbot::SetEndEffectorParams::Response &res)
{
    EndEffectorParams params;
    uint64_t queuedCmdIndex;

    params.xBias = req.xBias;
    params.yBias = req.yBias;
    params.zBias = req.zBias;

    res.result = SetEndEffectorParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorParamsService(doodbot::GetEndEffectorParams::Request &req, doodbot::GetEndEffectorParams::Response &res)
{
    EndEffectorParams params;

    res.result = GetEndEffectorParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xBias = params.xBias;
        res.yBias = params.yBias;
        res.zBias = params.zBias;
    }

    return true;
}

bool SetEndEffectorLaserService(doodbot::SetEndEffectorLaser::Request &req, doodbot::SetEndEffectorLaser::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorLaser(req.enableCtrl, req.on, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorLaserService(doodbot::GetEndEffectorLaser::Request &req, doodbot::GetEndEffectorLaser::Response &res)
{
    bool enableCtrl, on;

    res.result = GetEndEffectorLaser(&enableCtrl, &on);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.on = on;
    }

    return true;
}

bool SetEndEffectorSuctionCupService(doodbot::SetEndEffectorSuctionCup::Request &req, doodbot::SetEndEffectorSuctionCup::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorSuctionCup(req.enableCtrl, req.suck, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorSuctionCupService(doodbot::GetEndEffectorSuctionCup::Request &req, doodbot::GetEndEffectorSuctionCup::Response &res)
{
    bool enableCtrl, suck;

    res.result = GetEndEffectorLaser(&enableCtrl, &suck);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.suck = suck;
    }

    return true;
}

bool SetEndEffectorGripperService(doodbot::SetEndEffectorGripper::Request &req, doodbot::SetEndEffectorGripper::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorGripper(req.enableCtrl, req.grip, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorGripperService(doodbot::GetEndEffectorGripper::Request &req, doodbot::GetEndEffectorGripper::Response &res)
{
    bool enableCtrl, grip;

    res.result = GetEndEffectorLaser(&enableCtrl, &grip);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.grip = grip;
    }

    return true;
}

void InitEndEffectorServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetEndEffectorParams", SetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorParams", GetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEndEffectorLaser", SetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorLaser", GetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEndEffectorSuctionCup", SetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorSuctionCup", GetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEndEffectorGripper", SetEndEffectorGripperService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetEndEffectorGripper", GetEndEffectorGripperService);
    serverVec.push_back(server);
}

/*
 * JOG
 */
#include "doodbot/SetJOGJointParams.h"
#include "doodbot/GetJOGJointParams.h"
#include "doodbot/SetJOGCoordinateParams.h"
#include "doodbot/GetJOGCoordinateParams.h"
#include "doodbot/SetJOGCommonParams.h"
#include "doodbot/GetJOGCommonParams.h"
#include "doodbot/SetJOGCmd.h"

bool SetJOGJointParamsService(doodbot::SetJOGJointParams::Request &req, doodbot::SetJOGJointParams::Response &res)
{
    JOGJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGJointParamsService(doodbot::GetJOGJointParams::Request &req, doodbot::GetJOGJointParams::Response &res)
{
    JOGJointParams params;

    res.result = GetJOGJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCoordinateParamsService(doodbot::SetJOGCoordinateParams::Request &req, doodbot::SetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCoordinateParamsService(doodbot::GetJOGCoordinateParams::Request &req, doodbot::GetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;

    res.result = GetJOGCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCommonParamsService(doodbot::SetJOGCommonParams::Request &req, doodbot::SetJOGCommonParams::Response &res)
{
    JOGCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetJOGCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCommonParamsService(doodbot::GetJOGCommonParams::Request &req, doodbot::GetJOGCommonParams::Response &res)
{
    JOGCommonParams params;

    res.result = GetJOGCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool SetJOGCmdService(doodbot::SetJOGCmd::Request &req, doodbot::SetJOGCmd::Response &res)
{
    JOGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.isJoint = req.isJoint;
    cmd.cmd = req.cmd;
    res.result = SetJOGCmd(&cmd, false, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitJOGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetJOGJointParams", SetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetJOGJointParams", GetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetJOGCoordinateParams", SetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetJOGCoordinateParams", GetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetJOGCommonParams", SetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetJOGCommonParams", GetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetJOGCmd", SetJOGCmdService);
    serverVec.push_back(server);
}

/*
 * PTP
 */
#include "doodbot/SetPTPJointParams.h"
#include "doodbot/GetPTPJointParams.h"
#include "doodbot/SetPTPCoordinateParams.h"
#include "doodbot/GetPTPCoordinateParams.h"
#include "doodbot/SetPTPJumpParams.h"
#include "doodbot/GetPTPJumpParams.h"
#include "doodbot/SetPTPCommonParams.h"
#include "doodbot/GetPTPCommonParams.h"
#include "doodbot/SetPTPCmd.h"

bool SetPTPJointParamsService(doodbot::SetPTPJointParams::Request &req, doodbot::SetPTPJointParams::Response &res)
{
    PTPJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetPTPJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPJointParamsService(doodbot::GetPTPJointParams::Request &req, doodbot::GetPTPJointParams::Response &res)
{
    PTPJointParams params;

    res.result = GetPTPJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetPTPCoordinateParamsService(doodbot::SetPTPCoordinateParams::Request &req, doodbot::SetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetPTPCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPCoordinateParamsService(doodbot::GetPTPCoordinateParams::Request &req, doodbot::GetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;

    res.result = GetPTPCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool SetPTPJumpParamsService(doodbot::SetPTPJumpParams::Request &req, doodbot::SetPTPJumpParams::Response &res)
{
    PTPJumpParams params;
    uint64_t queuedCmdIndex;

    params.jumpHeight = req.jumpHeight;
    params.zLimit = req.zLimit;
    res.result = SetPTPJumpParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPJumpParamsService(doodbot::GetPTPJumpParams::Request &req, doodbot::GetPTPJumpParams::Response &res)
{
    PTPJumpParams params;

    res.result = GetPTPJumpParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.jumpHeight = params.jumpHeight;
        res.zLimit = params.zLimit;
    }

    return true;
}

bool SetPTPCommonParamsService(doodbot::SetPTPCommonParams::Request &req, doodbot::SetPTPCommonParams::Response &res)
{
    PTPCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetPTPCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPCommonParamsService(doodbot::GetPTPCommonParams::Request &req, doodbot::GetPTPCommonParams::Response &res)
{
    PTPCommonParams params;

    res.result = GetPTPCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool SetPTPCmdService(doodbot::SetPTPCmd::Request &req, doodbot::SetPTPCmd::Response &res)
{
    PTPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = req.ptpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.r = req.r;
    res.result = SetPTPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitPTPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetPTPJointParams", SetPTPJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPJointParams", GetPTPJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPCoordinateParams", SetPTPCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPCoordinateParams", GetPTPCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPJumpParams", SetPTPJumpParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPJumpParams", GetPTPJumpParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPCommonParams", SetPTPCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetPTPCommonParams", GetPTPCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetPTPCmd", SetPTPCmdService);
    serverVec.push_back(server);
}

/*
 * CP
 */
#include "doodbot/SetCPParams.h"
#include "doodbot/GetCPParams.h"
#include "doodbot/SetCPCmd.h"

bool SetCPParamsService(doodbot::SetCPParams::Request &req, doodbot::SetCPParams::Response &res)
{
    CPParams params;
    uint64_t queuedCmdIndex;

    params.planAcc = req.planAcc;
    params.juncitionVel = req.junctionVel;
    params.acc = req.acc;
    params.realTimeTrack = req.realTimeTrack;
    res.result = SetCPParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetCPParamsService(doodbot::GetCPParams::Request &req, doodbot::GetCPParams::Response &res)
{
    CPParams params;

    res.result = GetCPParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.planAcc = params.planAcc;
        res.junctionVel = params.juncitionVel;
        res.acc = params.acc;
        res.realTimeTrack = params.realTimeTrack;
    }

    return true;
}

bool SetCPCmdService(doodbot::SetCPCmd::Request &req, doodbot::SetCPCmd::Response &res)
{
    CPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cpMode = req.cpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.velocity = req.velocity;

    res.result = SetCPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitCPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetCPParams", SetCPParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetCPParams", GetCPParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetCPCmd", SetCPCmdService);
    serverVec.push_back(server);
}

/*
 * ARC
 */
#include "doodbot/SetARCParams.h"
#include "doodbot/GetARCParams.h"
#include "doodbot/SetARCCmd.h"

bool SetARCParamsService(doodbot::SetARCParams::Request &req, doodbot::SetARCParams::Response &res)
{
    ARCParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetARCParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetARCParamsService(doodbot::GetARCParams::Request &req, doodbot::GetARCParams::Response &res)
{
    ARCParams params;

    res.result = GetARCParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool SetARCCmdService(doodbot::SetARCCmd::Request &req, doodbot::SetARCCmd::Response &res)
{
    ARCCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cirPoint.x = req.x1;
    cmd.cirPoint.y = req.y1;
    cmd.cirPoint.z = req.z1;
    cmd.cirPoint.r = req.r1;
    cmd.toPoint.x = req.x2;
    cmd.toPoint.y = req.y2;
    cmd.toPoint.z = req.z2;
    cmd.toPoint.r = req.r2;

    res.result = SetARCCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitARCServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetARCParams", SetARCParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetARCParams", GetARCParamsService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetARCCmd", SetARCCmdService);
    serverVec.push_back(server);
}

/*
 * WAIT
 */
#include "doodbot/SetWAITCmd.h"

bool SetWAITCmdService(doodbot::SetWAITCmd::Request &req, doodbot::SetWAITCmd::Response &res)
{
    WAITCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.timeout = req.timeout;
    res.result = SetWAITCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitWAITServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetWAITCmd", SetWAITCmdService);
    serverVec.push_back(server);
}

/*
 * TRIG
 */
#include "doodbot/SetTRIGCmd.h"

bool SetTRIGCmdService(doodbot::SetTRIGCmd::Request &req, doodbot::SetTRIGCmd::Response &res)
{
    TRIGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.address = req.address;
    cmd.mode = req.mode;
    cmd.condition = req.condition;
    cmd.threshold = req.threshold;
    res.result = SetTRIGCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitTRIGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetTRIGCmd", SetTRIGCmdService);
    serverVec.push_back(server);
}

/*
 * EIO
 */
#include "doodbot/SetIOMultiplexing.h"
#include "doodbot/GetIOMultiplexing.h"
#include "doodbot/SetIODO.h"
#include "doodbot/GetIODO.h"
#include "doodbot/SetIOPWM.h"
#include "doodbot/GetIOPWM.h"
#include "doodbot/GetIODI.h"
#include "doodbot/GetIOADC.h"
#include "doodbot/SetEMotor.h"
#include "doodbot/SetInfraredSensor.h"
#include "doodbot/GetInfraredSensor.h"
#include "doodbot/SetColorSensor.h"
#include "doodbot/GetColorSensor.h"

bool SetIOMultiplexingService(doodbot::SetIOMultiplexing::Request &req, doodbot::SetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;
    uint64_t queuedCmdIndex;

    ioMultiplexing.address = req.address;
    ioMultiplexing.multiplex = req.multiplex;
    res.result = SetIOMultiplexing(&ioMultiplexing, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIOMultiplexingService(doodbot::GetIOMultiplexing::Request &req, doodbot::GetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;

    ioMultiplexing.address = req.address;
    res.result = GetIOMultiplexing(&ioMultiplexing);
    if (res.result == DobotCommunicate_NoError) {
        res.multiplex = ioMultiplexing.multiplex;
    }

    return true;
}

bool SetIODOService(doodbot::SetIODO::Request &req, doodbot::SetIODO::Response &res)
{
    IODO ioDO;
    uint64_t queuedCmdIndex;

    ioDO.address = req.address;
    ioDO.level = req.level;
    res.result = SetIODO(&ioDO, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIODOService(doodbot::GetIODO::Request &req, doodbot::GetIODO::Response &res)
{
    IODO ioDO;

    ioDO.address = req.address;
    res.result = GetIODO(&ioDO);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDO.level;
    }

    return true;
}

bool SetIOPWMService(doodbot::SetIOPWM::Request &req, doodbot::SetIOPWM::Response &res)
{
    IOPWM ioPWM;
    uint64_t queuedCmdIndex;

    ioPWM.address = req.address;
    ioPWM.frequency = req.frequency;
    ioPWM.dutyCycle = req.dutyCycle;
    res.result = SetIOPWM(&ioPWM, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIOPWMService(doodbot::GetIOPWM::Request &req, doodbot::GetIOPWM::Response &res)
{
    IOPWM ioPWM;

    ioPWM.address = req.address;
    res.result = GetIOPWM(&ioPWM);
    if (res.result == DobotCommunicate_NoError) {
        res.frequency = ioPWM.frequency;
        res.dutyCycle = ioPWM.dutyCycle;
    }

    return true;
}

bool GetIODIService(doodbot::GetIODI::Request &req, doodbot::GetIODI::Response &res)
{
    IODI ioDI;

    ioDI.address = req.address;
    res.result = GetIODI(&ioDI);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDI.level;
    }

    return true;
}

bool GetIOADCService(doodbot::GetIOADC::Request &req, doodbot::GetIOADC::Response &res)
{
    IOADC ioADC;

    ioADC.address = req.address;
    res.result = GetIOADC(&ioADC);
    if (res.result == DobotCommunicate_NoError) {
        res.value = ioADC.value;
    }

    return true;
}

bool SetEMotorService(doodbot::SetEMotor::Request &req, doodbot::SetEMotor::Response &res)
{
    EMotor eMotor;
    uint64_t queuedCmdIndex;

    eMotor.index = req.index;
    eMotor.isEnabled = req.isEnabled;
    eMotor.speed = req.speed;
    res.result = SetEMotor(&eMotor, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}


bool SetInfraredSensorService(doodbot::SetInfraredSensor::Request &req, doodbot::SetInfraredSensor::Response &res)
{
    InfraredPort infraredPort = InfraredPort(req.infraredPort);
    res.result = SetInfraredSensor(req.enableCtrl, infraredPort);

    return true;
}

bool GetInfraredSensorService(doodbot::GetInfraredSensor::Request &req, doodbot::GetInfraredSensor::Response &res)
{
    uint8_t value;
    InfraredPort infraredPort = InfraredPort(req.infraredPort);
    res.result = GetInfraredSensor(infraredPort, &value);
    if (res.result == DobotCommunicate_NoError) {
        res.value = value;
    }

    return true;
}

bool SetColorSensorService(doodbot::SetColorSensor::Request &req, doodbot::SetColorSensor::Response &res)
{
    ColorPort colorPort = ColorPort(req.colorPort);
    res.result = SetColorSensor(req.enableCtrl, colorPort);

    return true;
}

bool GetColorSensorService(doodbot::GetColorSensor::Request &req, doodbot::GetColorSensor::Response &res)
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    res.result = GetColorSensor(&r, &g, &b);
    if (res.result == DobotCommunicate_NoError) {
        res.r = r;
        res.g = g;
        res.b = b;
    }

    return true;
}

void InitEIOServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetIOMultiplexing", SetIOMultiplexingService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIOMultiplexing", GetIOMultiplexingService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetIODO", SetIODOService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIODO", GetIODOService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetIOPWM", SetIOPWMService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIOPWM", GetIOPWMService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIODI", GetIODIService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetIOADC", GetIOADCService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetEMotor", SetEMotorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetInfraredSensor", SetInfraredSensorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetInfraredSensor", GetInfraredSensorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetColorSensor", SetColorSensorService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/GetColorSensor", GetColorSensorService);
    serverVec.push_back(server);
}

/*
 * Queued command control
 */
#include "doodbot/SetQueuedCmdStartExec.h"
#include "doodbot/SetQueuedCmdStopExec.h"
#include "doodbot/SetQueuedCmdForceStopExec.h"
#include "doodbot/SetQueuedCmdClear.h"

bool SetQueuedCmdStartExecService(doodbot::SetQueuedCmdStartExec::Request &req, doodbot::SetQueuedCmdStartExec::Response &res)
{
    res.result = SetQueuedCmdStartExec();

    return true;
}

bool SetQueuedCmdStopExecService(doodbot::SetQueuedCmdStopExec::Request &req, doodbot::SetQueuedCmdStopExec::Response &res)
{
    res.result = SetQueuedCmdStopExec();

    return true;
}

bool SetQueuedCmdForceStopExecService(doodbot::SetQueuedCmdForceStopExec::Request &req, doodbot::SetQueuedCmdForceStopExec::Response &res)
{
    res.result = SetQueuedCmdForceStopExec();

    return true;
}

bool SetQueuedCmdClearService(doodbot::SetQueuedCmdClear::Request &req, doodbot::SetQueuedCmdClear::Response &res)
{
    res.result = SetQueuedCmdClear();

    return true;
}

void InitQueuedCmdServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService("/DobotServer/SetQueuedCmdStartExec", SetQueuedCmdStartExecService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetQueuedCmdStopExec", SetQueuedCmdStopExecService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetQueuedCmdForceStopExec", SetQueuedCmdForceStopExecService);
    serverVec.push_back(server);
    server = n.advertiseService("/DobotServer/SetQueuedCmdClear", SetQueuedCmdClearService);
    serverVec.push_back(server);
}


#include <stdio.h>


int main(int argc, char **argv)
{
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            getchar();
            return -3;
        break;
        default:
        break;
    }
    ros::init(argc, argv, "DobotServer");
    ros::NodeHandle n;

    std::vector<ros::ServiceServer> serverVec;

    InitCmdTimeoutServices(n, serverVec);
    InitDeviceInfoServices(n, serverVec);
    InitPoseServices(n, serverVec);
    InitAlarmsServices(n, serverVec);
    InitHOMEServices(n, serverVec);
    InitEndEffectorServices(n, serverVec);
    InitJOGServices(n, serverVec);
    InitPTPServices(n, serverVec);
    InitCPServices(n, serverVec);
    InitARCServices(n, serverVec);
    InitWAITServices(n, serverVec);
    InitTRIGServices(n, serverVec);
    InitEIOServices(n, serverVec);
    InitQueuedCmdServices(n, serverVec);

    ROS_INFO("Dobot service running...");
    ros::spin();
    ROS_INFO("Dobot service exiting...");

    // Disconnect Dobot
    DisconnectDobot();

    return 0;
}

