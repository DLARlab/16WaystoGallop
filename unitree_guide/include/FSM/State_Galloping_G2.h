
/*
 * @file State_Galloping_G2.h
 * @author Unitree Robotics
 * @brief 
 * @version 0.1
 * @date 2023-11-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef GALLOPING_G2_H
#define GALLOPING_G2_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include <fstream>


class State_Galloping_G2 : public FSMState{
public:
    State_Galloping_G2(CtrlComponents *ctrlComp);
    ~State_Galloping_G2();
    void enter(); 
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);

    void plotData(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data);
    void plotSubPlot(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data, int legNum);
    void plotXZ(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data);
    void plotGRF(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data);
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();
    Vec3 xProd(Vec3 vec1, Vec3 vec2);

    GaitGenerator *_gait; // GaitGenerator Class Pointer, gait generator
    Estimator *_est; // Estimator Class Pointer, estimator
    QuadrupedRobot *_robModel; // QuadrupedRobot Class Pointer, robot model    
    // if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
    //     return true;
    // }     // The robot height is smaller than 0.05m
    // else{
    //     return false;
    // }
    BalanceCtrl *_balCtrl; //BalanceCtrl Class Pointer, balance controller

    std::vector<std::vector<double>> _desFoot, _actFoot, _taoAll, _vTorso, _actQ, _desQ, _posTorso, _desPosTorso, accp, accg, _posFHipz, _desPosFHipz, _posRHipz, _desPosRHipz, _desPosXZ, _posXZ, _simGRF, _calGRF;
    std::vector<double> _desFoot0, _desFoot1, _desFoot2, _actFoot0, _actFoot1 , _actFoot2, _desFoot6, _desFoot7, _desFoot8, _actFoot6, _actFoot7 , _actFoot8;
    std::vector<double> _q0, _q1, _q2, _q3, _q4, _q5, _q6, _q7, _q8, _q9, _q10, _q11;

    std::vector<double> _qd0, _qd1, _qd2, _qd3, _qd4, _qd5, _qd6, _qd7, _qd8, _qd9, _qd10, _qd11;
    std::vector<double> _q0_d, _q1_d, _q2_d, _q3_d, _q4_d, _q5_d, _q6_d, _q7_d, _q8_d, _q9_d, _q10_d, _q11_d;
    std::vector<double> _vTorso0, _vTorso1, _vTorso2, _vTorso3, _vTorso4, _vTorso5;
    std::vector<double> _posTorso0, _posTorso1, _posTorso2, _posTorso3, _posTorso4, _posTorso5, _posTorso6, _posTorso7, _posTorso8, _posTorso9, _posTorso10, _posTorso11, _posTorso12, _posTorso13;
    std::vector<double> _desPosTorso0, _desPosTorso1, _desPosTorso2, _desPosTorso3, _desPosTorso4, _desPosTorso5, _desPosTorso6, _desPosTorso7, _desPosTorso8, _desPosTorso9, _desPosTorso10;
    std::vector<double> _tao0, _tao1, _tao2, _tao3, _tao4, _tao5, _tao6, _tao7, _tao8, _tao9, _tao10, _tao11;
    std::vector<double> _accp0, _accp1, _accp2, _accp3, _accp4, _accp5, _accp6, _accp7, _accp8, _accp9, _accp10, _accp11;
    std::vector<double> _accg0, _accg1, _accg2, _accg3, _accg4, _accg5, _accg6, _accg7, _accg8, _accg9, _accg10, _accg11;
    std::vector<double> _forceFeetGlobal0, _forceFeetGlobal1, _forceFeetGlobal2, _forceFeetGlobal3, _forceFeetGlobal4, _forceFeetGlobal5, _forceFeetGlobal6, _forceFeetGlobal7, _forceFeetGlobal8, _forceFeetGlobal9, _forceFeetGlobal10, _forceFeetGlobal11;
    std::vector<double> _simGRF0, _simGRF1, _simGRF2, _simGRF3, _simGRF4, _simGRF5, _simGRF6, _simGRF7, _simGRF8, _simGRF9, _simGRF10, _simGRF11, _simGRF12, _simGRF13;
    
    std::ofstream file_q0, file_q1, file_q2, file_q3, file_q4, file_q5, file_q6, file_q7, file_q8, file_q9, file_q10, file_q11;
    std::ofstream file_dq0, file_dq1, file_dq2, file_dq3, file_dq4, file_dq5, file_dq6, file_dq7, file_dq8, file_dq9, file_dq10, file_dq11;
    std::ofstream file_tauM0, file_tauM1, file_tauM2, file_tauM3, file_tauM4, file_tauM5, file_tauM6, file_tauM7, file_tauM8, file_tauM9, file_tauM10, file_tauM11;

    std::ofstream file_qd0, file_qd1, file_qd2, file_qd3, file_qd4, file_qd5, file_qd6, file_qd7, file_qd8, file_qd9, file_qd10, file_qd11;
    std::ofstream file_posTorso0, file_posTorso1, file_posTorso2, file_posTorso3, file_posTorso4, file_posTorso5, file_posTorso6, file_posTorso7, file_posTorso8, file_posTorso9, file_posTorso10, file_posTorso11, file_posTorso12, file_posTorso13;
    std::ofstream file_desPosTorso0, file_desPosTorso1, file_desPosTorso2, file_desPosTorso3, file_desPosTorso4, file_desPosTorso5, file_desPosTorso6, file_desPosTorso7, file_desPosTorso8, file_desPosTorso9, file_desPosTorso10;
    std::ofstream file_forceFeetGlobal0, file_forceFeetGlobal1, file_forceFeetGlobal2, file_forceFeetGlobal3, file_forceFeetGlobal4, file_forceFeetGlobal5, file_forceFeetGlobal6, file_forceFeetGlobal7, file_forceFeetGlobal8, file_forceFeetGlobal9, file_forceFeetGlobal10, file_forceFeetGlobal11;
    std::ofstream file_simforceFeetGlobal0, file_simforceFeetGlobal1, file_simforceFeetGlobal2, file_simforceFeetGlobal3, file_simforceFeetGlobal4, file_simforceFeetGlobal5, file_simforceFeetGlobal6, file_simforceFeetGlobal7, file_simforceFeetGlobal8, file_simforceFeetGlobal9, file_simforceFeetGlobal10, file_simforceFeetGlobal11;
    std::ofstream file_actFoot0, file_actFoot1, file_actFoot2, file_actFoot6, file_actFoot7, file_actFoot8;
    std::ofstream file_desFoot0, file_desFoot1, file_desFoot2, file_desFoot6, file_desFoot7, file_desFoot8;
    
     // Robot State
    // Rob State
    Vec3  _posBody, _velBody, _VelFhc, _VelRhc, _wPosFhc, _wPosRhc; // Vec3, position and velocity of the torso in  in world frame
    Vec3  _bPosBody, _bVelBody; // Vec3, position and velocity of the torso in body frame
    Vec3  _bPosFhc, _bVelFhc, _bPosRhc, _bVelRhc; // Vec3, position and velocity of the front hip center in body frame
    double _yaw, _dYaw; // double, yaw and yaw rate
    Vec34 _posFeetGlobal, _velFeetGlobal; // Vec34, position and velocity of the feet in global frame
    Vec34 _posFeet2BGlobal; // Vec34, position of the feet relative to the torso in the body frame
    RotMat _B2G_RotMat, _G2B_RotMat; // RotMat, rotation matrix from body frame to global frame and vice versa
    Vec12 _q, _qGoal12, _dq, _tauM; // Vec12, joint angles
    Vec3 _bB2FrontCenterJC = Vec3(0.1805, 0, 0), _bB2RearCenterJC = Vec3(-0.1805, 0, 0); // Vec3, position vectors of front right and rear right hips with respect to CoM
    Vec3 _bFC2BJC = Vec3(-0.1805, 0, 0), _bRC2BJC = Vec3(0.1805, 0, 0); // Vec3, position vectors of front right and rear right hips with respect to CoM
    Vec3 _wB2FrontCenterJC, _wB2RearCenterJC; // Vec3, position vectors of front right and rear right hips with respect to CoM
    Vec3 WxR, WxWxR; // Vec3, vectors to hold result of xProd

    // Robot command
    Vec3 _pcd; // Vec3, desire torso, front and rear hip center position in global frame
    Vec3  _bfhcd, _brhcd, _wfhcd, _wrhcd; // Vec3, desire front and rear hip center position in body frame
    Vec3 _vCmdGlobal, _vbCmdBody, _vCmdFhc, _vbCmdBodyDhc, _vCmdUser, _vCmdRhc; // Vec3, desired torso velocity in global frame and body frame
    double _yawCmd, _dYawCmd, _pitchCmd, _rollCmd; // double, desired yaw and yaw rate
    double _dYawCmdPast; // double, desired yaw rate in the previous control cycle
    Vec3 _wCmdGlobal; // Vec3, desired torso angular velocity in global frame
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal; // Vec34, desired foot position and velocity in global frame
    Vec34 _posFeet2BGoal, _velFeet2BGoal, _posFeet2HGoal, _velFeet2HGoal; // Vec34, position and velocity of the feet relative to the torso in the body frame/ foot postion in hip frame
    RotMat _Rd; // RotMat, desired orientation of the torso in global frame
    Vec3 _ddPcd, _dWbd, _ddPcd_adj; // Vec3, desired torso acceleration in global frame and body frame
    Vec34 _forceFeetGlobal, _forceFeetBody; // Vec34, desired force of the feet in global frame and body frame
    Vec34 _qGoal, _qdGoal; // Vec34, desired joint angles and velocities
    Vec12 _tau; // Vec12, desired joint torques

    // Control Parameters
    double _gaitHeight, inipcd2; // double, height of the torso in stance phase
    Vec3 _posError, _velError; // Vec3, position and velocity error of the torso in global frame
    Mat3 _Kpp, _Kdp, _Kdw; // Mat3, proportional and derivative gains for position and velocity control
    Mat3 _kpw; // double, proportional gain for yaw control
    Mat3 _KpSwing, _KdSwing; // Mat3, proportional and derivative gains for swing leg control
    Vec2 _vxLim, _vyLim, _wyawLim; // Vec2, limits for the desired velocity and yaw rate
    Vec4 *_phase; // Vec4, phase of the four legs
    VecInt4 *_contact; // VecInt4, contact state of the four legs

    double _period, _stancePhaseRatio, _paramBound, _paramHipPos, _paramPitch, _forceFeetGlobalAdd[4];
    Vec4 _bias;
    double counterFront, counterRear, counterFrontUse, counterRearUse, counterFrontRecord, counterRearRecord, counterUse, counterStance, counterflight, counterDuStance;
    int counter, phaseCounter, flightDetector;
    double _posErrorFH, _velErrorFH, _posErrorRH, _velErrorRH, _fFH, _fRH;

    double stanceIteration, strideIteration;

    Vec34 _bB2HipCJ, _bHipC2HipCJ;

    float _footForce[4];

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);
};

#endif  // GALLOPING_G2_H