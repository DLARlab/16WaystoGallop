/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Bounding_CoM.h"
#include <iomanip>

#include "thirdParty/matplotlibcpp.h"
#include <iomanip>
#include <string>
#include <fstream>

namespace plt = matplotlibcpp;

State_Bounding_CoM::State_Bounding_CoM(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::BOUNDING_COM, "bounding_com"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = Vec3(780, 780, 780).asDiagonal(); 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
                _Kpp = Vec3(150, 70, 200).asDiagonal();
                _Kdp = Vec3(20, 20, 30).asDiagonal();
                _kpw = Vec3(400, 800, 400).asDiagonal();
                _Kdw = Vec3(50, 50, 50).asDiagonal();
                _KpSwing = Vec3(400, 400, 400).asDiagonal();
                _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

}

State_Bounding_CoM::~State_Bounding_CoM(){
    delete _gait;
}

void State_Bounding_CoM::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();


    _G2B_RotMat = _B2G_RotMat.transpose();


    _wB2FrontCenterJC =  _B2G_RotMat*Vec3(0.1805, 0, 0);
    _wB2RearCenterJC =  _B2G_RotMat*Vec3(-0.1805, 0, 0);

    file_q0.open("q0.txt");
    file_q1.open("q1.txt");
    file_q2.open("q2.txt");
    file_q6.open("q6.txt");
    file_q7.open("q7.txt");
    file_q8.open("q8.txt");

    file_qd0.open("qd0.txt");
    file_qd1.open("qd1.txt");
    file_qd2.open("qd2.txt");
    file_qd6.open("qd6.txt");
    file_qd7.open("qd7.txt");
    file_qd8.open("qd8.txt");

    file_posTorso0.open("posTorso0.txt");
    file_posTorso1.open("posTorso1.txt");
    file_posTorso2.open("posTorso2.txt");
    file_posTorso3.open("posTorso3.txt");
    file_posTorso4.open("posTorso4.txt");
    file_posTorso5.open("posTorso5.txt");
    file_posTorso6.open("posTorso6.txt");
    file_posTorso7.open("posTorso7.txt");
    file_posTorso8.open("posTorso8.txt");
    file_posTorso9.open("posTorso9.txt");

    file_desPosTorso0.open("desPosTorso0.txt");
    file_desPosTorso1.open("desPosTorso1.txt");
    file_desPosTorso2.open("desPosTorso2.txt");
    file_desPosTorso3.open("desPosTorso3.txt");
    file_desPosTorso4.open("desPosTorso4.txt");
    file_desPosTorso5.open("desPosTorso5.txt");
    file_desPosTorso6.open("desPosTorso6.txt");
    file_desPosTorso7.open("desPosTorso7.txt");
    file_desPosTorso8.open("desPosTorso8.txt");
    file_desPosTorso9.open("desPosTorso9.txt");
    file_desPosTorso10.open("desPosTorso10.txt");

    file_forceFeetGlobal0.open("forceFeetGlobal0.txt");
    file_forceFeetGlobal1.open("forceFeetGlobal1.txt");
    file_forceFeetGlobal2.open("forceFeetGlobal2.txt");
    file_forceFeetGlobal6.open("forceFeetGlobal6.txt");
    file_forceFeetGlobal7.open("forceFeetGlobal7.txt");
    file_forceFeetGlobal8.open("forceFeetGlobal8.txt");

    file_simforceFeetGlobal0.open("simforceFeetGlobal0.txt");
    file_simforceFeetGlobal1.open("simforceFeetGlobal1.txt");
    file_simforceFeetGlobal2.open("simforceFeetGlobal2.txt");
    file_simforceFeetGlobal6.open("simforceFeetGlobal6.txt");
    file_simforceFeetGlobal7.open("simforceFeetGlobal7.txt");
    file_simforceFeetGlobal8.open("simforceFeetGlobal8.txt");

    file_desFoot0.open("desFoot0.txt");
    file_desFoot1.open("desFoot1.txt");
    file_desFoot2.open("desFoot2.txt");
    file_desFoot6.open("desFoot6.txt");
    file_desFoot7.open("desFoot7.txt");
    file_desFoot8.open("desFoot8.txt");

    file_actFoot0.open("actFoot0.txt");
    file_actFoot1.open("actFoot1.txt");
    file_actFoot2.open("actFoot2.txt");
    file_actFoot6.open("actFoot6.txt");
    file_actFoot7.open("actFoot7.txt");
    file_actFoot8.open("actFoot8.txt");
}

void State_Bounding_CoM::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();

    _desQ.push_back(_q0_d);
    _desQ.push_back(_q1_d);
    _desQ.push_back(_q2_d);
    _desQ.push_back(_q3_d);
    _desQ.push_back(_q4_d);
    _desQ.push_back(_q5_d);
    _desQ.push_back(_q6_d);
    _desQ.push_back(_q7_d);
    _desQ.push_back(_q8_d);
    _desQ.push_back(_q9_d);
    _desQ.push_back(_q10_d);
    _desQ.push_back(_q11_d);

    _actQ.push_back(_q0);
    _actQ.push_back(_q1);
    _actQ.push_back(_q2);
    _actQ.push_back(_q3);
    _actQ.push_back(_q4);
    _actQ.push_back(_q5);
    _actQ.push_back(_q6);
    _actQ.push_back(_q7);
    _actQ.push_back(_q8);
    _actQ.push_back(_q9);
    _actQ.push_back(_q10);
    _actQ.push_back(_q11);

    _desFoot.push_back(_desFoot0);
    _desFoot.push_back(_desFoot1);
    _desFoot.push_back(_desFoot2);
    _desFoot.push_back(_desFoot6);
    _desFoot.push_back(_desFoot7);
    _desFoot.push_back(_desFoot8);

    _actFoot.push_back(_actFoot0);
    _actFoot.push_back(_actFoot1);
    _actFoot.push_back(_actFoot2);
    _actFoot.push_back(_actFoot6);
    _actFoot.push_back(_actFoot7);
    _actFoot.push_back(_actFoot8);


    _taoAll.push_back(_tao0);
    _taoAll.push_back(_tao1);
    _taoAll.push_back(_tao2);
    _taoAll.push_back(_tao3);
    _taoAll.push_back(_tao4);
    _taoAll.push_back(_tao5);
    _taoAll.push_back(_tao6);
    _taoAll.push_back(_tao7);
    _taoAll.push_back(_tao8);
    _taoAll.push_back(_tao9);
    _taoAll.push_back(_tao10);
    _taoAll.push_back(_tao11);

    _posFHipz.push_back(_posTorso3);
    _posFHipz.push_back(_posTorso4);
    _posFHipz.push_back(_q1);
    _posFHipz.push_back(_q2);

    _desPosFHipz.push_back(_desPosTorso3);
    _desPosFHipz.push_back(_desPosTorso4);
    _desPosFHipz.push_back(_q1_d);
    _desPosFHipz.push_back(_q2_d);

    _desPosFHipz.push_back(_desPosTorso9);
    _desPosFHipz.push_back(_desPosTorso10);

    _posRHipz.push_back(_posTorso5);
    _posRHipz.push_back(_posTorso6);
    _posRHipz.push_back(_q7);
    _posRHipz.push_back(_q8);

    _desPosRHipz.push_back(_desPosTorso5);
    _desPosRHipz.push_back(_desPosTorso6);
    _desPosRHipz.push_back(_q7_d);
    _desPosRHipz.push_back(_q8_d);

    _desPosRHipz.push_back(_desPosTorso9);
    _desPosRHipz.push_back(_desPosTorso10);

    _posTorso.push_back(_posTorso0);
    _posTorso.push_back(_posTorso1);
    _posTorso.push_back(_posTorso2);
    _posTorso.push_back(_posTorso7);

    _desPosTorso.push_back(_desPosTorso0);
    _desPosTorso.push_back(_desPosTorso1);
    _desPosTorso.push_back(_desPosTorso2);
    _desPosTorso.push_back(_desPosTorso7);

    _desPosTorso.push_back(_desPosTorso9);
    _desPosTorso.push_back(_desPosTorso10);

    _posXZ.push_back(_posTorso3);
    _posXZ.push_back(_posTorso4);
    _posXZ.push_back(_posTorso5);
    _posXZ.push_back(_posTorso6);

    _desPosXZ.push_back(_desPosTorso3);
    _desPosXZ.push_back(_desPosTorso4);
    _desPosXZ.push_back(_desPosTorso5);
    _desPosXZ.push_back(_desPosTorso6);

    _calGRF.push_back(_forceFeetGlobal0);
    _calGRF.push_back(_forceFeetGlobal1);
    _calGRF.push_back(_forceFeetGlobal2);
    _calGRF.push_back(_forceFeetGlobal3);
    _calGRF.push_back(_forceFeetGlobal4);
    _calGRF.push_back(_forceFeetGlobal5);
    _calGRF.push_back(_forceFeetGlobal6);
    _calGRF.push_back(_forceFeetGlobal7);
    _calGRF.push_back(_forceFeetGlobal8);
    _calGRF.push_back(_forceFeetGlobal9);
    _calGRF.push_back(_forceFeetGlobal10);
    _calGRF.push_back(_forceFeetGlobal11);
    _calGRF.push_back(_desPosTorso9);
    _calGRF.push_back(_desPosTorso10);

    _simGRF.push_back(_simGRF0);
    _simGRF.push_back(_simGRF1);
    _simGRF.push_back(_simGRF2);
    _simGRF.push_back(_simGRF3);
    _simGRF.push_back(_simGRF4);
    _simGRF.push_back(_simGRF5);
    _simGRF.push_back(_simGRF6);
    _simGRF.push_back(_simGRF7);
    _simGRF.push_back(_simGRF8);
    _simGRF.push_back(_simGRF9);
    _simGRF.push_back(_simGRF10);
    _simGRF.push_back(_simGRF11);

file_q0.close();
file_q1.close();
file_q2.close();
file_q6.close();
file_q7.close();
file_q8.close();

file_qd0.close();
file_qd1.close();
file_qd2.close();
file_qd6.close();
file_qd7.close();
file_qd8.close();

file_posTorso0.close();
file_posTorso1.close();
file_posTorso2.close();
file_posTorso3.close();
file_posTorso4.close();
file_posTorso5.close();
file_posTorso6.close();
file_posTorso7.close();
file_posTorso8.close();
file_posTorso9.close();
file_posTorso10.close();

file_desPosTorso0.close();
file_desPosTorso1.close();
file_desPosTorso2.close();
file_desPosTorso3.close();
file_desPosTorso4.close();
file_desPosTorso5.close();
file_desPosTorso6.close();
file_desPosTorso7.close();
file_desPosTorso8.close();
file_desPosTorso9.close();
file_desPosTorso10.close();

file_forceFeetGlobal0.close();
file_forceFeetGlobal1.close();
file_forceFeetGlobal2.close();
file_forceFeetGlobal6.close();
file_forceFeetGlobal7.close();
file_forceFeetGlobal8.close();

file_simforceFeetGlobal0.close();
file_simforceFeetGlobal1.close();
file_simforceFeetGlobal2.close();
file_simforceFeetGlobal4.close();
file_simforceFeetGlobal5.close();
file_simforceFeetGlobal6.close();

file_actFoot0.close();
file_actFoot1.close();
file_actFoot2.close();
file_actFoot6.close();
file_actFoot7.close();
file_actFoot8.close();

file_desFoot0.close();
file_desFoot1.close();
file_desFoot2.close();
file_desFoot6.close();
file_desFoot7.close();
file_desFoot8.close(); 


matplotlibcpp::figure(1);
    plotSubPlot(_desPosFHipz, _posFHipz, 0);
matplotlibcpp::figure(2);
    plotSubPlot(_desPosRHipz, _posRHipz, 2);
matplotlibcpp::figure(3);
    plotSubPlot(_desPosTorso, _posTorso, -1);
matplotlibcpp::figure(4);
    plotXZ(_desPosXZ, _posXZ);
matplotlibcpp::figure(5);
    plotGRF(_calGRF, _simGRF);
matplotlibcpp::figure(6);
    plotData(_desQ, _actQ);
matplotlibcpp::figure(7);
    plotData(_desFoot, _actFoot);
matplotlibcpp::show();

    //plotData(accg, accp);


}

FSMStateName State_Bounding_CoM::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::BOUNDING_COM;
    }
}

void State_Bounding_CoM::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();


    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    _bPosBody = _G2B_RotMat * _posBody;
    _bVelBody = _G2B_RotMat * _velBody;

    _bPosFhc = _bPosBody + _bB2FrontCenterJC;
    _bVelFhc = _bVelBody + xProd(_G2B_RotMat * _lowState->getGyroGlobal(), _bB2FrontCenterJC); // calculate the velocity of the front hip center in body frame
    _VelFhc = _B2G_RotMat * _bVelFhc; // calculate the velocity of the front hip center in world frame    

    _bPosRhc = _bPosBody + _bB2RearCenterJC;
    _bVelRhc = _bVelBody + xProd(_G2B_RotMat * _lowState->getGyroGlobal(), _bB2RearCenterJC); // calculate the velocity of the front hip center in body frame
    
    _wPosFhc = _B2G_RotMat * _bPosFhc;
    _wPosRhc = _B2G_RotMat * _bPosRhc;
    _VelFRc = _B2G_RotMat * _bVelRhc; // calculate the velocity of the front hip center in world frame    



    _userValue = _lowState->userValue;


    if(_vCmdUser(0) > 25 ){
         _period = 0.26;
         _stancePhaseRatio = 0.4;// -  saturation(0.05 * (_vCmdUser(0) - 2), Vec2(0.0, 0.15));
         _bias = Vec4(0, 0, 0.5, 0.5);
     std::cout << "_stancePhaseRatio: " << _stancePhaseRatio << std::endl;
    }
    else if(_vCmdUser(0) > 25 ){
         _period = 0.4;
         _stancePhaseRatio = 0.4;
         _bias = Vec4(0, 0, 0.5, 0.5);
    }    
    else{//(0.3, 0.2, Vec4(0, 0.1, 0.51, 0.61))
        _period = 0.3;
        _stancePhaseRatio = 0.4;
        _bias = Vec4(0, 0.1, 0.51, 0.61);
    }

    _ctrlComp->setWaveGen(&_period, &_stancePhaseRatio, &_bias);


    getUserCmd();
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setSmallGain(i);
        }
    }

    _q0_d.push_back(_qGoal(0));
    _q1_d.push_back(_qGoal(1));
    _q2_d.push_back(_qGoal(2));

    _q6_d.push_back(_qGoal(6));
    _q7_d.push_back(_qGoal(7));
    _q8_d.push_back(_qGoal(8));


    _q0.push_back(_q(0));
    _q1.push_back(_q(1));
    _q2.push_back(_q(2));

    _q6.push_back(_q(6));
    _q7.push_back(_q(7));
    _q8.push_back(_q(8));

    if(file_q0.is_open()){file_q0<< _q(0)<< "\n";}
    if(file_q1.is_open()){file_q1<< _q(1)<< "\n";}
    if(file_q2.is_open()){file_q2<< _q(2)<< "\n";}
    if(file_q6.is_open()){file_q6<< _q(6)<< "\n";}
    if(file_q7.is_open()){file_q7<< _q(7)<< "\n";}
    if(file_q8.is_open()){file_q8<< _q(8)<< "\n";}
    if(file_qd0.is_open()){file_qd0<< _qGoal(0)<< "\n";}
    if(file_qd1.is_open()){file_qd1<< _qGoal(1)<< "\n";}
    if(file_qd2.is_open()){file_qd2<< _qGoal(2)<< "\n";}
    if(file_qd6.is_open()){file_qd6<< _qGoal(6)<< "\n";}
    if(file_qd7.is_open()){file_qd7<< _qGoal(7)<< "\n";}
    if(file_qd8.is_open()){file_qd8<< _qGoal(8)<< "\n";}

}

bool State_Bounding_CoM::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }else{
        return false;
    }
}

// void State_Bounding_CoM::setHighCmd(double vx, double vy, double wz){
//     _vCmdBody(0) = vx;
//     _vCmdBody(1) = vy;
//     _vCmdBody(2) = 0; 
//     _dYawCmd = wz;
// }

void State_Bounding_CoM::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1), -10, 10);
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    _vCmdUser = _vCmdBody;
    std::cout << "vCmdBody: " << _vCmdBody(0) << std::endl;
    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;


}

void State_Bounding_CoM::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));


    _vCmdGlobal(2) = 0;
 std::cout << "vCmdBody0: " << _vCmdBody(0) << std::endl;
 std::cout << "_velBody: " << _velBody(0) << std::endl;
    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;

    _posTorso0.push_back(_posBody(0));
    _posTorso1.push_back(_lowState->getPitch());
    _posTorso2.push_back(_posBody(2));
    _posTorso3.push_back(_wPosFhc(0));
    _posTorso4.push_back(_wPosFhc(2));
    _posTorso5.push_back(_wPosRhc(0));
    _posTorso6.push_back(_wPosRhc(2));
    _posTorso7.push_back(_est->getVelocity()(0));

    _desPosTorso0.push_back(_pcd(0));
    _desPosTorso1.push_back(0);
    _desPosTorso2.push_back(_pcd(2));
    _desPosTorso3.push_back(0);
    _desPosTorso4.push_back(0.28);
    _desPosTorso5.push_back(0);
    _desPosTorso6.push_back(0.28);
    _desPosTorso7.push_back(_vCmdUser(0));

    _desPosTorso9.push_back(((*_contact)(0) - 0.5) * 10);
    _desPosTorso10.push_back(((*_contact)(2) - 0.5) * 10);

    if(file_posTorso0.is_open()){file_posTorso0<< _posBody(0)<< "\n";}
    if(file_posTorso1.is_open()){file_posTorso1<< _lowState->getPitch()<< "\n";}
    if(file_posTorso2.is_open()){file_posTorso2<< _posBody(2)<< "\n";}
    if(file_posTorso3.is_open()){file_posTorso3<< _wPosFhc(0)<< "\n";}
    if(file_posTorso4.is_open()){file_posTorso4<< _wPosFhc(2)<< "\n";}
    if(file_posTorso5.is_open()){file_posTorso5<< _wPosRhc(0)<< "\n";}
    if(file_posTorso6.is_open()){file_posTorso6<< _wPosRhc(2)<< "\n";}
    if(file_posTorso7.is_open()){file_posTorso7<< _est->getVelocity()(0)<< "\n";}
    if(file_posTorso8.is_open()){file_posTorso8<< _footForce[0]<< "\n";}
    if(file_posTorso9.is_open()){file_posTorso9<< _footForce[2]<< "\n";}
    

    if(file_desPosTorso0.is_open()){file_desPosTorso0<< _pcd(0)<< "\n";}
    if(file_desPosTorso1.is_open()){file_desPosTorso1<< 0<< "\n";}
    if(file_desPosTorso2.is_open()){file_desPosTorso2<< _pcd(2)<< "\n";}
    if(file_desPosTorso3.is_open()){file_desPosTorso3<< 0<< "\n";}
    if(file_desPosTorso4.is_open()){file_desPosTorso4<< 0.28<< "\n";}
    if(file_desPosTorso5.is_open()){file_desPosTorso5<< 0<< "\n";}
    if(file_desPosTorso6.is_open()){file_desPosTorso6<< 0.28<< "\n";}
    if(file_desPosTorso7.is_open()){file_desPosTorso7<< _vCmdUser(0)<< "\n";}
    if(file_desPosTorso9.is_open()){file_desPosTorso9<< ((*_contact)(0) - 0.5) * 10<< "\n";}
    if(file_desPosTorso10.is_open()){file_desPosTorso10<< ((*_contact)(2) - 0.5) * 10<< "\n";}


}

void State_Bounding_CoM::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-70, 70)); // limit the desired acceleration of the torso in x
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-50, 50));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-90, 90));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-160, 160));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-660, 660));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-160, 160));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

    _forceFeetGlobal0.push_back(_forceFeetGlobal(0,0));
    _forceFeetGlobal1.push_back(_forceFeetGlobal(1,0));
    _forceFeetGlobal2.push_back(_forceFeetGlobal(2,0));
    _forceFeetGlobal3.push_back(_forceFeetGlobal(0,1));
    _forceFeetGlobal4.push_back(_forceFeetGlobal(1,1));
    _forceFeetGlobal5.push_back(_forceFeetGlobal(2,1));
    _forceFeetGlobal6.push_back(_forceFeetGlobal(0,2));
    _forceFeetGlobal7.push_back(_forceFeetGlobal(1,2));
    _forceFeetGlobal8.push_back(_forceFeetGlobal(2,2));
    _forceFeetGlobal9.push_back(_forceFeetGlobal(0,3));
    _forceFeetGlobal10.push_back(_forceFeetGlobal(1,3));
    _forceFeetGlobal11.push_back(_forceFeetGlobal(2,3));

    for (int i = 0; i < 4; i++){
        if (_lowState->footForce[i] >10000){
            _footForce[i] = 0.0;
        }
        else{
            _footForce[i] = _lowState->footForce[i];
        }
    }
    _simGRF0.push_back(_lowState->eeForce[0].x);
    _simGRF1.push_back(_lowState->eeForce[0].y);
    _simGRF2.push_back(_footForce[0]);
    _simGRF3.push_back(_lowState->eeForce[1].x);
    _simGRF4.push_back(_lowState->eeForce[1].y);
    _simGRF5.push_back(_footForce[1]);
    _simGRF6.push_back(_lowState->eeForce[2].x);
    _simGRF7.push_back(_lowState->eeForce[2].y);
    _simGRF8.push_back(_footForce[2]);
    _simGRF9.push_back(_lowState->eeForce[3].x);
    _simGRF10.push_back(_lowState->eeForce[3].y);
    _simGRF11.push_back(_footForce[3]);

    if(file_forceFeetGlobal0.is_open()){file_forceFeetGlobal0<< _forceFeetGlobal(0,0)<< "\n";}
    if(file_forceFeetGlobal1.is_open()){file_forceFeetGlobal1<< _forceFeetGlobal(1,0)<< "\n";}
    if(file_forceFeetGlobal2.is_open()){file_forceFeetGlobal2<< _forceFeetGlobal(2,0)<< "\n";}
    if(file_forceFeetGlobal6.is_open()){file_forceFeetGlobal6<< _forceFeetGlobal(0,2)<< "\n";}
    if(file_forceFeetGlobal7.is_open()){file_forceFeetGlobal7<< _forceFeetGlobal(1,2)<< "\n";}
    if(file_forceFeetGlobal8.is_open()){file_forceFeetGlobal8<< _forceFeetGlobal(2,2)<< "\n";}

    if(file_simforceFeetGlobal0.is_open()){file_simforceFeetGlobal0<< _lowState->eeForce[0].x<< "\n";}
    if(file_simforceFeetGlobal1.is_open()){file_simforceFeetGlobal1<< _lowState->eeForce[0].y<< "\n";}
    if(file_simforceFeetGlobal2.is_open()){file_simforceFeetGlobal2<< _footForce[0]<< "\n";}
    if(file_simforceFeetGlobal6.is_open()){file_simforceFeetGlobal6<< _lowState->eeForce[2].x<< "\n";}
    if(file_simforceFeetGlobal7.is_open()){file_simforceFeetGlobal7<< _lowState->eeForce[2].y<< "\n";}
    if(file_simforceFeetGlobal8.is_open()){file_simforceFeetGlobal8<< _footForce[2]<< "\n";}

}

void State_Bounding_CoM::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));

    _desFoot0.push_back(_posFeet2B(0,0));
    _desFoot1.push_back(_posFeet2B(1,0));
    _desFoot2.push_back(_posFeet2B(2,0));

    _desFoot6.push_back(_posFeet2B(0,2));
    _desFoot7.push_back(_posFeet2B(1,2));
    _desFoot8.push_back(_posFeet2B(2,2));

    _actFoot0.push_back(_posFeet2BGlobal(0,0));
    _actFoot1.push_back(_posFeet2BGlobal(1,0));
    _actFoot2.push_back(_posFeet2BGlobal(2,0));

    _actFoot6.push_back(_posFeet2BGlobal(0,2));
    _actFoot7.push_back(_posFeet2BGlobal(1,2));
    _actFoot8.push_back(_posFeet2BGlobal(2,2));



    if(file_desFoot0.is_open()){file_desFoot0<< _posFeet2B(0,0)<< "\n";}
    if(file_desFoot1.is_open()){file_desFoot1<< _posFeet2B(1,0)<< "\n";}
    if(file_desFoot2.is_open()){file_desFoot2<< _posFeet2B(2,0)<< "\n";}
    if(file_desFoot6.is_open()){file_desFoot6<< _posFeet2B(0,2)<< "\n";}
    if(file_desFoot7.is_open()){file_desFoot7<< _posFeet2B(1,2)<< "\n";}
    if(file_desFoot8.is_open()){file_desFoot8<< _posFeet2B(2,2)<< "\n";}

    if(file_actFoot0.is_open()){file_actFoot0<< _posFeet2BGoal(0,0)<< "\n";}
    if(file_actFoot1.is_open()){file_actFoot1<< _posFeet2BGoal(1,0)<< "\n";}
    if(file_actFoot2.is_open()){file_actFoot2<< _posFeet2BGoal(2,0)<< "\n";}
    if(file_actFoot6.is_open()){file_actFoot6<< _posFeet2BGoal(0,2)<< "\n";}
    if(file_actFoot7.is_open()){file_actFoot7<< _posFeet2BGoal(1,2)<< "\n";}
    if(file_actFoot8.is_open()){file_actFoot8<< _posFeet2BGoal(2,2)<< "\n";}
}


Vec3 State_Bounding_CoM::xProd(Vec3 vec1, Vec3 vec2){
    Vec3 vec3;

    vec3(0) = vec1(1)*vec2(2) - vec1(2)*vec2(1);
    vec3(1) = -1*(vec1(0)*vec2(2) - vec1(2)*vec2(0));
    vec3(2) = vec1(0)*vec2(1) - vec1(1)*vec2(0);
    return(vec3);
}

void State_Bounding_CoM::plotSubPlot(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data, int legNum){
    
    matplotlibcpp::figure_size(1600, 780);
    std::string legName, keyword;
    int size1 = desd_data.size();
    int size2 = actual_data.size();
    size_t numCols = (size1 > 0) ? desd_data[0].size() : 0;

    std::vector<double> time(numCols);
    for(int i = 0; i < numCols; i++){
        time[i] = i * 0.002;
    }
    std::string label1 = "desired";
    std::string label2 = "acutal";
    std::string label3 = "FT";
    std::string label4 = "RT";
    switch (legNum){
        case 0:
            legName = "Front Right";
            break;
        case 1:
            legName = "Front Left";
            break;
        case 2:
            legName = "Rear Right";
            break;
        case 3:
            legName = "Rear Left";
            break;
        default:
            legName = "Torso";
            break;
    }


    auto chooseLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes
        std::vector<std::string> colors = {
            "r", "g", "b",    // Red, Green, Blue
            "c", "m", "y",    // Cyan, Magenta, Yellow
            "k", "gray",         // Black, White
            "pink", "purple", "orange", "olive", "brown"  // Other colors
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k";  // Default to black for out-of-range values
        }
    };

    auto chooseDashedLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes with dashed lines
        std::vector<std::string> colors = {
            "r--", "g--", "b--",    // Red, Green, Blue with dashed lines
            "c--", "m--", "y--",    // Cyan, Magenta, Yellow with dashed lines
            "k--",           // Black, White with dashed lines
             ":", ":",    
            ":", ":", ":"
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k--";  // Default to black dashed line for out-of-range values
        }
    };

    if (size2 > 0) {
        matplotlibcpp::suptitle(legName.c_str());
        for (int i = 0; i < size2; i++) {   
                switch (i){
                    case 0:
                        keyword = legName + " Hip in x";
                        break;
                    case 1:
                        keyword = legName + " Hip in z";
                        break;
                    case 2:
                        keyword = legName + " Joint Thigh";
                        break;
                    case 3:
                        keyword = legName + " Joint Calf";
                        break;
                }
            if (legNum == -1) keyword = "Torso";
            matplotlibcpp::subplot(2, size2/2, i+1);

            matplotlibcpp::named_plot(label1, time, desd_data[i], chooseDashedLineColor(i));
            // std::string label = "actual";
            matplotlibcpp::named_plot(label2, time, actual_data[i], chooseLineColor(i));

            matplotlibcpp::named_plot(label3, time, desd_data[size2], chooseDashedLineColor(9));
            matplotlibcpp::named_plot(label4, time, desd_data[size2+1], chooseDashedLineColor(10));
            matplotlibcpp::title(keyword.c_str());
            //plt::ylim(0.4, 1.1);
            matplotlibcpp::legend();
        }
    }
}

void State_Bounding_CoM::plotGRF(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data){
    
    matplotlibcpp::figure_size(1600, 780);
    std::string legName, keyword;
    int size1 = desd_data.size();
    int size2 = actual_data.size();
    std::cout << "size1: " << size1 << std::endl;
    std::cout << "size2: " << size2 << std::endl;
    size_t numCols = (size1 > 0) ? desd_data[0].size() : 0;
 std::cout << "numCols: " << numCols << std::endl;
    std::vector<double> time(numCols);
    for(int i = 0; i < numCols; i++){
        time[i] = i * 0.002;
    }
    std::string label1 = "desired";
    std::string label2 = "acutal";
    std::string label3 = "FT";
    std::string label4 = "RT";

    legName = "GRF";



    auto chooseLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes
        std::vector<std::string> colors = {
            "r", "g", "b",    // Red, Green, Blue
            "c", "m", "y",    // Cyan, Magenta, Yellow
            "k", "gray",         // Black, White
            "pink", "purple", "orange", "olive", "brown"  // Other colors
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k";  // Default to black for out-of-range values
        }
    };

    auto chooseDashedLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes with dashed lines
        std::vector<std::string> colors = {
            "r--", "g--", "b--",    // Red, Green, Blue with dashed lines
            "c--", "m--", "y--",    // Cyan, Magenta, Yellow with dashed lines
            "k--",           // Black, White with dashed lines
             ":", ":",    
            ":", ":", ":"
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k--";  // Default to black dashed line for out-of-range values
        }
    };

    if (size2 > 0) {
        matplotlibcpp::suptitle(legName.c_str());
        for (int i = 0; i < 4; i++) {   
                switch (i){
                    case 0:
                        keyword = legName + " in front right";
                        break;
                    case 1:
                        keyword = legName + " in front left";
                        break;
                    case 2:
                        keyword = legName + " in rear right";
                        break;
                    case 3:
                        keyword = legName + " in rear left";
                        break;
                }

            matplotlibcpp::subplot(2, 2, i+1);
            for (int j = 0; j < 3; j++) {   
                matplotlibcpp::named_plot(label1, time, desd_data[i*3 + j], chooseDashedLineColor(j));
                // std::string label = "actual";
                matplotlibcpp::named_plot(label2, time, actual_data[i*3 + j], chooseLineColor(j));
            }

            matplotlibcpp::named_plot(label3, time, desd_data[size2], chooseDashedLineColor(9));
            matplotlibcpp::named_plot(label4, time, desd_data[size2+1], chooseDashedLineColor(10));
            matplotlibcpp::title(keyword.c_str());
            //plt::ylim(0.4, 1.1);
            matplotlibcpp::legend();
        }
    }

}

void State_Bounding_CoM::plotXZ(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data){
    
    matplotlibcpp::figure_size(1200, 780);
    std::string legName, keyword;
    int size1 = desd_data.size();
    int size2 = actual_data.size();
    size_t numCols = (size1 > 0) ? desd_data[0].size() : 0;

    std::vector<double> time(numCols);
    for(int i = 0; i < numCols; i++){
        time[i] = i * 0.002;
    }
    std::string label1 = "desired";
    std::string label2 = "acutal";
    std::string label3 = "FT";
    std::string label4 = "RT";



    auto chooseLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes
        std::vector<std::string> colors = {
            "r", "g", "b",    // Red, Green, Blue
            "c", "m", "y",    // Cyan, Magenta, Yellow
            "k", "gray",         // Black, White
            "pink", "purple", "orange", "olive", "brown"  // Other colors
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k";  // Default to black for out-of-range values
        }
    };

    auto chooseDashedLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes with dashed lines
        std::vector<std::string> colors = {
            "r--", "g--", "b--",    // Red, Green, Blue with dashed lines
            "c--", "m--", "y--",    // Cyan, Magenta, Yellow with dashed lines
            "k--",           // Black, White with dashed lines
             ":", ":",    
            ":", ":", ":"
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k--";  // Default to black dashed line for out-of-range values
        }
    };

        matplotlibcpp::suptitle(legName.c_str());

            matplotlibcpp::subplot(2, 1, 1);

            matplotlibcpp::named_plot(label1, desd_data[0], desd_data[1], chooseDashedLineColor(0));
            // std::string label = "actual";
            matplotlibcpp::named_plot(label2, actual_data[0], actual_data[1], chooseLineColor(1));

            matplotlibcpp::title("Front Hip");
            //plt::ylim(0.4, 1.1);
            matplotlibcpp::legend();

            matplotlibcpp::subplot(2, 1, 2);

            matplotlibcpp::named_plot(label1, desd_data[2], desd_data[3], chooseDashedLineColor(0));
            // std::string label = "actual";
            matplotlibcpp::named_plot(label2, actual_data[2], actual_data[3], chooseLineColor(1));

            matplotlibcpp::title("Rear Hip");
            //plt::ylim(0.4, 1.1);
            matplotlibcpp::legend();
}

void State_Bounding_CoM::plotData(std::vector<std::vector<double>>& desd_data, std::vector<std::vector<double>>& actual_data){
    
    matplotlibcpp::figure_size(1200, 780);

    int size1 = desd_data.size();
    int size2 = actual_data.size();
    size_t numCols = (size1 > 0) ? desd_data[0].size() : 0;

    std::vector<double> time(numCols);
    for(int i = 0; i < numCols; i++){
        time[i] = i * 0.002;
    }

    auto chooseLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes
        std::vector<std::string> colors = {
            "r", "g", "b",    // Red, Green, Blue
            "c", "m", "y",    // Cyan, Magenta, Yellow
            "k", "gray",         // Black, White
            "pink", "purple", "orange", "olive", "brown"  // Other colors
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k";  // Default to black for out-of-range values
        }
    };

    auto chooseDashedLineColor = [](int value) -> std::string {
        // Define an array of 12 distinct color codes with dashed lines
        std::vector<std::string> colors = {
            "r--", "g--", "b--",    // Red, Green, Blue with dashed lines
            "c--", "m--", "y--",    // Cyan, Magenta, Yellow with dashed lines
            "k--",           // Black, White with dashed lines
             ":", ":",    
            ":", ":", ":"
        };

        // Check if the value is within the valid range
        if (value >= 0 && value < colors.size()) {
            return colors[value];
        } else {
            return "k--";  // Default to black dashed line for out-of-range values
        }
    };
   
    if (size1 > 0) {
        for (int i = 0; i < size1; i++) {
            std::string label = "desired " + std::to_string(i+1);
            matplotlibcpp::named_plot(label, time, desd_data[i], chooseDashedLineColor(i));
        }
    }
    
    if (size2 >0 )
    {
        for(int i = 0; i < size2; i++){
            std::string label = "actual" + std::to_string(i+1);
            matplotlibcpp::named_plot(label, time, actual_data[i], chooseLineColor(i));
        }
    }

    //plt::ylim(0.4, 1.1);
    matplotlibcpp::legend();
}