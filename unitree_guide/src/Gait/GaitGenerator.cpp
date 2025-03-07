/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _state(ctrlComp->lowState){
    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

GaitGenerator::~GaitGenerator(){
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::setGaitExtra(Vec3 vCmdBody){
    _vCmdBody = vCmdBody;
}


void GaitGenerator::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){
        _startP = _est->getFeetPos();
        _firstRun = false;
    }

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            if((*_phase)(i) < 0.5){
                _startP.col(i) = _est->getFootPos(i);
            }

            // if((*_phase)(i) < 0.8){
            //     _startP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));
            // }
            // else{
            //     _startP.col(i) = _est->getFootPos(i);_feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));
            // }

            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else{
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));

            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }

    _pastP = feetPos;
    _phasePast = *_phase;
}

Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;
    // if  (i<2){
    //     if (_vCmdBody(0) > 1)
    //     {
    //         footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i)) ;//+ 0.05;// * (_vCmdBody(0) - 1)
    //     }
    //     else if (_vCmdBody(0) > 3)
    //     {
    //         footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i)) + 0.01 * (_vCmdBody(0) - 2);
    //     }
    //     else
    //     {
    //         footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    //     }
    // }
    // else if (i>=2){
    //     if (_vCmdBody(0) > 1)
    //     {
    //         footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i)) ;//+ 0.05;// * (_vCmdBody(0) - 1)
    //     }
    //     else if (_vCmdBody(0) > 3)
    //     {
    //         footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i)) + 0.01 * (_vCmdBody(0) - 2);
    //     }
    //     else
    //     {
    //         footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    //     }
    // }
    footPos(0) = cycloidXPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i), i);    
    
    footPos(1) = cycloidYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i), i);

    
    if (i<2)
    {
        footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    }
    else
    {
        footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    }
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));

    if (i<2)
        footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));
    else{
        footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));
    }
    return footVel;
}

float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator::cycloidYPosition(float start, float end, float phase, int i){
    float phasePI = 2 * M_PI * phase;
    if (_vCmdBody(0) > 1.5){
        if (i<2)
            _footDistance = saturation((_vCmdBody(0) - 1.5) * 0.025, Vec2(0.0, 0.025)) * std::pow(-1.0, i);
        else{
            _footDistance = saturation((_vCmdBody(0) - 1.5) * 0.03, Vec2(0.0, 0.03)) * std::pow(-1.0, i+1);
        }
    }
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start + _footDistance;
}

float GaitGenerator::cycloidXPosition(float start, float end, float phase, int i){
    float phasePI = 2 * M_PI * phase;
    // if (_vCmdBody(0) > 2){
    //     if (i<2)
    //         _footExtension = saturation((_vCmdBody(0) - 2) * 0.05, Vec2(0.0, 0.1));
    //     else{
    //         _footExtension = saturation((_vCmdBody(0) - 2) * 0.05, Vec2(0.0, 0.1));
    //     }
    // }

    //1. add offset to the foot extension
    if (_vCmdBody(0) > 10){
        if (i<2)
            _footExtension = 1 * saturation((_vCmdBody(0) - 0) * 0.2, Vec2(0.0, 0.2));
        else{
            _footExtension = 1 * saturation((_vCmdBody(0) - 0) * 0.2, Vec2(0.0, 0.2));
        }
    }
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start + _footExtension;
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start;
}

float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing();
}