/**
 * @file tiltingState.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "coreData/observer.h"
#include "interfaceStruct.h"

class CTiltingState : public CObserver
{
private:
    E_SYS_TILT_STATE state;
    void setStateValue(E_SYS_TILT_STATE newKey);    

public:
    CTiltingState();
    ~CTiltingState();
    void update(CExternData* pExternData) override;
    E_SYS_TILT_STATE getStateValue(void);
};