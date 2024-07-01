/**
 * @file systemData.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <memory>
#include "robotDefine.h"
#include "commonStruct.h"
#include "systemInterface_ts800.h"
#include "systemInterface_q8.h"
#include "systemTool.h"
#include "ebtypedef.h"

class CSystemData : public SystemInterfaceModel
{
public:
    CSystemData()
    {
        eblog(LOG_LV,  "");
    }

    ~CSystemData()
    {
        ceblog(LOG_LV, GREEN, "");
    }
};


