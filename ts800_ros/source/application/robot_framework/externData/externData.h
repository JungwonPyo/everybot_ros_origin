/**
 * @file externData.h
 * @author jspark
 * @brief rosInterface 와 systemInterface 의 데이터들을 저장.
 * @date 2023-05-03
 */
#pragma once

#include "externData/systemData.h"
#include "externData/appData.h"
#include "externData/rosData/rosData.h"

class CExternData
{
public:
    CSystemData     systemData;
    CAppData        appData;
    CRosData        rosData;

public:
    CExternData(ros::NodeHandle _nh);
    ~CExternData();
};