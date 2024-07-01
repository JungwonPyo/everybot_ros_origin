/**
 * @file robotModel.h
 * @author jspark
 * @brief TS800 프로젝트와 Q8 프로젝트의 소스코드 통일화를 위해
 * 두 프로젝트의 차이점들은 #define 으로 코드 일치 권장.
 * @version 0.1
 * @date 2023-04-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "define.h"

// systemInterface 모델별로 분리. jspark 23.04.17
#if ( (ROBOT_MODEL == TS800_ES) || (ROBOT_MODEL == TS800_WS) ) // TS800 일 경우
    #define SystemInterfaceModel    CSystemInterfaceTs800
#else // Q8 일 경우
    #define SystemInterfaceModel    CSystemInterfaceQ8
#endif