#include "coreData/serviceData/robotInfo.h"
#include "coreData/serviceData.h"
#include "eblog.h"

CRobotInfo::CRobotInfo(){}
CRobotInfo::~CRobotInfo(){}

void CRobotInfo::setRobotInfo()
{
    //AP Version
    robotInfo.apVersion = "Temp AP Version";
    ServiceData.awsData.setSendApVersion("Temp AP Version");

    //mcu Version
    robotInfo.mcuVersion = "Temp Mcu Version";
    ServiceData.awsData.setSendMcuVersion("Temp Mcu Version");
}

tRobotInfo CRobotInfo::getRobotInfo()
{
    return robotInfo;
}