#pragma once
#include "commonStruct.h"

typedef struct _tRobotInfo
{
  _tRobotInfo() {}
  _tRobotInfo(std::string _apVersion, std::string _mcuVersion): apVersion(_apVersion),mcuVersion(_mcuVersion) {}

  std::string apVersion; 
  std::string mcuVersion; 

}tRobotInfo;


class CRobotInfo
{
private:
    tRobotInfo robotInfo;
public:
    CRobotInfo();
    ~CRobotInfo();

    void setRobotInfo();
    tRobotInfo getRobotInfo();
};