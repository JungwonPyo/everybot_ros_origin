#include "coreData/serviceData/forbiddenArea.h"
#include "coreData/serviceData.h"
#include "eblog.h"

CForbiddenArea::CForbiddenArea(){}
CForbiddenArea::~CForbiddenArea(){}

void CForbiddenArea::setForbiddenLine()
{
    forbiddenArea.type = 1;
    tForbiddenLine* linePtr = ServiceData.rsBridge.getForbiddenLine().first;
    if(!forbiddenArea.line.empty()){
        forbiddenArea.line.clear();
    }

    if (ServiceData.rsBridge.getForbiddenLine().second == 0xff){
        forbiddenArea.lineNumber = 0;
    }
    else    forbiddenArea.lineNumber = ServiceData.rsBridge.getForbiddenLine().second;
    eblog(LOG_LV_NECESSARY," service customNumber : "<<forbiddenArea.lineNumber);
    for (int i = 0; i < forbiddenArea.lineNumber; ++i) {
        forbiddenArea.line.push_back(linePtr[i]);
        eblog(LOG_LV_NECESSARY," ForbiddenLine "<<"[" << i << "] : x1 = " << forbiddenArea.line.back().x1 
                                <<"[" << i << "] : y1 = " << forbiddenArea.line.back().y1
                                <<"[" << i << "] : x2 = " << forbiddenArea.line.back().x2
                                <<"[" << i << "] : y2 = " << forbiddenArea.line.back().y2);
    }
    ServiceData.awsData.setSendForbiddenLine();
}
std::list<tForbiddenLine> CForbiddenArea::getForbiddenLine()
{
    return forbiddenArea.line;
}

void CForbiddenArea::setForbiddenRect()
{
    forbiddenArea.type = 2;
    tForbiddenRect* rectPtr = ServiceData.rsBridge.getForbiddenRect().first;
    if(!forbiddenArea.rect.empty()){
        forbiddenArea.rect.clear();
    }

    if (ServiceData.rsBridge.getForbiddenRect().second == 0xff){
        forbiddenArea.rectNumber = 0;
    }
    else    forbiddenArea.rectNumber = ServiceData.rsBridge.getForbiddenRect().second;
    eblog(LOG_LV_NECESSARY," service customNumber : "<<forbiddenArea.rectNumber);
    for (int i = 0; i < forbiddenArea.rectNumber; ++i) {
        forbiddenArea.rect.push_back(rectPtr[i]);
        eblog(LOG_LV_NECESSARY," ForbiddenRect "<<"[" << i << "] : x = " << forbiddenArea.rect.back().x 
                                <<"[" << i << "] : y = " << forbiddenArea.rect.back().y
                                <<"[" << i << "] : w = " << forbiddenArea.rect.back().w
                                <<"[" << i << "] : h = " << forbiddenArea.rect.back().h); 
    }
    ServiceData.awsData.setSendForbiddenRect();
}
std::list<tForbiddenRect> CForbiddenArea::getForbiddenRect()
{
    return forbiddenArea.rect;
}