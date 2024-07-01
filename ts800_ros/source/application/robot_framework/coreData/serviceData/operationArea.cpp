#include "coreData/serviceData/operationArea.h"
#include "coreData/serviceData.h"
#include "eblog.h"

COperationArea::COperationArea(){}
COperationArea::~COperationArea(){}
 
void COperationArea::setAreaAll()
{
    operationArea.type = 1;
    operationArea.all = ServiceData.rsBridge.getAreaAll();
    ServiceData.awsData.setSendAreaAll();
}
short COperationArea::getAreaAll(){return operationArea.all;}

void COperationArea::setAreaSpot()
{
    operationArea.type = 2;
    tSpot* spotPtr = ServiceData.rsBridge.getAreaSpot().first;
    if(!operationArea.spot.empty()){
        operationArea.spot.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        operationArea.spot.push_back(spotPtr[i]);
        eblog(LOG_LV_AWS," Spot "<<"[" << i << "] : x = " << operationArea.spot.back().x
                                <<"[" << i << "] : y = " << operationArea.spot.back().y);
    }
    // operationArea.spot = ServiceData.rsBridge.getAreaSpot().first;
    if (ServiceData.rsBridge.getAreaSpot().second == 0xff){
        operationArea.spotNumber = 0;
    }
    else    operationArea.spotNumber = ServiceData.rsBridge.getAreaSpot().second;
    ServiceData.awsData.setSendAreaSpot();
}
std::list<tSpot> COperationArea::getAreaSpot(){return operationArea.spot;}

void COperationArea::setAreaRoom()
{
    operationArea.type = 3;
    tRoom* roomPtr = ServiceData.rsBridge.getAreaRoom().first;
    if(!operationArea.room.empty()){
        operationArea.room.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        operationArea.room.push_back(roomPtr[i]);
        eblog(LOG_LV_AWS," Room "<<"[" << i << "] : x = " << operationArea.room.back().x 
                                <<"[" << i << "] : y = " << operationArea.room.back().y
                                <<"[" << i << "] : w = " << operationArea.room.back().w
                                <<"[" << i << "] : h = " << operationArea.room.back().h);
    }
    if (ServiceData.rsBridge.getAreaRoom().second == 0xff){
        operationArea.roomNumber = 0;
    }
    else    operationArea.roomNumber = ServiceData.rsBridge.getAreaRoom().second;
    ServiceData.awsData.setSendAreaRoom();    
}
std::list<tRoom> COperationArea::getAreaRoom(){return operationArea.room;}

void COperationArea::setAreaCustom()
{
    operationArea.type = 4;
    tCustom* customPtr = ServiceData.rsBridge.getAreaCustom().first;
    if(!operationArea.custom.empty()){
        operationArea.custom.clear();
    }
    for (int i = 0; i < MAXLINE; ++i) {
        operationArea.custom.push_back(customPtr[i]);
        eblog(LOG_LV_AWS," Custom "<<"[" << i << "] : x = " << operationArea.custom.back().x 
                                <<"[" << i << "] : y = " << operationArea.custom.back().y
                                <<"[" << i << "] : w = " << operationArea.custom.back().w
                                <<"[" << i << "] : h = " << operationArea.custom.back().h);
    }
    if (ServiceData.rsBridge.getAreaCustom().second == 0xff){
        operationArea.customNumber = 0;
    }
    else    operationArea.customNumber = ServiceData.rsBridge.getAreaCustom().second;
    eblog(LOG_LV_AWS," service customNumber : "<<operationArea.customNumber);
    ServiceData.awsData.setSendAreaCustom();
}
std::list<tCustom> COperationArea::getAreaCustom(){return operationArea.custom;}