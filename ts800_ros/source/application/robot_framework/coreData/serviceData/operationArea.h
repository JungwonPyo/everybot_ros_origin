#pragma once
#include "commonStruct.h"

typedef struct _tOperationArea
{
    _tOperationArea() {}


    short type; //1: all, 2: spot, 3: room, 4: custom
    short all;
    std::list<tSpot> spot;
    short spotNumber;
    std::list<tRoom> room;
    short roomNumber;
    std::list<tCustom> custom;
    short customNumber;

}tOperationArea;

class COperationArea
{
private:
    tOperationArea operationArea;
    
public:
    COperationArea();
    ~COperationArea();
    
    void setAreaAll();
    void setAreaSpot();
    void setAreaRoom();
    void setAreaCustom();

    short getAreaAll();

    std::list<tSpot> getAreaSpot();
    short getAreaSpotNum();
    
    std::list<tRoom> getAreaRoom();
    short getAreaRoomNum();
    
    std::list<tCustom> getAreaCustom();
    short getAreaCustomNum();

};