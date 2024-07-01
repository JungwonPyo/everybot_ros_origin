
#include "wayPoint.h"
#include "eblog.h"

CWayPoint::CWayPoint(/* args */) {}

CWayPoint &CWayPoint::operator=(const CWayPoint& ref)
{
    this->action.clear();
    for( tAction tempAction : ref.action )
    {
        this->action.push_back(tempAction);
    }
}

CWayPoint::CWayPoint(const CWayPoint& ref)
{
    this->action.clear();
    for(tAction refAction: ref.action)
    {
        this->action.push_back(refAction);
    }
}

CWayPoint::~CWayPoint()
{
    if(!action.empty())
    {
        action.clear();
    }
}

/**
 * @brief wayPoint 에 역할 push
 * 
 * @param push 
 */
void CWayPoint::pushAction(tAction push)
{
    action.push_back(push);
}

/**
 * @brief action list pop
 * 
 * @return tAction 
 */
tAction CWayPoint::popAction()
{
    tAction ret = action.front();
    if(action.size() > 0)
        action.pop_front();
    return ret;
}

tAction CWayPoint::getAction()
{
    return action.front();
}

/**
 * @brief wayPoint action 전부 삭제
 * 
 */
void CWayPoint::clearAction()
{
    action.clear();
}

/**
 * @brief wayPoint 수량 판단
 * 
 * @return int 
 */
int CWayPoint::getActionCount()
{
    return action.size();
}



void CWayPoint::setCleanPath(std::list<tPose> set)
{
    cleanPathList.clear();
    cleanPathList = set;
}

std::list<tPose> CWayPoint::getCleanPath()
{
    return cleanPathList;
}

std::list<tPoint> CWayPoint::getCurrentPath()
{
    std::list<tPoint> path;

    for( tAction a : action)
    {
        if( a.type == E_ACTION_TYPE::LINEAR_TO_POINT_ON_MAP || a.type == E_ACTION_TYPE::LINEAR_TO_LINE_ON_MAP )
        {
            path.push_back(a.linear.targetPoint);
        }
    }
    return path;
}
