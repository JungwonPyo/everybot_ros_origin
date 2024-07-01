#include "wayPointManager.h"
#include "motionPlanner/motionPlanner.h"
#include "rosPublisher.h"

CWayPointManager::CWayPointManager()
{
    setWayPointStep(E_WAYPOINT_STEP::NONE);
}

CWayPointManager::~CWayPointManager()
{
}

void CWayPointManager::setWayPointStep(E_WAYPOINT_STEP set)
{
    if ( step != set )
    {
        ceblog(LOG_LV_CONTROL, BOLDBLACK, "wayPoint step 변경. ["<<WHITE<<enumToString(step)<<BOLDBLACK<<"] -> ["<<CYN<<enumToString(set)<<BOLDBLACK<<"]");
    }
    step = set;
}

E_WAYPOINT_STEP CWayPointManager::getWayPointStep()
{
    return step;
}

bool CWayPointManager::isActive()
{
    bool bRet = false;
    E_WAYPOINT_STEP waypointStep = getWayPointStep();
    if ( waypointStep == E_WAYPOINT_STEP::ACTION_POP || waypointStep == E_WAYPOINT_STEP::ACTION_WAIT )
        bRet = true;
    return bRet;
}

/**
 * @brief CWayPoint 새로 개정.
 * 
 * @param set 
 */
void CWayPointManager::setWayPoint(CWayPoint &set)
{
    wayPoint.clearAction();
    wayPoint = set;
    ceblog(LOG_LV_CONTROL, BOLDBLACK, "wayPoint 가 setup 됐습니다. getActionCount : " << wayPoint.getActionCount());
    setWayPointStep(E_WAYPOINT_STEP::NONE);

    debugCurrentPath.clear();
    debugCurrentPath = wayPoint.getCurrentPath();
}

void CWayPointManager::startWayPoint()
{
    setWayPointStep(E_WAYPOINT_STEP::ACTION_POP);
}

E_WAYPOINT_STEP CWayPointManager::runWayPoint()
{
    switch (getWayPointStep())
    {    
    case E_WAYPOINT_STEP::NONE:
        setWayPointStep(stepNone());
        break;
    case E_WAYPOINT_STEP::ACTION_POP:
        setWayPointStep(stepActionPop());
        break;    
    case E_WAYPOINT_STEP::ACTION_WAIT:    
        //회피 동작이 끝날때 까지 기다리자.
        setWayPointStep(stepActionWait());
        break;
    case E_WAYPOINT_STEP::COMPLETE:        
        // 회피 동작이 끝났다.
        setWayPointStep(stepComplete());        
        break;	
    default:
        break;
    }
    return getWayPointStep();
}

E_WAYPOINT_STEP CWayPointManager::stepNone()
{
    E_WAYPOINT_STEP ret = E_WAYPOINT_STEP::NONE;
    
    return ret;
}

tAction CWayPointManager::getCurrentAction()
{
    return currentAction;
}

E_WAYPOINT_STEP CWayPointManager::stepActionPop()
{
    E_WAYPOINT_STEP ret = E_WAYPOINT_STEP::ACTION_POP;

    currentAction = wayPoint.popAction();
    MOTION.actionStart(currentAction);
    ret = E_WAYPOINT_STEP::ACTION_WAIT;
    return ret;
}

E_WAYPOINT_STEP CWayPointManager::stepActionWait()
{
    E_WAYPOINT_STEP ret = E_WAYPOINT_STEP::ACTION_WAIT;

    if (MOTION.isRunning() == false){
        if(wayPoint.getActionCount() == 0){
            ret = E_WAYPOINT_STEP::COMPLETE;
            ceblog(LOG_LV_OBSTACLE, BLUE, "더이상 수행할 action 이 없습니다." );
            debugCurrentPath.clear();
            DEBUG_PUB.publishCurrentPathPlan(debugCurrentPath, tPose());
        }
        else{
            ret = E_WAYPOINT_STEP::ACTION_POP;
            ceblog(LOG_LV_OBSTACLE, BLUE, "다음 action 으로 고고.. 남은 action : "<< wayPoint.getActionCount()+1 );
            if(currentAction.type == E_ACTION_TYPE::LINEAR_TO_POINT_ON_MAP) {debugCurrentPath.pop_front();}
        }
    }
    
    return ret;
}

E_WAYPOINT_STEP CWayPointManager::stepComplete()
{
    E_WAYPOINT_STEP ret = E_WAYPOINT_STEP::COMPLETE;

    ret = E_WAYPOINT_STEP::NONE;
    
    return ret;
}

bool CWayPointManager::isWayPointComplete()
{
    return getWayPointStep() == E_WAYPOINT_STEP::COMPLETE ? true : false;
}