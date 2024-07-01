#include <cmath>
#include "motionController.h"
#include "control/control.h"
#include "utils.h"
#include "MessageHandler.h"
#include "subTask.h"
#include "motionPlanner/motionPlanner.h"
#include "debugCtr.h"
#include <rosPublisher.h>

#define MOTION_CONTROLLER_DEBUG_LOG_ON 0

using namespace std;

CMotionController::CMotionController()
{
    setState(E_STATE::NOTHING);
    pthread_mutex_init(&mutexPointControl, nullptr);
    msgSeq = 0;

    waitingTime = 0;

    curLinVel = 0;
    curAngVel = 0;
    combineTwist = tTwist(curLinVel, curAngVel);
}

CMotionController::~CMotionController()
{
    bThMotion = false;
    pthread_mutex_destroy(&mutexPointControl);
    pthread_join(thMotionHandler, nullptr);
}

CMotionController& CMotionController::getInstance()
{
    static CMotionController s;
    
    return s;
}
/**
 * @brief 남겨둠 2024.05.23 (by hyjoe)
 * 
 * @param action 
 */
void CMotionController::actionStart(tAction action)
{
    ceblog(LOG_LV_NECESSARY, GREEN, "action Start 합니다. : "<<enumToString(action.type) << " power state: " << DEC(ServiceData.power.getPowerState()) << " tilt state:" << enumToString(ServiceData.tilting.getStateValue()));
    
    tPose robotPose = ServiceData.localiz.getPose();

    switch (action.type)
    {   
    case  E_ACTION_TYPE::LINEAR_TO_POINT_ON_MAP:
        ceblog(LOG_LV_NECESSARY, GREEN, " > Map["<<action.linear.targetPoint.x<<", "<<action.linear.targetPoint.y<<" 목표점으로 직진 제어");
        if(pwmControl)
        {
            action.type = E_ACTION_TYPE::LINEAR_TO_DISTANCE_ON_ROBOT;
            if(utils::math::distanceTwoPoint(action.linear.targetPoint,robotPose) >= 0)   MOTION.startDriveWheelPwm(robotPose,1,200,200,50,(int)(utils::math::distanceTwoPoint(action.linear.targetPoint,robotPose)*1000),true,action.profile);
            else                                                                          MOTION.startDriveWheelPwm(robotPose,1,-200,-200,50,(int)(utils::math::distanceTwoPoint(action.linear.targetPoint,robotPose)*1000),true,action.profile);
        }
        else
        {
            MOTION.startLinearToPointOnMap(robotPose, action.linear.targetPoint, action.profile);
        }                    
        break;

    case  E_ACTION_TYPE::TILTING_UP:
        ceblog(LOG_LV_NECESSARY, GREEN, " > 틸팅 up");
        ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::UP);
        break;

    case  E_ACTION_TYPE::TILTING_DOWN:
        ceblog(LOG_LV_NECESSARY, GREEN, " > 틸팅 down");
        ROBOT_CONTROL.startTilt(E_TILTING_CONTROL::DOWN);
        break;

    case  E_ACTION_TYPE::STOP:
        
        break;

    case  E_ACTION_TYPE::NONE:
        ceblog(LOG_LV_NECESSARY, BOLDRED, "잘못된 Action 사용입니다.");
        break;
    
    default:
        ceblog(LOG_LV_NECESSARY, BOLDRED, "잘못된 Action 사용입니다.-default");
        break;
    }
}

void CMotionController::init()
{
    bThMotion = true;
    encCnt = 0;
    debugCnt = 0;
    encCnt = 0;
}

void CMotionController::clearVelocity()
{
    if(pwmControl)
    {
        pwmInfo.direction = 0;
        pwmInfo.lspeed = 0;
        pwmInfo.rspeed = 0;
        pwmInfo.bspeed = 0;
        pwmInfo.duty = 0;
        pwmInfo.pid = false;
    }
    else
    {
        curLinVel = 0.0;
        curAngVel = 0.0;
        combineTwist = tTwist(curLinVel, curAngVel);
    }
}

/**
 * @brief 정지 제어기
 * 
 * @param robotPose 
 * @param prof 
 * @param emergency 
 */
void CMotionController::startStopOnMap(tProfile prof, bool emergency)
{
    ceblog(LOG_LV_NECESSARY, RED," [ STOP ] 정지 제어");
    //eblog(LOG_LV_NECESSARY, "robotPose : "<<robotPose.x<<" , "<<robotPose.y<<" , "<<robotPose.angle);

    pwmControl = false;    
    controlType = E_CONTROL_TYPE::STOP;
    DrvInfo.bStopEmergency = emergency;

    setState(E_STATE::RUNNING);
}

/**
 * @brief 전방 포인트 이동 제어기
 * 
 * @param robotPose 
 * @param targetPoint 
 * @param prof 
 */
void CMotionController::startLinearToPointOnMap(const tPose& robotPose, tPoint targetPoint, tProfile prof)
{
    ceblog(LOG_LV_NECESSARY, YELLOW, " [ LINEAR ] 전방 포인트 이동 제어");
    pwmControl = false;    
    controlType = E_CONTROL_TYPE::LINEAR;

    DrvInfo.pf = prof;
    DrvInfo.startPose = robotPose;
    DrvInfo.targetPoint = targetPoint;
 
    setPointLists(robotPose, targetPoint, 0.05);
    lineCoeff = utils::math::calculateLineEquation(tPoint(robotPose.x, robotPose.y), targetPoint);
#if defined (DEBUG_ROS_PUB) && (DEBUG_ROS_PUB == 1)
    DEBUG_PUB.publishCleanLine1(robotPose, targetPoint);
#endif
    setState(E_STATE::RUNNING);
}

/**
 * @brief 전방 각속 우선 포인트 이동 제어기
 * 
 * @param robotPose 
 * @param targetPoint 
 * @param prof 
 */
void CMotionController::startLinearAngularPriorToPointOnMap(const tPose& robotPose, tPoint targetPoint, tProfile prof)
{
    ceblog(LOG_LV_NECESSARY, YELLOW, " [ ANGULAR_PRIOR ] 전방 각속 우선 포인트 이동 제어");
    pwmControl = false;    
    controlType = E_CONTROL_TYPE::ANGULAR_PRIOR;
    currentStep = E_ANGULAR_PRIOR_CONTROL_STEP::V_DECEL;

    DrvInfo.pf = prof;
    DrvInfo.startPose = robotPose;
    DrvInfo.targetPoint = targetPoint;

    setPointLists(robotPose, targetPoint, 0.05);
    lineCoeff = utils::math::calculateLineEquation(tPoint(robotPose.x, robotPose.y), targetPoint);
#if defined (DEBUG_ROS_PUB) && (DEBUG_ROS_PUB == 1)
    DEBUG_PUB.publishCleanLine1(robotPose, targetPoint);
#endif
    setState(E_STATE::RUNNING);
}

/**
 * @brief 후방 포인트 이동 제어기
 * 
 * @param robotPose 
 * @param targetPoint 
 * @param prof 
 */
void CMotionController::startBackToPointOnMap(const tPose& robotPose, tPoint targetPoint, tProfile prof)
{
    ceblog(LOG_LV_NECESSARY, YELLOW," [ BACK ] 후방 포인트 이동 제어");
    pwmControl = false;    
    controlType = E_CONTROL_TYPE::BACK;

    DrvInfo.pf = prof;
    DrvInfo.startPose = robotPose;
    DrvInfo.targetPoint = targetPoint;

    setPointLists(robotPose, targetPoint, 0.05);
    lineCoeff = utils::math::calculateLineEquation(tPoint(robotPose.x, robotPose.y), targetPoint);
#if defined (DEBUG_ROS_PUB) && (DEBUG_ROS_PUB == 1)
    DEBUG_PUB.publishCleanLine1(robotPose, targetPoint);
#endif
    setState(E_STATE::RUNNING);
}

/**
 * @brief 회전 제어기
 * 
 * @param robotPose 
 * @param targetAngle 
 * @param prof 
 * @param dir 
 */
void CMotionController::startRotation(const tPose& robotPose, double targetAngle, tProfile prof, E_ROTATE_DIR dir)
{
    ceblog(LOG_LV_NECESSARY, YELLOW," [ ROTATE ] 회전 제어");
    rotateDir = dir;
    pwmControl = false;    
    controlType = E_CONTROL_TYPE::ROTATE;

    DrvInfo.pf = prof;
    DrvInfo.startPose = robotPose;
    DrvInfo.targetAngle = targetAngle;
    DrvInfo.pf.isStopAtTarget = false;

    setState(E_STATE::RUNNING);
}

/**
 * @brief 모터 pwm 제어기
 * 
 * @param robotPose 
 * @param direction 
 * @param lspeed 
 * @param rspeed 
 * @param bspeed 
 * @param duty 
 * @param pid 
 * @param prof 
 */
void CMotionController::startDriveWheelPwm(const tPose& robotPose, int direction, int lspeed, int rspeed, int bspeed, int duty, bool pid, tProfile prof)
{    
    pwmControl = true;
    pwmInfo.direction = direction;
    pwmInfo.lspeed = lspeed;
    pwmInfo.rspeed = rspeed;
    pwmInfo.bspeed = bspeed;
    pwmInfo.duty = duty;
    pwmInfo.pid = pid;

    if(pwmInfo.lspeed != 0 || pwmInfo.rspeed != 0)
    {
        if(pwmInfo.lspeed == pwmInfo.rspeed)
        {
            controlType = E_CONTROL_TYPE::LINEAR;
            if(duty != 0)
            {
                DrvInfo.startPose = ServiceData.localiz.getSysPose();
                DrvInfo.previousPoint = tPoint(ServiceData.localiz.getSysPose().x, ServiceData.localiz.getSysPose().y);
                DrvInfo.movementDistance = 0.0;
                if(pwmInfo.lspeed < 0)  DrvInfo.targetDistance = (double)(-duty)/1000;
                else                    DrvInfo.targetDistance = (double)duty/1000;
            }
            else
            {
                DrvInfo.targetDistance = 0;
            }
            ceblog(LOG_LV_NECESSARY, BOLDGREEN,"MOVE FORWARD/BACK TARGET DISTANCE : " << DrvInfo.targetDistance << "LSPEED : " << pwmInfo.lspeed << " RSPEED : " << pwmInfo.rspeed);
        }
        else
        {
            if((pwmInfo.lspeed > 0 && pwmInfo.rspeed < 0) || (pwmInfo.lspeed < 0 && pwmInfo.rspeed > 0))    controlType = E_CONTROL_TYPE::ROTATE;

            if(duty != 0)
            {
                DrvInfo.startPose = ServiceData.localiz.getSysPose();
                DrvInfo.previousAngle = robotPose.angle;
                
                if(controlType == E_CONTROL_TYPE::ROTATE)
                {
                    if(pwmInfo.lspeed < 0)  DrvInfo.targetAngle = DEG2RAD(double(duty));
                    else                    DrvInfo.targetAngle = DEG2RAD(double(-duty));
                }
                else
                {
                    if(pwmInfo.lspeed >  pwmInfo.rspeed) DrvInfo.targetAngle = DEG2RAD(double(-duty));
                    else                                 DrvInfo.targetAngle = DEG2RAD(double(duty));
                }
            }
            else
            {
                DrvInfo.targetAngle = 0;
            }

            if(controlType == E_CONTROL_TYPE::ROTATE)
            {
                ceblog(LOG_LV_NECESSARY, BOLDGREEN,"MOVE TURN TARGET ANGLE : " << RAD2DEG(DrvInfo.targetAngle) << "LSPEED : " << pwmInfo.lspeed << " RSPEED : " << pwmInfo.rspeed);
            }
        }
    }

    setState(E_STATE::RUNNING);
}

/**
 * @brief  목표지점을 추종하는 데 필요한 points의 리스트를 만들어 주는 함수
 * 
 * @param targetPoint 
 * @param point2pointDist 
 * @return * void 
 */
void CMotionController::setPointLists(tPose startPose, tPoint targetPoint, double point2pointDist)
{
    pointLists.clear();
    lineCoeff.clear();
    int cnt = 1;
    tPoint tempPoint;
    double theta = atan2(targetPoint.y - startPose.y, targetPoint.x - startPose.x);
    double robot2targetDist = utils::math::distanceTwoPoint(startPose, targetPoint);
    double deltaX=point2pointDist*cos(theta);
    double deltaY=point2pointDist*sin(theta);

    while(true)
    {
        if(pointLists.size()!=0)
        {
            if(utils::math::distanceTwoPoint(startPose, targetPoint) <= utils::math::distanceTwoPoint(startPose,  pointLists.back()))
            {
                break;
            }
            else
            {
                tempPoint.x = startPose.x + cnt*deltaX;
                tempPoint.y = startPose.y + cnt*deltaY;
                pointLists.push_back(tempPoint);
            }
        }
        else
        {
            tempPoint.x = startPose.x + cnt*deltaX;
            tempPoint.y = startPose.y + cnt*deltaY;
            pointLists.push_back(tempPoint);
        }
        cnt++;
    }
}

/**
 * @brief point2robotDist 만큼의 거리를 유지하며 point를 업데이트해주는 함수
 * 
 * @param robotPose 
 * @param point2robotDist 
 */
void CMotionController::updatePointLists(tPose robotPose, double point2robotDist)
{
    double curPoint2RobotDist = utils::math::distanceTwoPoint(pointLists.front(), robotPose);
    while(curPoint2RobotDist <= point2robotDist)
    {
        if (pointLists.size() <= 1)
        {
            break;
        }
        pointLists.pop_front();
        curPoint2RobotDist = utils::math::distanceTwoPoint(pointLists.front(), robotPose);
    }
}


/**
 * @brief 구형 정지-대기 제어기, avoiding 때문에 남겨둠 2024.05.23 (by hyjoe)
 * 
 */
void CMotionController::stop(int sleepMs)
{    
    setState(E_STATE::RUNNING);
    controlType = E_CONTROL_TYPE::STOP;
    clearVelocity();
    if(pwmControl)
    {
        pwmControl = false;
        sendMessageDriveOld(pwmInfo.direction,pwmInfo.lspeed,pwmInfo.rspeed,pwmInfo.bspeed,pwmInfo.duty,pwmInfo.pid);
    }
    else
    {
        sendMessageDrvie(combineTwist.v, combineTwist.w);
    }

    stopTime = int(get_system_time()*1000); // 단위 ms
    waitingTime = sleepMs;
    ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "정지대기 시작 시간: " << stopTime << " ms \t\t 대기할 시간:" << waitingTime << " ms");
}

E_STATE CMotionController::getState()
{
    return state;
}

E_CONTROL_TYPE CMotionController::getControlType()
{
    return controlType;
}

E_ANGULAR_PRIOR_CONTROL_STEP CMotionController::getAngularPriorControlType()
{
    return currentStep;
}

tTwist CMotionController::getControlVelocity()
{
    return combineTwist;
}

void CMotionController::threadMotion()
{
    int debugPrint = 0;

    while (bThMotion)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        DEBUG_CTR.isAliveMotionController.set(true);
    }
}

void CMotionController::setState(E_STATE state)
{
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 0)  // 모션 컨트롤러 상태변화 디버그용 : RBT_PLUS TEST 시 비활성하자
    if( this->state != state)
    {
        tPose robotPose = ServiceData.localiz.getPose();
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, "Motion State이 변경 됩니다. ["<<WHITE<<enumToString(this->state)<<BOLDBLACK<<"]\t --> ["<<BOLDCYAN<<enumToString(state)<<BOLDBLACK<<"]");
        eblog(LOG_LV_NECESSARY, "robotPose : " << robotPose.x << "," << robotPose.y << " angle : " << utils::math::rad2deg(robotPose.angle)<<" 도" );
    }
#endif
    this->state = state;
}

void CMotionController::procDirectPwm()
{
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    ServiceData.pwmDriveInfo.desVel = getCalculatedPwmSpeed();
    sendMessageDriveOld(pwmInfo.direction,ServiceData.pwmDriveInfo.desVel.left,ServiceData.pwmDriveInfo.desVel.right,ServiceData.pwmDriveInfo.desVel.back,pwmInfo.duty,pwmInfo.pid);
    ServiceData.pwmDriveInfo.curVel = ServiceData.pwmDriveInfo.desVel;

    if(pwmInfo.direction == 0)
    {
        if(pObstacle->wheel.leftEncoder == 0 && pObstacle->wheel.rightEncoder == 0 && pObstacle->wheel.dummyEncoder == 0)
        {
            setState(E_STATE::NOTHING);
        }
    }
}

void CMotionController::procLinearToPointOnMap(const tPose& robotPose)
{
    ServiceData.motionInfo.desVel = getCalculatedTwist(robotPose);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    combineTwist = ServiceData.motionInfo.desVel;
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CMotionController::procLinearAngularPriorToPointOnMap(const tPose& robotPose)
{
    ServiceData.motionInfo.desVel = getCalculatedAngularPriorTwist(robotPose);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    combineTwist = ServiceData.motionInfo.desVel;
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CMotionController::procLinearToPointOnMapCheckObs(const tPose& robotPose)
{
    ServiceData.motionInfo.desVel = getCalculatedTwistCheckObs(robotPose);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    combineTwist = ServiceData.motionInfo.desVel;
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CMotionController::procBackToPointOnMap(const tPose& robotPose)
{
    ServiceData.motionInfo.desVel = getCalculatedReverseTwist(robotPose);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    combineTwist = ServiceData.motionInfo.desVel;
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CMotionController::procRotation(const tPose& robotPose)
{
    switch (rotateDir)
    {
    case E_ROTATE_DIR::NONE :
        ServiceData.motionInfo.desVel = getCalculatedRotateTwist(robotPose);
        break;
    case E_ROTATE_DIR::CW :
        ServiceData.motionInfo.desVel = getCalculatedCWRotateTwist(robotPose);
        break;
    case E_ROTATE_DIR::CCW :
        ServiceData.motionInfo.desVel = getCalculatedCCWRotateTwist(robotPose);
        break;
    default:
        ServiceData.motionInfo.desVel = getCalculatedRotateTwist(robotPose);
        break;
    }
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    combineTwist = ServiceData.motionInfo.desVel;
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CMotionController::initControlValues()
{
    targetPoint         = tPoint(0,0);
    tempTargetAngle     = 0.0;
    angleError          = 0.0;
    lineDisErr          = 0.0;
    relativePosition    = 0.0;
    disRobot2Goal       = 0.0;
    desiredV            = 0.0;
    desiredW            = 0.0;
    isRobotCrossEndLine = false;
}

void CMotionController::setControlValues(tPose robotPose)
{
    if (controlType==E_CONTROL_TYPE::ROTATE
        || (controlType == E_CONTROL_TYPE::ANGULAR_PRIOR
        && currentStep == E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE))
    {
        setRotateControlValues(robotPose);
    }
    else
    {
        setNormalControlValues(robotPose);
        if (controlType == E_CONTROL_TYPE::BACK)
        {
            if(angleError > 0)  {angleError = angleError - M_PI ;}
            else                {angleError = angleError + M_PI ;}
        }
        errorBuffer.push_front(angleError);
        if (errorBuffer.size() > bufferSize)    {errorBuffer.pop_back();}
        for (const auto& error : errorBuffer)   {angleIError += error;}
    }
}

void CMotionController::setRotateControlValues(tPose robotPose)
{
    initControlValues();
    if (controlType == E_CONTROL_TYPE::ROTATE
        && rotateDir == E_ROTATE_DIR::NONE)
    {
        angleError = DrvInfo.targetAngle - robotPose.angle;
    }
    else if (controlType == E_CONTROL_TYPE::ROTATE
        && rotateDir == E_ROTATE_DIR::CCW)     // 각도 에러 [0,360) 범위만 허용
    {
        angleError = DrvInfo.targetAngle - robotPose.angle;
        while (angleError < 0)          {angleError += 2*M_PI;}
        while (angleError >= 2*M_PI)    {angleError -= 2*M_PI;}
    }
    else if (controlType == E_CONTROL_TYPE::ROTATE
        && rotateDir == E_ROTATE_DIR::CW) // 각도 에러 (-360,0] 범위만 허용
    {
        angleError = DrvInfo.targetAngle - robotPose.angle;
        while (angleError > 0)          {angleError -= 2*M_PI;}
        while (angleError <= -2*M_PI)   {angleError += 2*M_PI;}
    }
    else if (controlType == E_CONTROL_TYPE::ANGULAR_PRIOR
        && currentStep == E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE)
    {
        targetPoint     = pointLists.front();
        tempTargetAngle = atan2(targetPoint.y - robotPose.y, targetPoint.x - robotPose.x);
        angleError      = utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle);
        // fabs(angleError) < 180 deg 로 만들기
        if (fabs(angleError) > DEG2RAD(360))                                {angleError = 0;}
        else if (angleError >  DEG2RAD(180) && angleError <=  DEG2RAD(360)) {angleError = angleError - M_PI;}
        else if (angleError > -DEG2RAD(180) && angleError <=  DEG2RAD(180)) {angleError = angleError;}
        else if (angleError > -DEG2RAD(360) && angleError <= -DEG2RAD(180)) {angleError = angleError + M_PI;}
    }
    else {ceblog((LOG_LV_MOTION || LOG_LV_NECESSARY), BOLDRED,
    "잘못된 제어값 세팅! 사용한 제어기가 올바른지 확인하세요. (조환영 연구원에게 문의)");}
}

void CMotionController::setNormalControlValues(tPose robotPose)
{
    targetPoint      = pointLists.front();
    tempTargetAngle  = atan2(targetPoint.y - robotPose.y, targetPoint.x - robotPose.x); // 목표점을 향한 목표각도 (rad)
    angleError       = utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle);
    lineDisErr       = utils::math::calculateDistanceFromLine(tPoint(robotPose.x,robotPose.y), lineCoeff);
    relativePosition = utils::math::pointRelativeToLine(tPoint(robotPose.x, robotPose.y),
                       tPoint(DrvInfo.startPose.x, DrvInfo.startPose.y), DrvInfo.targetPoint);

    if(relativePosition>0)       {lineDisErr = lineDisErr * (-1);}
    if (lineDisErr > 0.35)       {lineDisErr = 0.35;}
    else if (lineDisErr < -0.35) {lineDisErr = -0.35;}

    disRobot2Goal       = utils::math::distanceTwoPoint(tPoint(robotPose.x, robotPose.y), DrvInfo.targetPoint);
    isRobotCrossEndLine = utils::math::isRobotCrossLine(robotPose, DrvInfo.startPose.convertPoint(), DrvInfo.targetPoint);
}

void CMotionController::procStop()
{
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    if(state == E_STATE::RUNNING)
    {
        if(DrvInfo.bStopEmergency)
        {
            ServiceData.motionInfo.desVel = getCalculatedEmergencyStopTwist();
        }    
        else
        {
            ServiceData.motionInfo.desVel = getCalculatedStopTwist();
        }

        MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
        combineTwist = ServiceData.motionInfo.desVel;
        ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
        
        if(ServiceData.motionInfo.desVel.v == 0 && ServiceData.motionInfo.desVel.w == 0)
        {
            if(ServiceData.motionInfo.curVel.v == 0 && ServiceData.motionInfo.curVel.w == 0)
            {
                if(pObstacle->wheel.leftEncoder== 0 && pObstacle->wheel.rightEncoder == 0 && pObstacle->wheel.dummyEncoder == 0)
                {
                    setState(E_STATE::NOTHING);
                }else{
                    ceblog((LOG_LV_MOTION || LOG_LV_NECESSARY), BOLDRED, "정지를 내렸는데 엔코더 멈추지 않음 LEFT :" << pObstacle->wheel.leftEncoder << " RIGHT :" << pObstacle->wheel.rightEncoder << " BACK : " << pObstacle->wheel.dummyEncoder);
                }
                
            }
        }
    }
}

tPwmSpeed CMotionController::getCalculatedPwmSpeed()
{
    tPwmSpeed curVel = ServiceData.pwmDriveInfo.curVel;
    
    
    if(curVel.left < pwmInfo.lspeed-CONFIG.pwmAccel)            curVel.left += CONFIG.pwmAccel;
    else if(curVel.left > pwmInfo.lspeed+CONFIG.pwmDeccel)      curVel.left -= CONFIG.pwmDeccel;
    else                                                        curVel.left = pwmInfo.lspeed; 

    if(curVel.right < pwmInfo.rspeed-CONFIG.pwmAccel)           curVel.right += CONFIG.pwmAccel;
    else if(curVel.right > pwmInfo.rspeed+CONFIG.pwmDeccel)     curVel.right -= CONFIG.pwmDeccel;
    else                                                        curVel.right = pwmInfo.rspeed; 

    if(curVel.back < pwmInfo.bspeed-CONFIG.pwmAccel)           curVel.back += CONFIG.pwmAccel;
    else if(curVel.back > pwmInfo.bspeed+CONFIG.pwmDeccel)     curVel.back -= CONFIG.pwmDeccel;
    else                                                       curVel.back = pwmInfo.bspeed; 

    if (debugCnt >= 100) // 1000ms에 한번 출력 == 1초에 1번
    {
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, "[ LEFT, RIGHT, BACK : "<<  curVel.left << " , " << curVel.right << " , " << curVel.back <<  " ] ");
        debugCnt = 0;
    }
    debugCnt += 1;

    return curVel;
}
/**
 * @brief 스무스 정지 함수
 * - 선/각속도가 연속적으로 감소하도록 제어
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedStopTwist()
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    initControlValues();
    curVel.v = getStopCmdVel(curVel.v, true);
    curVel.w = getStopCmdVel(curVel.w, false);

    debugPrintInfo(ServiceData.localiz.getPose(), curVel.v, curVel.w);

    return curVel;
}
/**
 * @brief 긴급 정지 함수
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedEmergencyStopTwist()
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    initControlValues();

    curVel.v = 0;
    curVel.w = 0;

    debugPrintInfo(ServiceData.localiz.getPose(), curVel.v, curVel.w);

    return curVel;
}

/**
 * @brief 제자리 회전 함수
 * - Global 좌표계 상의 Desired Angle을 바라보도록 각속도 제어.
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedRotateTwist(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    setControlValues(robotPose);

    desiredW = getAngularDesiredVel();
    if(fabs(angleError)< DEG2RAD(1) || curVel.v != 0)   {curVel.w = 0;}
    else                                                {curVel.w = getAngularCmdVel(curVel.w, desiredW);}

    curVel.v = getStopCmdVel(curVel.v, true);

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
}

/**
 * @brief CCW(반시계 방향) 제자리 회전 함수
 * - Global 좌표계 상의 Desired Angle을 바라보도록 각속도 제어.
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedCCWRotateTwist(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    setControlValues(robotPose);

    desiredW = getAngularDesiredVel();
    if(angleError < DEG2RAD(1) || angleError > DEG2RAD(359) || curVel.v != 0 )  {curVel.w = 0;}
    else                                                                        {curVel.w = getAngularCmdVel(curVel.w, desiredW);}

    curVel.v = getStopCmdVel(curVel.v, true);

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
}

/**
 * @brief CW(시계 방향) 제자리 회전 함수
 * - Global 좌표계 상의 Desired Angle을 바라보도록 각속도 제어.
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedCWRotateTwist(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    setControlValues(robotPose);

    desiredW = getAngularDesiredVel();
    if(angleError > -DEG2RAD(1) || angleError < -DEG2RAD(359) || curVel.v != 0 )    {curVel.w = 0;}
    else                                                                            {curVel.w = getAngularCmdVel(curVel.w, desiredW);}

    curVel.v = getStopCmdVel(curVel.v, true);

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
}

/**
 * @brief 후진 Point 제어 함수
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedReverseTwist(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    updatePointLists(robotPose, CONFIG.motionController_waypoint_dist);
    setControlValues(robotPose);
    
    desiredW = getAngularDesiredVel();
    curVel.w = getAngularCmdVel(curVel.w, angleError);
    desiredV = -getLinearDesiredVel(curVel.w);
    curVel.v = getBackCmdVel(curVel.v, desiredV);

    prevAangleError = angleError;

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
}

/**
 * @brief 장애물 감지시 감속하는 후진 Point 제어 함수
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedReverseTwistCheckObs(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    updatePointLists(robotPose, CONFIG.motionController_waypoint_dist);
    setControlValues(robotPose);
    
    desiredW = getAngularDesiredVel();
    curVel.w = getAngularCmdVel(curVel.w, angleError);
    desiredV = -getLinearDesiredVel(curVel.w);

    // 현재 walltrack 파라미터 사용중, 필요하다면 나중에 분리해야 함
    if(getBackLidarData() < CONFIG.wallTrack_v2_decel_Dis) 
    {
        curVel.v += CONFIG.wallTrack_v2_decel_V;
    }
    else    {curVel.v = getBackCmdVel(curVel.v, desiredV);}

    prevAangleError = angleError;

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
}

/**
 * @brief 장애물 감지시 감속하는 직진 Point 제어 함수
 * 
 * @param robotPose 
 */
tTwist CMotionController::getCalculatedTwistCheckObs(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    updatePointLists(robotPose, CONFIG.motionController_waypoint_dist);
    setControlValues(robotPose);
    
    desiredW = getAngularDesiredVel();
    curVel.w = getAngularCmdVel(curVel.w, desiredW);
    desiredV = getLinearDesiredVel(curVel.w);

    // 현재 walltrack 파라미터 사용중, 필요하다면 나중에 분리해야 함
    if(getFrontLidarData() < CONFIG.wallTrack_v2_decel_Dis || (int)(ServiceData.obstacle.getObstacleData()->tof.knoll.rangeAvg) < (CONFIG.wallTrack_v2_decel_Dis*1000)-250) 
    {
        curVel.v -= CONFIG.wallTrack_v2_decel_V;
    }
    else    {curVel.v = getLinearCmdVel(curVel.v, desiredV);}

    prevAangleError = angleError;

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
 }
/**
 * @brief 각속 우선 제어 함수
 * 
 * @param robotPose 
 * @return tTwist 
 */
tTwist CMotionController::getCalculatedAngularPriorTwist(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);
    /*
        청소 중 라인 패턴 시에는 회전이 발생할 경우 업데이트 하지 말자
        0 : 정지,이동 상태에서는 맵 생성됨/정지&회전 상태에서는 맵이 생성되지 않음
        1 : 로봇 좌표만 생성, 맵은 생성되지 않음 ( 사용 처 : 틸업 혹 기타 경우 )
        2 : 정지, 이동, 정지&회전 모두 맵 생성됨 ( 처음 탐색 시작 시점에서 맵 확장이 필요한 시점, 
        혹은 지도 로딩 시)
    */
    MAP_FILTER_MODE mapFilterMode = ROBOT_CONTROL.slam.getMapFilterMode();
    int mapMode = static_cast<int>(mapFilterMode);

    updatePointLists(robotPose, CONFIG.motionController_waypoint_dist);
    setControlValues(robotPose);

    desiredW = getAngularDesiredVel();
    switch (currentStep)
    {
    case E_ANGULAR_PRIOR_CONTROL_STEP::V_DECEL :
        curVel.v = getStopCmdVel(curVel.v, true);
        if (curVel.v == 0)
        {
            currentStep = E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE;
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "STEP : V_DECEL -> ROTATE");
        }
        break;
    case E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE :
        if (fabs(angleError) < DEG2RAD(3))
        {
            currentStep = E_ANGULAR_PRIOR_CONTROL_STEP::MOVE_FORWARD;
            ceblog(LOG_LV_NECESSARY, BOLDBLUE, "STEP : ROTATE -> MOVE_FORWARD");
        }
        else
        {
            curVel.w = getAngularCmdVel(curVel.w, desiredW);
        }
        break;
    case E_ANGULAR_PRIOR_CONTROL_STEP::MOVE_FORWARD :
        curVel.w = getAngularCmdVel(curVel.w, desiredW);
        desiredV = getLinearDesiredVel(curVel.w);
        curVel.v = getLinearCmdVel(curVel.v, desiredV);
        break;
    }

    prevAangleError = angleError;

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
}

/**
 * @brief 직진 Point 제어 함수
 *
 * @param robotPose
 */

tTwist CMotionController::getCalculatedTwist(tPose robotPose)
{
    tTwist curVel = ServiceData.motionInfo.curVel;
    CPthreadLockGuard lock(mutexPointControl);

    updatePointLists(robotPose, CONFIG.motionController_waypoint_dist);
    setControlValues(robotPose);
    
    desiredW = getAngularDesiredVel();
    curVel.w = getAngularCmdVel(curVel.w, desiredW);
    desiredV = getLinearDesiredVel(curVel.w);
    curVel.v = getLinearCmdVel(curVel.v, desiredV);

    prevAangleError = angleError;

    debugPrintInfo(robotPose, curVel.v, curVel.w);

    return curVel;
}


// V-W 반비례 1차함수 프로파일
double CMotionController::getLinearDesiredVel(double curVelW)
{
    double ret;

    if(fabs(curVelW) < DEG2RAD(5))          {ret = CONFIG.pointControl_max_V;}
    else if(fabs(curVelW) > DEG2RAD(15))    {ret = 0.1;}
    else                                    {ret = -1.1781*fabs(curVelW)+0.4029;}

    return ret;
}

// 각도에러 + 거리에러 고려한 desiredW
double CMotionController::getAngularDesiredVel()
{
    double ret;

    ret = getNormalAngularDesiredVel();

    if (controlType==E_CONTROL_TYPE::ROTATE
        || (controlType == E_CONTROL_TYPE::ANGULAR_PRIOR
        && currentStep == E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE))
    {
        if      (angleError > DrvInfo.pf.desAngVel)     {ret =  DrvInfo.pf.desAngVel;}
        else if (angleError < -DrvInfo.pf.desAngVel)    {ret = -DrvInfo.pf.desAngVel;}
        else                                            {ret =  angleError;}
    }

    return ret;
}

// 각도에러만 봄 (각도에러가 거리에러도 반영해서 계산됨)
double CMotionController::getNormalAngularDesiredVel()
{
    double ret;

    double anglePTerm = angleError                      * CONFIG.pointControl_angle_P_gain;
    double angleITerm = angleIError                     * CONFIG.pointControl_angle_I_gain;
    double angleDTerm = (angleError - prevAangleError)  * CONFIG.pointControl_angle_D_gain;
    ret = anglePTerm + angleITerm + angleDTerm;

    if(disRobot2Goal!=0 && disRobot2Goal<CONFIG.pointControl_decel_dis) {ret = (0.35/CONFIG.pointControl_decel_dis)*disRobot2Goal;}

    return ret;
}

/**
 * @brief cmdVel.w 반환
 * 
 * @param curVelW 
 * @param desiredW 
 * @return double 
 */
double CMotionController::getAngularCmdVel(double curVelW, double desiredW)
{
    double ret = curVelW;

    if(fabs(desiredW) > DEG2RAD(CONFIG.pointControl_max_W))
    {
        if(curVelW > DEG2RAD(CONFIG.pointControl_max_W))       {desiredW = DEG2RAD(CONFIG.pointControl_max_W);}
        else if(curVelW < -DEG2RAD(CONFIG.pointControl_max_W)) {desiredW = -DEG2RAD(CONFIG.pointControl_max_W);}    
    }

    // 각속도 가감속
    if(curVelW < desiredW - DEG2RAD(CONFIG.pointControl_accel_W)*10)       {ret = curVelW + DEG2RAD(CONFIG.pointControl_accel_W)*3;}
    else if(curVelW < desiredW)                                            {ret = curVelW + DEG2RAD(CONFIG.pointControl_accel_W);}
    else if(curVelW > desiredW + DEG2RAD(CONFIG.pointControl_decel_W)*10)  {ret = curVelW - DEG2RAD(CONFIG.pointControl_decel_W)*3;}
    else if(curVelW > desiredW)                                            {ret = curVelW - DEG2RAD(CONFIG.pointControl_decel_W);}
    else                                                                   {ret = desiredW;}
    ret = applyRotateDeadzone(ret, angleError);

    return ret;
}

/**
 * @brief 전진 cmdVel.v 반환
 * 
 * @param curVelV 
 * @param desiredV 
 * @return double 
 */
double CMotionController::getLinearCmdVel(double curVelV, double desiredV)
{
    double ret = curVelV;

    // 선속도 가감속
    if(curVelV > desiredV + CONFIG.pointControl_decel_V )   {ret = curVelV - CONFIG.pointControl_decel_V*2;}
    else if(curVelV > desiredV )                            {ret = curVelV - CONFIG.pointControl_decel_V;}
    else if(curVelV < 0)                                    {ret = curVelV + CONFIG.pointControl_accel_V*3;}
    else                                                    {ret = curVelV + CONFIG.pointControl_accel_V;}

    // 선속도 최대, 최소 범위 (연속성을 위해 현재 속도를 고려, 최소 속도 0으로 제한)
    if(curVelV < 0)
    {
        if(curVelV > CONFIG.pointControl_max_V)      {ret = CONFIG.pointControl_max_V;}
    }
    else
    {
        if(curVelV > CONFIG.pointControl_max_V)      {ret = CONFIG.pointControl_max_V;}
        else if(curVelV < 0)    {ret = 0;}
    }

    // 모터 안움직이는 상황이면 V 강제로 죽임 => W 명령만 내려서 3바퀴 모두 돌도록 하기 위해서
    if (isEncoderStop() && desiredV != 0)
    {
        ceblog(LOG_LV_NECESSARY | LOG_LV_MOTION, BOLDRED, " 세 바퀴 모두 움직이지 않아 선속도 V=0, W 명령만 내려 세바퀴 모두 돌도록 제어! ");
        ret= 0;
    }

    return ret;
}

/**
 * @brief 후진 cmdVel.v 반환
 * 
 * @param curVelV 
 * @param desiredV 
 * @return double 
 */
double CMotionController::getBackCmdVel(double curVelV, double desiredV)
{
    double ret = curVelV;

    // 선속도 가감속
    if(curVelV < desiredV - CONFIG.pointControlReverse_decel_V )    {ret = curVelV + CONFIG.pointControlReverse_decel_V*2;}
    else if(curVelV < desiredV )                                    {ret = curVelV + CONFIG.pointControlReverse_decel_V;}
    else if(curVelV > 0)                                            {ret = curVelV - CONFIG.pointControl_accel_V*3;}
    else                                                            {ret = curVelV - CONFIG.pointControlReverse_accel_V;}

    // 선속도 최대, 최소 범위 (연속성을 위해 현재 속도를 고려, 최소 속도 0으로 제한)
    if(curVelV > 0)
    {
        if(curVelV < -CONFIG.pointControl_max_V)     {ret = -CONFIG.pointControl_max_V;}
    }
    else
    {
        if(curVelV < -CONFIG.pointControl_max_V)     {ret = -CONFIG.pointControl_max_V;}
        else if(curVelV > 0)    {ret = 0;}
    }

    // 모터 안움직이는 상황이면 V 강제로 죽임 => W 명령만 내려서 3바퀴 모두 돌도록 하기 위해서
    if (isEncoderStop() && desiredV != 0)
    {
        ceblog(LOG_LV_NECESSARY | LOG_LV_MOTION, BOLDRED, " 세 바퀴 모두 움직이지 않아 선속도 V=0, W 명령만 내려 세바퀴 모두 돌도록 제어! ");
        ret= 0;
    }

    return ret;
}

/**
 * @brief 정지 cmdVel 반환 (linear = true : 선속 | false : 각속)
 * 
 * @param curVel 
 * @param linear 
 * @return double 
 */
double CMotionController::getStopCmdVel(double curVel, bool linear)
{
    double ret = curVel;
    double decel;
    if(linear)  {decel = CONFIG.stopControl_decel_V;}
    else        {decel = CONFIG.stopControl_decel_W;}

    if     (curVel >  decel*10)   {ret = curVel - decel*3;}
    else if(curVel >  decel*5)    {ret = curVel - decel*2;}
    else if(curVel >  decel)      {ret = curVel - decel;}
    else if(curVel < -decel*10)   {ret = curVel + decel*3;}
    else if(curVel < -decel*5)    {ret = curVel + decel*2;}
    else if(curVel < -decel)      {ret = curVel + decel;}
    else                          {ret = 0;}

    return ret;
}

/**
 * @brief 전방 라이다 데이터 get
 * 
 * @return double 
 */
double CMotionController::getFrontLidarData()
{
    double ret;
    int lidarAngleRange = 15;
    double lidarSum = 0;
    int lidarIdx = 0;
    int lidarCnt=0;
    std::list<double> lidarList;
    double rightLidarDist;
    double leftLidarDist;
	while(1)
    {
        leftLidarDist = ServiceData.localiz.getLidarDist(lidarIdx);
        if ((leftLidarDist != 0) && !std::isinf(leftLidarDist) && !std::isnan(leftLidarDist)) // 0, inf, NaN는 합산하지 않는다.
        {
            lidarList.emplace_back(leftLidarDist);
            lidarCnt++;
        }

        rightLidarDist  = ServiceData.localiz.getLidarDist(359 - lidarIdx);
        if ((rightLidarDist != 0) && !std::isinf(rightLidarDist) && !std::isnan(rightLidarDist)) // 0, inf, NaN는 합산하지 않는다.
        {
            lidarList.emplace_front(rightLidarDist);
            lidarCnt++;
        }

        if(lidarIdx >= lidarAngleRange)
        {
            break;
        }
        lidarIdx++;
    }

    for(double lidarValue : lidarList)
    {
        lidarSum += lidarValue;
    }

    ret = lidarSum/lidarCnt;
    return ret;
}

/**
 * @brief 후방 라이다 데이터 get
 * 
 * @return double 
 */
double CMotionController::getBackLidarData()
{
    double ret;
    int lidarAngleRange = 15;
    double lidarSum = 0;
    int lidarIdx = 0;
    int lidarCnt=0;
    std::list<double> lidarList;
    double rightLidarDist;
    double leftLidarDist;
	while(1)
    {
        leftLidarDist  = ServiceData.localiz.getLidarDist(180 - lidarIdx);
        if ((leftLidarDist != 0) && !std::isinf(leftLidarDist) && !std::isnan(leftLidarDist)) // 0, inf, NaN는 합산하지 않는다.
        {
            lidarList.emplace_front(leftLidarDist);
            lidarCnt++;
        }

        rightLidarDist = ServiceData.localiz.getLidarDist(180 + lidarIdx);
        if ((rightLidarDist != 0) && !std::isinf(rightLidarDist) && !std::isnan(rightLidarDist)) // 0, inf, NaN는 합산하지 않는다.
        {
            lidarList.emplace_back(rightLidarDist);
            lidarCnt++;
        }

        if(lidarIdx >= lidarAngleRange)
        {
            break;
        }
        lidarIdx++;
    }

    for(double lidarValue : lidarList)
    {
        lidarSum += lidarValue;
    }

    ret = lidarSum/lidarCnt;
    return ret;
}

bool CMotionController::isRunning()
{    
    return (getState() == E_STATE::NOTHING) ? false : true;
}

/**
 * @brief 엔코더 값이 0으로 유지되는지 판단하는 함수. (5초동안 모니터링 후 변화 없으면 true 반환)
 * 
 * @return true 
 * @return false 
 */
bool CMotionController::isEncoderStop()
{
    bool ret = false;

    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    if (pObstacle->wheel.leftEncoder == 0 && pObstacle->wheel.rightEncoder == 0 && pObstacle->wheel.dummyEncoder == 0)  {encCnt += 1;}
    else                                                                                                                {encCnt = 0;}

    if (encCnt >= 500)
    {
#if 1 //현재 습식에서 이 부분으로 인해서 주행 중 로봇이 멈추는 경우가 발생
      // (일단 막음-환영씨 펌웨쪽과 더 확인하세요)
        isErrorEncoder = true;
#else
        ret = true;
#endif
    }
    else
    {
        isErrorEncoder = false;
    }
 
    return ret;
}

/**
 * @brief 로봇 위치와 타겟 좌표의 거리 에러가 마진(distanceMargin) 내에 들어왔는지 판단
 * 
 * @param robotPose 
 * @param target 
 * @param distanceMargin 
 * @return true 
 * @return false 
 */
bool CMotionController::isNearTargetPose(const tPose &robotPose, 
    tPoint target, double distanceMargin)
{
    double d = utils::math::distanceTwoPoint(target, robotPose);
    bool ret = (d < distanceMargin) ? true : false;

    return ret;
}

/**
 * @brief 로봇 헤딩과 타겟 각도의 각도 에러가 마진(marginRad) 내에 들어왔는지 판단
 * 
 * @param robotPose 
 * @param targetRad 
 * @param marginRad 
 * @return true 
 * @return false 
 */
bool CMotionController::isNearTargetRad(const tPose& robotPose, double targetRad, double marginRad)
{
    double error = robotPose.angle - targetRad;
    error = fmod(error,2*M_PI);
    if(error < 0) error += 2*M_PI;
    error = std::min(error, 2*M_PI - error);

    return fabs(error) < fabs(marginRad);
}

/**
 * @brief 로봇이 목표지점을 넘어갔다고 판단되는 경우.
 * 1 방식. 목표점을 지나고, 시작점과 목표점을 잇는 직선에 법선을 구함.
 * 해당 법선을 기준으로
 * 같은 공간에 있으면 목표지점을 안넘음.
 * 다른 공간에 있으면 목표지점을 넘음으로 판단.
 * 
 * 2 방식. 빠른 구현을 위해 단순하게 시작지점과 목표지점의 거리를 기준으로 삼음.
 * 
 * @param robotPose
 * @param targetPoint
 * @return true 
 * @return false 
 */
bool CMotionController::isOverTargetPoint(const tPose &robotPose, tPose startPose)
{
    
    return (normalLineFunction(startPose.x, startPose.y)*normalLineFunction(robotPose.x, robotPose.y))<0 ? true : false;
}

bool CMotionController::isOverTargetPoint(const tPose &robotPose, tPoint startPoint, tPoint targetPoint)
{
    double a = (targetPoint.x-startPoint.x)*(targetPoint.x-startPoint.x)+(targetPoint.y-startPoint.y)*(DrvInfo.targetPoint.y-startPoint.y);
    double b = (targetPoint.x-startPoint.x)*(targetPoint.x-robotPose.x)+(targetPoint.y-startPoint.y)*(DrvInfo.targetPoint.y-robotPose.y);

    bool ret = (a*b) < 0 ? true : false;
    if (ret)
    {
        ceblog(LOG_LV_NECESSARY, BOLDGREEN, "startPoint " << startPoint.x << " , " << startPoint.y 
            << " targetPoint " << targetPoint.x << " , " << targetPoint.y 
            << " 로봇위치 : " << robotPose.x << " ," << robotPose.y 
            << " ,a : "<<a<<" ,b : "<<b);
    }
    return ret;
}

void CMotionController::sendMessageDrvie(double linearVelocity, double angularVelocity)
{
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_CONTROL_WHEEL;
    tMsgCtrWheel msg = {};
    msg.type = E_DRIVE_WHEEL_TYPE::VELOCITY;
    msg.dynamic.seq = msgSeq++;
    msg.dynamic.linearVelocity = (s32)(linearVelocity*1000.0);
    msg.dynamic.angularVelocity = (s32)(angularVelocity*1000.0);
    msg.dynamic.radius = 150;
    SEND_MESSAGE(message_t(type, msg));
}

void CMotionController::sendMessageDriveOld(int direction, int lspeed, int rspeed, int bspeed, int duty, bool pid)
{
    E_MESSAGE_TYPE type = E_MESSAGE_TYPE_CONTROL_WHEEL;
    tMsgCtrWheel msg = {};
    msg.type = E_DRIVE_WHEEL_TYPE::PWM;
    msg.pwm.seq = msgSeq++;
    msg.pwm.direction = direction;
    msg.pwm.L_Speed = lspeed;
    msg.pwm.R_Speed = rspeed;
    msg.pwm.B_Speed = bspeed;
    msg.pwm.duty = duty;
    msg.pwm.bHeadingPid = pid;
    SEND_MESSAGE(message_t(type, msg));
}

double CMotionController::normalLineFunction(double x, double y)
{
    return (DrvInfo.targetPoint.x-DrvInfo.startPose.x)*(DrvInfo.targetPoint.x-x)+(DrvInfo.targetPoint.y-DrvInfo.startPose.y)*(DrvInfo.targetPoint.y-y);
}

double CMotionController::applyRotateDeadzone(double curVelw, double angleError)
{
    double ret = curVelw;

    if ((controlType==E_CONTROL_TYPE::ROTATE
        && rotateDir==E_ROTATE_DIR::NONE)
        || (controlType == E_CONTROL_TYPE::ANGULAR_PRIOR
        && currentStep == E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE))
    {
        if (curVelw  <  DEG2RAD(CONFIG.pointControlRotate_deadzone_w)
            && curVelw > 0 && fabs(angleError) < DEG2RAD(CONFIG.pointControlRotate_deadzone_w))
        {
            ret =  DEG2RAD(CONFIG.pointControlRotate_deadzone_w);
        }
        if (curVelw > -DEG2RAD(CONFIG.pointControlRotate_deadzone_w)
            && curVelw < 0 && fabs(angleError) < DEG2RAD(CONFIG.pointControlRotate_deadzone_w))
        {
            ret = -DEG2RAD(CONFIG.pointControlRotate_deadzone_w);
        }
    }
    else if (controlType == E_CONTROL_TYPE::ROTATE && rotateDir == E_ROTATE_DIR::CCW)
    {
        if (curVelw  <  DEG2RAD(CONFIG.pointControlRotate_deadzone_w)
            && curVelw > 0 && angleError < DEG2RAD(CONFIG.pointControlRotate_deadzone_w))
        {
            ret =  DEG2RAD(CONFIG.pointControlRotate_deadzone_w);
        }
    }
    else if (controlType == E_CONTROL_TYPE::ROTATE && rotateDir == E_ROTATE_DIR::CW)
    {
        if (curVelw > -DEG2RAD(CONFIG.pointControlRotate_deadzone_w)
            && curVelw < 0 && angleError > -DEG2RAD(CONFIG.pointControlRotate_deadzone_w))
        {
            ret = -DEG2RAD(CONFIG.pointControlRotate_deadzone_w);
        }
    }
    return ret;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool CMotionController::isRotate() 
{   
    bool bRotate;
    if ( isRunning())
    {
        switch (controlType)
        {
            case E_CONTROL_TYPE::ROTATE:
                ceblog(LOG_LV_NECESSARY, BOLDBLUE, "회전(1) 중");
                bRotate = true;
                break;
            
            case E_CONTROL_TYPE::ANGULAR_PRIOR:
                {
                    if (currentStep == E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE)
                    {
                        ceblog(LOG_LV_NECESSARY, BOLDBLUE, "회전(2) 중");
                        bRotate = true;
                    }
                    else
                    {
                        bRotate = false;
                    }
                }
                break;

            default:
                bRotate = false;
                break;
        }


        if (controlType == E_CONTROL_TYPE::ROTATE)
        {
            bRotate = true;
        }
        else
        {
            bRotate = false;
        }
    } 
    else 
    {
        bRotate = false;
    }

    return bRotate;
}

void CMotionController::debugPrintInfo(tPose robotPose, double curVelv, double curVelw)
{
    if (controlType==E_CONTROL_TYPE::LINEAR
        ||(controlType==E_CONTROL_TYPE::ANGULAR_PRIOR
        && currentStep == E_ANGULAR_PRIOR_CONTROL_STEP::MOVE_FORWARD)
        ||controlType==E_CONTROL_TYPE::BACK)
    {

# if !MOTION_CONTROLLER_DEBUG_LOG_ON
        if (debugCnt >= 100) // 1000ms에 한번 출력 == 1초에 1번
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " V , W : " << curVelv << " , " << RAD2DEG(curVelw)
            << "  |  현재좌표 : " << robotPose.x << " , " << robotPose.y
            << "  |  각도에러, 거리에러(정규화) : "<<  RAD2DEG(angleError) << " , " << RAD2DEG(nomalizedLineDisErr)
            << "  |  타겟좌표 : " << DrvInfo.targetPoint.x << " , " << DrvInfo.targetPoint.y
            << "  |  임시타겟좌표 : " << targetPoint.x << " , " << targetPoint.y
            << "  |  타겟헤딩 : " << RAD2DEG(tempTargetAngle)
            << "  |  로봇헤딩 : " << RAD2DEG(robotPose.angle)
            << "  |  세바헤딩 : " << RAD2DEG(ServiceData.localiz.getSysPose().angle)
            << "  |  제어타입 : " << enumToString(controlType));
            debugCnt = 0;
        }
        debugCnt += 1;
#elif MOTION_CONTROLLER_DEBUG_LOG_ON
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " V , W : " << curVelv << " , " << RAD2DEG(curVelw)
            << "  |  현재좌표 : " << robotPose.x << " , " << robotPose.y
            << "  |  각도에러, 거리에러(정규화) : "<<  RAD2DEG(angleError) << " , " << RAD2DEG(nomalizedLineDisErr)
            << "  |  타겟좌표 : " << DrvInfo.targetPoint.x << " , " << DrvInfo.targetPoint.y
            << "  |  임시타겟좌표 : " << targetPoint.x << " , " << targetPoint.y
            << "  |  타겟헤딩 : " << RAD2DEG(tempTargetAngle)
            << "  |  로봇헤딩 : " << RAD2DEG(robotPose.angle)
            << "  |  제어타입 : " << enumToString(controlType));
#endif
    }
    else if (controlType==E_CONTROL_TYPE::ROTATE
        || (controlType == E_CONTROL_TYPE::ANGULAR_PRIOR
        && currentStep == E_ANGULAR_PRIOR_CONTROL_STEP::ROTATE))
    {
# if !MOTION_CONTROLLER_DEBUG_LOG_ON
        if (debugCnt >= 100) // 1000ms에 한번 출력 == 1초에 1번
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " V , W : " << curVelv << " , " << RAD2DEG(curVelw)
            << "  |  현재좌표 : " << robotPose.x << " , " << robotPose.y
            << "  |  각도에러 : " << RAD2DEG(angleError)
            << "  |  타겟헤딩 : " << RAD2DEG(tempTargetAngle)
            << "  |  요구각속 : " << RAD2DEG(desiredW)
            << "  |  로봇헤딩 : " << RAD2DEG(robotPose.angle)
            << "  |  세바헤딩 : " << RAD2DEG(ServiceData.localiz.getSysPose().angle)
            << "  |  제어타입 : " << enumToString(controlType));
            debugCnt = 0;
        }
        debugCnt += 1;
#elif MOTION_CONTROLLER_DEBUG_LOG_ON
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " V , W : " << curVelv << " , " << RAD2DEG(curVelw)
        << "  |  현재좌표 : " << robotPose.x << " , " << robotPose.y
        << "  |  각도에러 : " << RAD2DEG(angleError)
        << "  |  타겟헤딩 : " << RAD2DEG(tempTargetAngle)
        << "  |  요구각속 : " << RAD2DEG(desiredW)
        << "  |  로봇헤딩 : " << RAD2DEG(robotPose.angle)
        << "  |  제어타입 : " << enumToString(controlType));
#endif
    }
    else if (controlType==E_CONTROL_TYPE::STOP)
    {
# if !MOTION_CONTROLLER_DEBUG_LOG_ON
        if (debugCnt >= 100) // 1000ms에 한번 출력 == 1초에 1번
        {
            ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " V , W : " << curVelv << " , " << RAD2DEG(curVelw)
            << "  |  현재좌표 : " << robotPose.x << " , " << robotPose.y
            << "  |  타겟헤딩 : " << RAD2DEG(DrvInfo.targetAngle)
            << "  |  로봇헤딩 : " << RAD2DEG(robotPose.angle)
            << "  |  제어타입 : " << enumToString(controlType));
            debugCnt = 0;
        }
        debugCnt += 1;
#elif MOTION_CONTROLLER_DEBUG_LOG_ON
        ceblog(LOG_LV_NECESSARY, BOLDYELLOW, " V , W : " << curVelv << " , " << RAD2DEG(curVelw)
        << "  |  현재좌표 : " << robotPose.x << " , " << robotPose.y
        << "  |  타겟헤딩 : " << RAD2DEG(DrvInfo.targetAngle)
        << "  |  로봇헤딩 : " << RAD2DEG(robotPose.angle)
        << "  |  제어타입 : " << enumToString(controlType));
#endif
    }
}

/**
 * @brief motionController Procedure 2024.05.24
 * 
 * @param robotPose 
 */
void CMotionController::proc()
{
    tPose robotPose = ServiceData.localiz.getPose();
    switch (controlType)
    {
    case E_CONTROL_TYPE::STOP:
        procStop();
        break;
    case E_CONTROL_TYPE::LINEAR:
        procLinearToPointOnMap(robotPose);
        break;
    case E_CONTROL_TYPE::ANGULAR_PRIOR:
        procLinearAngularPriorToPointOnMap(robotPose);
        break;
    case E_CONTROL_TYPE::BACK:
        procBackToPointOnMap(robotPose);
        break;
    case E_CONTROL_TYPE::ROTATE:
        procRotation(robotPose);
        break;
    default:
        break;
    }

    if(pwmControl)
    {
        sendMessageDriveOld(pwmInfo.direction,pwmInfo.lspeed,pwmInfo.rspeed,pwmInfo.bspeed,pwmInfo.duty,pwmInfo.pid);
        ceblog(LOG_LV_NECESSARY, BOLDGREEN,"SEND MESSAGE OLD DRIVE WHEEL  DIR : " << (int)pwmInfo.direction << " LSPEED : " << pwmInfo.lspeed << " RSPEED : " << pwmInfo.rspeed << " BSPEED : " << pwmInfo.bspeed << " DUTY : " << pwmInfo.duty);
    }
}

bool CMotionController::isPwmTargetDistanceArrived(tPose robotPose)
{
    bool ret = false;
    bool isFront = DrvInfo.targetDistance >= 0 ? true : false;
    double deltaDistance = utils::math::distanceTwoPoint(DrvInfo.previousPoint, robotPose);

    DrvInfo.movementDistance += isFront ? deltaDistance : deltaDistance*(-1);
    double errDistance = DrvInfo.targetDistance - DrvInfo.movementDistance;

    if(DrvInfo.targetDistance != 0 && DrvInfo.movementDistance != 0)
    {
        if(isFront && errDistance < 0.01) ret = true;
        else if(!isFront && errDistance > -0.01) ret = true;
    }

    return ret;
}
