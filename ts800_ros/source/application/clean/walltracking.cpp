#include "walltracking.h"
#include "eblog.h"
#include "utils.h"
#include "robotmap.h"
#include "systemTool.h"
#include <cmath>
#include "motionPlanner/motionPlanner.h"
#include <sys/time.h>
#include "rosPublisher.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CWalltracking::CWalltracking()
{
    CStopWatch __debug_sw;
    eblog(LOG_LV,  "");
    preSideTof  = 0;
    preFrontTof = 0;
    preFrontIR  = 0;
    debugCnt = 0;

    isOppBack = false;
    isBack    = false;
	isWallBump=false;
	isOppBump=false;
	isFrontObs=false;
	rotateDir=1;
    isOutOfArea = false;
    obsTime = 0.0;

    TIME_CHECK_END(__debug_sw.getTime());
}

CWalltracking::~CWalltracking()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}



/**
 * @brief 일반 벽타기 proc
 * 
 * @param robotPose 
 * @param direction 
 */
void CWalltracking::procWalltracking(tPose robotPose, E_WALLTRACK_DIR direction)
{
    ServiceData.motionInfo.desVel = runWallTrackPattern(direction, robotPose);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

/**
 * @brief 영역 벽타기 proc
 * 
 * @param robotPose 
 * @param direction 
 * @param path 
 */
void CWalltracking::procAreaWalltracking(tPose robotPose, E_WALLTRACK_DIR direction, tPoint startPoint, tPoint targetPoint, bool update)
{
    ServiceData.motionInfo.desVel = runAreaWallTrackPattern(direction, robotPose,startPoint,targetPoint,update);
    MOTION.sendMessageDrvie(ServiceData.motionInfo.desVel.v,ServiceData.motionInfo.desVel.w);
    ServiceData.motionInfo.curVel = ServiceData.motionInfo.desVel;
}

void CWalltracking::startAccumulateAngle(tPose robotPose)
{
    tempAngle = robotPose.angle;
    accumulateAngle = 0;
}

double CWalltracking::getAccumulateAngle(tPose robotPose)
{
   double diffAngle =  utils::math::rad2deg(robotPose.angle)- utils::math::rad2deg(tempAngle);
   
   if ( diffAngle < -180.0) diffAngle += 360;
   if ( diffAngle > 180.0) diffAngle -= 360;

   accumulateAngle += diffAngle;
   tempAngle = robotPose.angle;
   return accumulateAngle;
}

void CWalltracking::setSensorData(E_WALLTRACK_DIR direction)
{
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();

    RSF_OBSTACLE_MASK lidar = pObstacle->lidar.obstacle;
    tCliffActionState actionState = ServiceData.obstacle.cliff.getActionState();
    

    frontTof   =(int)(pObstacle->tof.knoll.rangeAvg);
    isFrontObs = (bool)pObstacle->front.obstacle.value;

    if(direction == E_WALLTRACK_DIR::RIGHT)
    {
        sideTof    = (int)(pObstacle->tof.rightwall.rangeAvg);
        cliffTof   = (int)(pObstacle->tof.rcliff.rangeAvg);

        bool bWallBump = (bool)pObstacle->bumper.b.fright_side;
        bool bWallCliff = actionState.bRightDetect;        
        if (bWallBump || bWallCliff){
            isWallBump = true;
        }
        else{
            isWallBump = false;
        }

        bool bOppBump  = (bool)pObstacle->bumper.b.fleft_side;
        bool bOppCliff  = actionState.bLeftDetect;
        if (bOppBump || bOppCliff){
            isOppBump = true;
        }
        else{
            isOppBump = false;
        }


        isLidarObs = (bool)(pObstacle->lidar.obstacle.value & 0xFE);
        rotateDir  = 1;
    }
    else
    {
        sideTof = (int)(pObstacle->tof.leftwall.rangeAvg);
        cliffTof = (int)(pObstacle->tof.lcliff.rangeAvg);
        
        bool bWallBump = (bool)pObstacle->bumper.b.fleft_side;
        bool bWallCliff = actionState.bLeftDetect;
        if (bWallBump || bWallCliff){
            isWallBump = true;
        }
        else{
            isWallBump = false;
        }

        bool bOppBump  = (bool)pObstacle->bumper.b.fright_side;
        bool bOppCliff  = actionState.bRightDetect;
        if (bOppBump || bOppCliff){
            isOppBump = true;
        }
        else{
            isOppBump = false;
        }

        isLidarObs = (bool)(pObstacle->lidar.obstacle.value & 0x7F);
        rotateDir = -1;
    }

    // 센서값 LPF
    sideTof  = 0.20 * sideTof  + (1 - 0.20) * preSideTof  ; // SideToF  현재 데이터의 20% 반영
    frontTof = 0.15 * frontTof + (1 - 0.15) * preFrontTof ; // FrontToF 현재 데이터의 15% 반영

    preSideTof  = sideTof;
    preFrontTof = frontTof;
}

/**
 * @brief 벽타기 정지 플래그 반환
 * 
 * @return true 
 * @return false 
 */
bool CWalltracking::isObsDetectForStop() 
{
    bool ret = false;
    if(isWallBump){
        ceblog(LOG_LV_MOTION, BOLDYELLOW, " 범퍼 장애물 감지 SIDE TOF : " << sideTof);
    }

    if((isWallBump && sideTof > 150) || isFrontObs || isLidarObs || frontTof <= 5)
    {
        ceblog(LOG_LV_MOTION, BOLDYELLOW, " 전방 장애물 감지 ");
        ret=true;
    }
    return ret;
}


/**
 * @brief 벽타기 감속 플래그 반환
 * 
 * @return true 
 * @return false 
 */
bool CWalltracking::isObsDetectForTurning() 
{
    bool ret = false;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    if(pObstacle->front.approach.value || isLidarObs || frontTof < (CONFIG.wallTrack_v2_decel_Dis*1000)-250)  {ret = true;}
    return ret;
}
/**
 * @brief 벽타기 감속 플래그 반환
 * 
 * @return true 
 * @return false 
 */
bool CWalltracking::isObsDetectForSlowdown() 
{
    bool ret = false;
    RSU_OBSTACLE_DATA *pObstacle = ServiceData.obstacle.getObstacleData();
    if(pObstacle->front.approach.value || isLidarObs || frontTof < (CONFIG.wallTrack_v2_decel_Dis*1000)-250)  {ret = true;}
    return ret;
}

/**
 * @brief 현재 위치 기준 타겟 좌표를 추종하기 위해 waypoint 만큼 떨어진 목표점 생성 
 * 
 * @param robotPose 
 * @param targetPoint 
 * @param waypoint 
 * @return tPoint 
 */
tPoint CWalltracking::getCalculatedTargetPoint(tPose robotPose, tPoint targetPoint, double waypoint)
{
    tPoint result;

    double AB_distance = utils::math::distanceTwoPoint(robotPose, targetPoint);
    double ratio = waypoint / AB_distance;
    
    result.x = robotPose.x + ratio * (targetPoint.x - robotPose.x);
    result.y = robotPose.y + ratio * (targetPoint.y - robotPose.y);

    return result;
}

/**
 * @brief 일반 벽타기 실행 함수
 * 
 * @param direction 
 * @param robotPose 
 * @return tTwist 
 */
tTwist CWalltracking::runWallTrackPattern(E_WALLTRACK_DIR direction, tPose robotPose)
{
    CStopWatch __debug_sw;    
    tTwist curVel = ServiceData.motionInfo.curVel;

    setSensorData(direction);

    if(isOppBack || isBack || isRotate) //회전일때만
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
    else
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);

    if(bAvoiding)
    {
        if(isRotate && (isOppBump || isWallBump)) {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::OPP_BACK, CONFIG.wallTrack_v2_oppback_time);}
        else                                      {curVel = driveEscapeWall(robotPose, curVel, direction, isOppBack, isBack, isRotate);}
    } 
    else
    {
        if(isOppBump)                           {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::OPP_BACK, CONFIG.wallTrack_v2_oppback_time);}
        else if(isWallBump || (isFrontObs || isLidarObs || frontTof <= 5 ))
        {
            if(isObsDetectForStop())            {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::OPP_BACK, CONFIG.wallTrack_v2_oppback_time);}
            else                                {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::BACK, CONFIG.wallTrack_v2_back_time);}
        }
        else
        {
            curVel = driveWallTrack(curVel, robotPose, direction);
        }
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    if (++debugCnt >= 100) {
        ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDYELLOW, " 벽타기 제어 | 현재 좌표 : ( " << robotPose.x << " , " << robotPose.y << " )");
        debugCnt = 0;
    }
    return curVel;
}

/**
 * @brief 영역 벽타기 실행 함수
 * 
 * @param direction 
 * @param robotPose 
 * @param path 
 * @return tTwist 
 */
tTwist CWalltracking::runAreaWallTrackPattern(E_WALLTRACK_DIR direction, tPose robotPose, tPoint startPoint, tPoint targetPoint , bool update)
{
    CStopWatch __debug_sw;    
    tTwist curVel = ServiceData.motionInfo.curVel;
    
    setSensorData(direction);

    if(update)
    {
        areaControlStep = E_AREA_WALLTRACK_CONTROL_STEP::V_DECEL;
        ceblog(LOG_LV_NECESSARY, BLUE, "STEP INIT : V_DECEL");
    }

    if (isRotate) //회전일때만
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_REMOVE_MAP);
    else
        ROBOT_CONTROL.slam.setMapFilterMode(MAP_FILTER_MODE::ROBOT_MOVE_DEFAULT);

    if(bAvoiding)
    {
        if(isRotate && (isOppBump || isWallBump))   {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::OPP_BACK, CONFIG.wallTrack_v2_oppback_time);}
        else                                        {curVel = driveEscapeWall(robotPose, curVel, direction, isOppBack, isBack, isRotate);}
    } 
    else
    {
        if(isOppBump)                                   {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::OPP_BACK, CONFIG.wallTrack_v2_oppback_time);}
        else if(isWallBump || ( isLidarObs || isFrontObs || frontTof <= 5 ))
        {
            if(isObsDetectForStop())                    {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::OPP_BACK, CONFIG.wallTrack_v2_oppback_time);}
            else                                        {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::BACK, CONFIG.wallTrack_v2_back_time);}
        }
        else
        {
            curVel = driveAreaLineTrack(curVel, robotPose,startPoint,targetPoint);
        }
    }

    if (++debugCnt >= 100) {
        ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDYELLOW, "영역 벽타기 (모션 제어) | 현재 좌표 : ( " << robotPose.x << " , " << robotPose.y << " ) | 타겟 좌표 : ( " << targetPoint.x << " , " << targetPoint.y << " )");
        debugCnt = 0;
    }

    TIME_CHECK_END(__debug_sw.getTime());
    preTargetPoint = targetPoint;
    return curVel;
}

/**
 * @brief 기본 벽타기 제어
 * 
 * @param cur 
 * @param robotPose 
 * @param direction 
 * @return tTwist 
 */
tTwist CWalltracking::driveWallTrack(tTwist cur, tPose robotPose, E_WALLTRACK_DIR direction)
{
    tTwist curVel = cur;

    if (cliffTof > WALLTRACK_CLIFF_TOF)  
    {
        // 최소 각속도 [CW]가 2도/초가 되도록 제한
        if(direction == E_WALLTRACK_DIR::RIGHT)
        {
            if (curVel.w > -DEG2RAD(5))  {curVel.w = -DEG2RAD(5);}
        }
        else
        {
            if (curVel.w < DEG2RAD(5))   {curVel.w = DEG2RAD(5);}
        }
        // 벽에서 완전히 멀어졌을 때
        if (sideTof > 100) 
        {
            if(direction == E_WALLTRACK_DIR::RIGHT)  {curVel.w -= DEG2RAD(CONFIG.wallTrack_v2_accel_decel_W) * CONFIG.wallTrack_v2_W_Weight;}
            else                                        {curVel.w += DEG2RAD(CONFIG.wallTrack_v2_left_accel_decel_W) * CONFIG.wallTrack_v2_left_W_Weight;}
        }
        else
        {
            // 벽과의 거리에 따라 각속도 및 속도 조절
            if(direction == E_WALLTRACK_DIR::RIGHT)  {curVel.w -= DEG2RAD(CONFIG.wallTrack_v2_accel_decel_W);}
            else                                        {curVel.w += DEG2RAD(CONFIG.wallTrack_v2_left_accel_decel_W);}

            // 최대 각속도 제한
            if(direction == E_WALLTRACK_DIR::RIGHT)
            {
                if(curVel.w < -DEG2RAD(18)) {curVel.w = -DEG2RAD(18);}
            }
            else
            {
                if(curVel.w > DEG2RAD(18))  {curVel.w = DEG2RAD(18);}
            }
        }
    }
    else    // 벽에 가까워진 경우
    {
        // 최소 각속도 [CCW]가 5도/초가 되도록 제한
        if(direction == E_WALLTRACK_DIR::RIGHT)
        {
            if(curVel.w < DEG2RAD(5))   {curVel.w = DEG2RAD(5);}
        }
        else
        {
            if(curVel.w > -DEG2RAD(5))  {curVel.w = -DEG2RAD(5);}
        }
        if(direction == E_WALLTRACK_DIR::RIGHT)  {curVel.w += CONFIG.wallTrack_v2_accel_decel_W;}
        else                                        {curVel.w -= CONFIG.wallTrack_v2_left_accel_decel_W;}
    }
    
    if (curVel.w > DEG2RAD(50))          {curVel.w = DEG2RAD(50);}
    else if (curVel.w < -DEG2RAD(50))    {curVel.w = -DEG2RAD(50);}

    //  선속도 범위 조정
    double desiredV;
    desiredV = -1.4312*fabs(curVel.w)+0.474995;
    if(isObsDetectForSlowdown()) 
    {
        curVel.v -= CONFIG.wallTrack_v2_decel_V;
        if (curVel.v > 0.3)          {curVel.v = 0.3;}
        else if (curVel.v < 0.07)    {curVel.v = 0.07;}
        
        if(curVel.v < 0.2)
        {
            if(direction ==E_WALLTRACK_DIR::RIGHT){ curVel.w = DEG2RAD(CONFIG.wallTrack_v2_slow_W);}
            else                                     { curVel.w = -DEG2RAD(CONFIG.wallTrack_v2_left_slow_W);}
        }
    }
    else if(curVel.v > desiredV )
    {
        curVel.v -= CONFIG.wallTrack_v2_decel_V;
        if(direction ==E_WALLTRACK_DIR::RIGHT)
        {
            if (curVel.v > 0.3)          {curVel.v = 0.3;}
            else if (curVel.v < 0.07)    {curVel.v = 0.07;}
        }
        else
        {
            if (curVel.v > 0.3)          {curVel.v = 0.3;}
            else if (curVel.v < CONFIG.wallTrack_v2_left_V_weight)    {curVel.v = CONFIG.wallTrack_v2_left_V_weight;}
        }
    }
    else                                                
    {
        curVel.v += CONFIG.wallTrack_v2_accel_V;
        if (curVel.v > 0.3)          {curVel.v = 0.3;}
        else if (curVel.v < 0.07)    {curVel.v = 0.07;}
    }

    //ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDYELLOW, " 벽타기 제어 v,w ( " << curVel.v << " , " << curVel.w << " )");
    return curVel;
}

/**
 * @brief 벽 반대쪽 범퍼 감지 시 회전 제어
 * 
 * @param cur 
 * @param robotPose 
 * @param direction 
 * @return tTwist 
 */
tTwist CWalltracking::driveOppCurveBack(tTwist cur, tPose robotPose)
{
    tTwist curVel = cur;
    
    if(curVel.w > DEG2RAD(10))                              {curVel.w -= DEG2RAD(CONFIG.wallTrack_v2_oppback_accel_W)*10;}
    else if(curVel.w > DEG2RAD(5))                          {curVel.w -= DEG2RAD(CONFIG.wallTrack_v2_oppback_accel_W);}
    
    if(curVel.w < -DEG2RAD(10))                             {curVel.w += DEG2RAD(CONFIG.wallTrack_v2_oppback_accel_W)*10;}
    else if(curVel.w < -DEG2RAD(5))                         {curVel.w += DEG2RAD(CONFIG.wallTrack_v2_oppback_accel_W);}
       
    if (curVel.w < DEG2RAD(5) && curVel.w > -DEG2RAD(5))    {curVel.w = 0;}

    if(curVel.v > 0)                                        {curVel.v -= CONFIG.wallTrack_v2_oppback_accel_V*3;}
    else                                                    {curVel.v -= CONFIG.wallTrack_v2_oppback_accel_V;}
    
    if (curVel.v < -0.1)                                    {curVel.v = -0.1;}
    return curVel;
}

/**
 * @brief 벽쪽 범퍼 감지 시 회전 제어
 * 
 * @param cur 
 * @param robotPose 
 * @param direction 
 * @return tTwist 
 */
tTwist CWalltracking::driveCurveBack(tTwist cur, tPose robotPose, E_WALLTRACK_DIR direction)
{
    tTwist curVel = cur;
    
    if(direction == E_WALLTRACK_DIR::RIGHT)     {curVel.w = DEG2RAD(CONFIG.wallTrack_v2_back_W);}
    else                                        {curVel.w = -DEG2RAD(CONFIG.wallTrack_v2_back_W);}

    curVel.v = -CONFIG.wallTrack_v2_back_V; 
    return curVel;
}

/**
 * @brief 회전 제어
 * 
 * @param cur 
 * @param robotPose 
 * @param direction 
 * @return tTwist 
 */
tTwist CWalltracking::rotateWallTrack(tTwist cur, tPose robotPose, E_WALLTRACK_DIR direction)
{
    tTwist curVel = cur;

    // 각속도 가감속
    if(direction == E_WALLTRACK_DIR::RIGHT)
    {
        if(curVel.w < DEG2RAD(40))      {curVel.w += DEG2RAD(CONFIG.wallTrack_v2_rotate_accel_W)*3;}
        else                            {curVel.w += DEG2RAD(CONFIG.wallTrack_v2_rotate_accel_W);}
        
        if(curVel.w > DEG2RAD(50))      {curVel.w = DEG2RAD(50);}
        
        if(curVel.v < 0)                {curVel.v += CONFIG.wallTrack_v2_rotate_accel_V*3;}
        else                            {curVel.v += CONFIG.wallTrack_v2_rotate_accel_V;}

        if(curVel.v > 0.1)              {curVel.v = 0.1;}
    }
    else
    {
        if(curVel.w > -DEG2RAD(40))     {curVel.w -= DEG2RAD(CONFIG.wallTrack_v2_left_rotate_accel_W)*3;}
        else                            {curVel.w -= DEG2RAD(CONFIG.wallTrack_v2_left_rotate_accel_W);}

        if(curVel.w < -DEG2RAD(CONFIG.wallTrack_v2_left_rotate_W_max))
                                        {curVel.w = -DEG2RAD(CONFIG.wallTrack_v2_left_rotate_W_max);}
        
        if(curVel.v < 0)                {curVel.v += CONFIG.wallTrack_v2_left_rotate_accel_V*3;}
        else                            {curVel.v += CONFIG.wallTrack_v2_left_rotate_accel_V;}

        if(curVel.v > CONFIG.wallTrack_v2_left_rotate_V_max)
                                        {curVel.v = CONFIG.wallTrack_v2_left_rotate_V_max;}
    }

    return curVel;
}

/**
 * @brief [영역 벽타기] 영역 Path 추종 제어기
 * 
 * @param cur 
 * @param robotPose 
 * @param lineTargetPoint 
 * @return tTwist 
 */
tTwist CWalltracking::driveAreaLineTrack(tTwist cur, tPose robotPose, tPoint startPoint, tPoint lineTargetPoint)
{
    double desiredV;
    double desiredW;
    tTwist curVel = cur;

	tPoint calTargetPoint       = getCalculatedTargetPoint(robotPose, lineTargetPoint, 0.35);
    double tempTargetAngle      = atan2(calTargetPoint.y - robotPose.y, calTargetPoint.x - robotPose.x);

    double angleError           = utils::math::getTurnRadAngle(tempTargetAngle, robotPose.angle);
    std::list<double> lineCoeff = utils::math::calculateLineEquation(tPoint(robotPose.x, robotPose.y), calTargetPoint);
    double lineDisErr           = utils::math::calculateDistanceFromLine(tPoint(robotPose.x,robotPose.y), lineCoeff);
    double relativePosition     = utils::math::pointRelativeToLine(tPoint(robotPose.x, robotPose.y), startPoint, calTargetPoint);

    if(relativePosition>0)  {lineDisErr = lineDisErr * (-1);}

    double nomalizedLineDisErr;
    if (lineDisErr > 0.35)          {lineDisErr = 0.35;}
    else if (lineDisErr < -0.35)    {lineDisErr = -0.35;}
    nomalizedLineDisErr = (M_PI_2 - acos(lineDisErr/0.35));

    /*
        각도에러 + 거리에러 고려한 desiredW
        거리 에러만 클때 => 거리에러 가중치 10배 (gain = 1 기준)
        각도 에러만 클때 => 각도에러 가중치 10배 (gain = 1 기준)
    */
    if (angleError < DEG2RAD(10) && lineDisErr > 0.1 || fabs(lineDisErr) > 10)
    {
        double anglePTerm = angleError                      * CONFIG.pointControl_angle_P_gain * 0.1;
        double lineTerm   = nomalizedLineDisErr             * CONFIG.pointControl_Line_P_gain;
        desiredW = anglePTerm + lineTerm;
    }
    else if (angleError > DEG2RAD(10) && lineDisErr < 0.1 || fabs(angleError) > DEG2RAD(CONFIG.pointControl_max_W))
    {
        double anglePTerm = angleError                      * CONFIG.pointControl_angle_P_gain;
        double lineTerm   = nomalizedLineDisErr             * CONFIG.pointControl_Line_P_gain * 0.1;
        desiredW = anglePTerm + lineTerm;
    }
    else
    {
        double anglePTerm = angleError                      * CONFIG.pointControl_angle_P_gain;
        double lineTerm   = nomalizedLineDisErr             * CONFIG.pointControl_Line_P_gain;
        desiredW = anglePTerm + lineTerm;
    }

    switch(areaControlStep)
    {
    case E_AREA_WALLTRACK_CONTROL_STEP::V_DECEL :
        if(curVel.v > CONFIG.stopControl_decel_V*10)                {curVel.v -= CONFIG.stopControl_decel_V*5;}
        else if(curVel.v > CONFIG.stopControl_decel_V*5)            {curVel.v -= CONFIG.stopControl_decel_V*4;}
        else if(curVel.v > CONFIG.stopControl_decel_V*2)            {curVel.v -= CONFIG.stopControl_decel_V*3;}
        else if(curVel.v < -CONFIG.stopControl_decel_V*10)          {curVel.v += CONFIG.stopControl_decel_V*5;}
        else if(curVel.v < -CONFIG.stopControl_decel_V*5)           {curVel.v += CONFIG.stopControl_decel_V*4;}
        else if(curVel.v < -CONFIG.stopControl_decel_V*2)           {curVel.v += CONFIG.stopControl_decel_V*3;}
        else
        {
            curVel.v = 0;
            areaControlStep = E_AREA_WALLTRACK_CONTROL_STEP::ROTATE;
            ceblog(LOG_LV_NECESSARY, BLUE, "STEP : V_DECEL -> ROTATE");
        }
        break;
    case E_AREA_WALLTRACK_CONTROL_STEP::ROTATE :
        if (fabs(angleError) < DEG2RAD(3))
        {
            areaControlStep = E_AREA_WALLTRACK_CONTROL_STEP::MOVE_FORWARD;
            ceblog(LOG_LV_NECESSARY, BLUE, "STEP : ROTATE -> MOVE_FORWARD");
        }
        else
        {
            double tempDesiredW;
            if (angleError > DEG2RAD(CONFIG.pointControl_max_W))                        {tempDesiredW = DEG2RAD(CONFIG.pointControl_max_W);}
            else if (angleError < -DEG2RAD(CONFIG.pointControl_max_W))                  {tempDesiredW = -DEG2RAD(CONFIG.pointControl_max_W);}
            else                                                                        {tempDesiredW = angleError;}

            if(curVel.w < tempDesiredW - DEG2RAD(CONFIG.pointControl_accel_W)*10)       {curVel.w += DEG2RAD(CONFIG.pointControl_accel_W)*3;}
            else if(curVel.w < tempDesiredW)                                            {curVel.w += DEG2RAD(CONFIG.pointControl_accel_W);}
            else if(curVel.w > tempDesiredW + DEG2RAD(CONFIG.pointControl_decel_W)*10)  {curVel.w -= DEG2RAD(CONFIG.pointControl_decel_W)*3;}
            else if(curVel.w > tempDesiredW)                                            {curVel.w -= DEG2RAD(CONFIG.pointControl_decel_W);}
            else                                                                        {curVel.w = tempDesiredW;}

            if (curVel.w  <  DEG2RAD(CONFIG.pointControlRotate_deadzone_w) && curVel.w > 0 && fabs(angleError) < DEG2RAD(CONFIG.pointControlRotate_deadzone_w))
            {curVel.w =  DEG2RAD(CONFIG.pointControlRotate_deadzone_w);}
            else if (curVel.w  >  -DEG2RAD(CONFIG.pointControlRotate_deadzone_w) && curVel.w  < 0 && fabs(angleError) < DEG2RAD(CONFIG.pointControlRotate_deadzone_w))
            {curVel.w = - DEG2RAD(CONFIG.pointControlRotate_deadzone_w);}
        }
        break;
    case E_AREA_WALLTRACK_CONTROL_STEP::MOVE_FORWARD :
    
        if(fabs(desiredW) > DEG2RAD(CONFIG.pointControl_max_W))
        {
            if(curVel.w > DEG2RAD(CONFIG.pointControl_max_W))                       {curVel.w = DEG2RAD(CONFIG.pointControl_max_W);}
            else if(curVel.w < -DEG2RAD(CONFIG.pointControl_max_W))                 {curVel.w = -DEG2RAD(CONFIG.pointControl_max_W);}    
        }
        else
        {
            if(curVel.w > fabs(desiredW))                                           {curVel.w = fabs(desiredW);}
            else if(curVel.w < -fabs(desiredW))                                     {curVel.w = -fabs(desiredW);}
        }

        // 각속도 가감속
        if(curVel.w < desiredW - DEG2RAD(CONFIG.pointControl_accel_W)*10)           {curVel.w += DEG2RAD(CONFIG.pointControl_accel_W)*3;}
        else if(curVel.w < desiredW)                                                {curVel.w += DEG2RAD(CONFIG.pointControl_accel_W);}
        else if(curVel.w > desiredW + DEG2RAD(CONFIG.pointControl_decel_W)*10)      {curVel.w -= DEG2RAD(CONFIG.pointControl_decel_W)*3;}
        else if(curVel.w > desiredW)                                                {curVel.w -= DEG2RAD(CONFIG.pointControl_decel_W);}
        else                                                                        {curVel.w  = desiredW;}
        
        double disRobot2Goal        = utils::math::distanceTwoPoint(tPoint(robotPose.x, robotPose.y), lineTargetPoint);
        bool isRobotCrossEndLine    = utils::math::isRobotCrossLine(robotPose, startPoint, lineTargetPoint);

        // 도착지점 근처 감속 조건
        if(disRobot2Goal < CONFIG.pointControl_decel_dis) 
        {
            desiredV = (0.35/CONFIG.pointControl_decel_dis)*disRobot2Goal;
            curVel.w = curVel.w*(disRobot2Goal/CONFIG.pointControl_decel_dis);
        }
        else                                              
        {
            if(fabs(curVel.w) < DEG2RAD(5))                     {desiredV = 0.35;}
            else if(fabs(curVel.w) > DEG2RAD(15))               {desiredV = 0.02;}
            else                                                {desiredV = - 1.8856 * fabs(curVel.w) + 0.514869;}   // -1.660964*fabs(curVel.w)+0.494804 : 0.06
        }

        // 선속도 가감속
        if(isObsDetectForSlowdown()) 
        {
            curVel.v -= CONFIG.wallTrack_v2_decel_V;
            if(curVel.v <0.04)
            {
                curVel.v = 0.04;
            }
        }
        else
        {
            if(curVel.v > desiredV + CONFIG.pointControl_decel_V )  {curVel.v -= CONFIG.pointControl_decel_V*4;}
            else if(curVel.v > desiredV )                           {curVel.v -= CONFIG.pointControl_decel_V*2;}
            else if(curVel.v < 0)                                   {curVel.v += CONFIG.pointControl_accel_V*6;}
            else                                                    {curVel.v += CONFIG.pointControl_accel_V*2;}
        }
    }

    if(curVel.v > 0.35)  {curVel.v = 0.35;}
    else if(curVel.v < 0)  {curVel.v = 0;}
    
    return curVel;
}

/**
 * @brief 벽타기 장애물 탈출 제어
 * 
 * @param robotPose 
 * @param curVel 
 * @param isOppBack 
 * @param isBack 
 * @param isRotate 
 * @return tTwist 
 */
tTwist CWalltracking::driveEscapeWall(tPose robotPose, tTwist curVel, E_WALLTRACK_DIR direction, bool isOppBack, bool isBack, bool isRotate)
{
    tTwist ret;

    if(isOppBack)           
    {
        ret = driveOppCurveBack(curVel, robotPose);
        if(checkPatternTime())      {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::ROTATE, CONFIG.wallTrack_v2_rotate_time);}
    }
    else if(isBack)         
    {
        ret = driveCurveBack(curVel, robotPose, direction);
        if(checkPatternTime())      {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::NONE);}
    }
    else if(isRotate)       
    {
        ret = rotateWallTrack(curVel, robotPose, direction);
        if(checkPatternTime() && isObsDetectForTurning() == false)      {setPatternFlags(E_WALLTRACK_PATTERN_FLAG::NONE);}
    }
    
    avoidStartTime = SYSTEM_TOOL.getSystemTime();

    return ret;
}

void CWalltracking::setPatternFlags(E_WALLTRACK_PATTERN_FLAG flag, double time)
{
    tPose robotPose = ServiceData.localiz.getPose();
    switch (flag)
    {
    case E_WALLTRACK_PATTERN_FLAG::NONE:
        isRotate    = false;
        isOppBack   = false;
        isBack      = false;
        bAvoiding = false;
        ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDYELLOW, " 회피 끝! ");
        break;
    case E_WALLTRACK_PATTERN_FLAG::ROTATE:
        isRotate    = true;
        isOppBack   = false;
        isBack      = false;
        // ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDYELLOW, " 돌아 ");
        break;
    case E_WALLTRACK_PATTERN_FLAG::OPP_BACK:
        isRotate    = false;
        isOppBack   = true;
        isBack      = false;
        bAvoiding = true;
        MOTION.startStopOnMap(tProfile(),true);
        
        // ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDYELLOW, " 옵 후진 ");
        break;
    case E_WALLTRACK_PATTERN_FLAG::BACK:
        isRotate    = false;
        isOppBack   = false;
        isBack      = true;
        bAvoiding = true;
        MOTION.startStopOnMap(tProfile(),true);
        
        // ceblog((LOG_LV_NECESSARY|LOG_LV_MOTION), BOLDYELLOW, " 걍 후진 ");
        break;    
    default:
        break;
    }
    obsTime = time;
    obsChecktime = get_system_time();
}

bool CWalltracking::checkPatternTime()
{
    bool ret = false;
    if(get_system_time(obsChecktime) >= obsTime)    {ret=true;}
    return ret;   
}

bool CWalltracking::areaWallTrackStateChecker(tPose robotPose, E_AREA_WALLTRACK_CONTROL_STEP curStep)
{
    bool ret = false;
    if (robotPose.distance(preTargetPoint) <= 0.15/* && curStep == E_AREA_WALLTRACK_CONTROL_STEP::MOVE_FORWARD*/) {ret = true;}
    return ret;
}