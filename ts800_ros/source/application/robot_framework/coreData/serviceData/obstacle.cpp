#include "coreData/serviceData/obstacle.h"
#include "externData/externData.h"
#include "eblog.h"
#include "utils.h"
#include "MessageHandler.h"
#include "control/control.h"
#include "obstacle.h"
#include "motionPlanner/motionPlanner.h"
#include <cmath>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CImuSensors::CImuSensors(/* args */)
{

}

CImuSensors::~CImuSensors()
{

}

void CImuSensors::initImuSensor()
{
    data.Ax = 0;
    data.Ay = 0;
    data.Az = 0;
    data.Groll = 0;
    data.Gpitch = 0;
    data.Gyaw = 0;
    data.state = 0;
    data.filteredState = 0;
}

void CImuSensors::updateImuSensor(tSysIMU _set)
{
    data = _set;
}

tSysIMU CImuSensors::getImuSensor()
{
    return data;
}

u8 CImuSensors::getImuState()
{
    return data.filteredState; // hhryu240405 : imu 상태 값이 흔들리는 현상이 있어서 필터 적용함, 필요시 imu.state를 이용할 수 있다.
}

CTofSensors::CTofSensors(/* args */){
    initTofSensor();
}
CTofSensors::~CTofSensors(){
}

void CTofSensors::initTofSensor(){
    leftwall.deviceState = 0;
    leftwall.rangeStatus = 0;
    leftwall.rangeAvg = 0;
    rightwall.deviceState = 0;
    rightwall.rangeStatus = 0;
    rightwall.rangeAvg = 0;
    knoll.deviceState = 0;
    knoll.rangeStatus = 0;
    knoll.rangeAvg = 0;

    continuousCliffErrorCount = 0;
    errorLogTime = std::numeric_limits<u32>::max();
}
void CTofSensors::updateTofSensorData(tTof tof, tSysCliff cliff)
{
    leftwall = tof.leftSide;
    rightwall = tof.rightSide;
    knoll = tof.knollCenter;
    left = cliff.left;
    right = cliff.right;
}

/**
 * @exception 현재 구조에서 tilt state를 tofSensors가 받을 수 없으나, 필요해서 룰 파괴한 함수
 * @brief 틸 다운일 때 에러 체크
 * @note 연산시간 ms
 * @date 2023-11-06
 * @author hhryu
 */
void CTofSensors::tiltDownCheckTofError()
{
    if (left.rangeAvg > TOF_ERROR_TILT_DOWN_RANGE || right.rangeAvg > TOF_ERROR_TILT_DOWN_RANGE) // temp. 1000 -> 300
    {
        continuousCliffErrorCount++; // count말고 time으로 변경 예정
        if (continuousCliffErrorCount > 100)
        {
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::LIFT_CLIFF)));
            ceblog(LOG_LV_NECESSARY, YELLOW, "Error Detected\t Cliff [" << left.rangeAvg << ", " << right.rangeAvg << "]");
            continuousCliffErrorCount = 0;
        }
    }
    else if (left.rangeAvg != 0 && right.rangeAvg != 0)
    {
        continuousCliffErrorCount = 0;
        // ceblog(LOG_LV_OBSTACLE, YELLOW, "continuousCliffErrorCount CLEAR");
    }
}

/**
 * @exception 현재 구조에서 tilt state를 tofSensors가 받을 수 없으나, 필요해서 룰 파괴한 함수
 * @brief 틸 업 일 때 에러 체크
 * @note 연산시간 ms
 * @date 2023-11-06
 * @author hhryu
 */
void CTofSensors::tiltUpCheckTofError()
{
    if (left.rangeAvg > TOF_ERROR_TILT_UP_RANGE || right.rangeAvg > TOF_ERROR_TILT_UP_RANGE) // temp. 1000 -> 300
    {
        continuousCliffErrorCount++; // count말고 time으로 변경 예정
        if (continuousCliffErrorCount > 100)
        {
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::LIFT_CLIFF)));
            ceblog(LOG_LV_NECESSARY, YELLOW, "Error Detected\t Cliff [" << left.rangeAvg << ", " << right.rangeAvg << "]");
            continuousCliffErrorCount = 0;
        }
    }
    else if (left.rangeAvg != 0 && right.rangeAvg != 0)
    {
        continuousCliffErrorCount = 0;
        // ceblog(LOG_LV_OBSTACLE, YELLOW, "continuousCliffErrorCount CLEAR");
    }
}

/**
 * @exception 현재 구조에서 tilt state를 tofSensors가 받을 수 없으나, 필요해서 룰 파괴한 함수
 * @brief ToF 상태를 체크해서 에러 메시지를 날리는 함수
 * @param tilt 
 * @note 연산시간 ms
 * @date 2023-11-06
 * @author hhryu
 */
void CTofSensors::checkAndSendErrorMessage(tSysTilting tilt)
{
    if (left.deviceState == E_TOF_STATUS::TOF_READY && right.deviceState == E_TOF_STATUS::TOF_READY && knoll.deviceState == E_TOF_STATUS::TOF_READY && right.deviceState == E_TOF_STATUS::TOF_READY)
    {
        if ((left.rangeAvg <= TOF_MIN_RANGE || left.rangeAvg > TOF_MAX_RANGE) || (right.rangeAvg <= TOF_MIN_RANGE || right.rangeAvg > TOF_MAX_RANGE)) // ???
        {
            if (errorLogTime == std::numeric_limits<u32>::max())
            {
                errorLogTime = SYSTEM_TOOL.getSystemTick();
            }
            else
            {
                if (SYSTEM_TOOL.getSystemTick() - errorLogTime >= 3 * SEC_1)
                {
                    // ROBOT_CONTROL.system.initTofSensor();
                    errorLogTime = SYSTEM_TOOL.getSystemTick();
                    ceblog(LOG_LV_NECESSARY, BLUE, "TOF RANGE ERORR RAGNE LEFT : " << left.rangeAvg << "\tRIGHT: " << right.rangeAvg);
                }
            }
        }
        else
        {
            if (tilt.state == E_SYS_TILT_STATE::TILTED_DOWN)
            {
                tiltDownCheckTofError();
            }
            else if (tilt.state == E_SYS_TILT_STATE::TILTED_UP)
            {
                tiltUpCheckTofError();
            }
            else
            {
                /**
                 * hhryu231106
                 * 틸 업 중이거나, 다운 중, 에러, 모르는 상태 일 때는
                 * 주행 중이 아니니 에러 체크를 안해도 되지않을까요?
                 * 틸 에러라고 여기서 에러를 띄우는 것 보다는 틸 체크하는 부분에서 띄우는게 관리에 용이할 것 같음....
                 */
                // 240130 : 틸 상태가 대부분 정상이 아님, 임시로 처리해놓고 정상화되면 삭제 예정.
#if SKIP_CHECKTILT > 0
                tiltUpCheckTofError(); // 틸 업 중에 낙하 감지하면 회피를 해야지, 연속 낙하 감지로 에러를 띄워서는 안된다. (틸 업 중 회피제어를 할 수가 없으므로)
#endif
            }
        }
    }
    else
    {
        if (errorLogTime == std::numeric_limits<u32>::max())
        {
            errorLogTime = SYSTEM_TOOL.getSystemTick();
        }
        else
        {
            if (SYSTEM_TOOL.getSystemTick() - errorLogTime >= 3 * SEC_1)
            {
                // ROBOT_CONTROL.system.initTofSensor();
                errorLogTime = SYSTEM_TOOL.getSystemTick();
                ROBOT_CONTROL.system.initTofSensor();
                ceblog(LOG_LV_NECESSARY, YELLOW, "TOF DEVICE ERORR STATE: LEFT : " << left.deviceState << "\tRIGHT: " << right.deviceState);
            }
        }
    }
}

tSysTofInfo CTofSensors::getTofLeftWall(){
    return leftwall;
}

tSysTofInfo CTofSensors::getTofRightWall(){
    return rightwall;
}

tSysTofInfo CTofSensors::getTofKnoll()
{
    return knoll;
}

tSysTofInfo CTofSensors::getLeftCliff()
{
    return left;
}

tSysTofInfo CTofSensors::getRightCliff()
{
    return right;
}


CBumperSensors::CBumperSensors(/* args */){
    initBumperSensor();
}
CBumperSensors::~CBumperSensors(){

}
void CBumperSensors::initBumperSensor(){
    mask.value = 0;
    pre_mask.value = 0;
}
RSF_OBSTACLE_MASK CBumperSensors::getBumperMask()
{
    RSF_OBSTACLE_MASK ret;
    ret.value = 0;
    if(pre_mask.value)
    {
        eblog(LOG_LV_OBSTACLE,  " 범퍼감지 :  " << enumToString(pre_mask));
        ret = pre_mask;
        pre_mask.value = 0;
    }

    return ret;
}

void CBumperSensors::updateBumperSensor(tSysBumper bumper)
{
    mask.value = 0;

    if(bumper.right) mask.b.fright_side = 1;
    if(bumper.left)  mask.b.fleft_side = 1;

    if(mask.value)   pre_mask.value = mask.value;
}

CFrontSensors::CFrontSensors(/* args */){
    initFrontSensor();
}
CFrontSensors::~CFrontSensors(){}

void CFrontSensors::initFrontSensor(){
    mask.approach.value = 0;
    mask.obstacle.value = 0;
    pre_mask.value = 0;
    frontIr.center.lpfData = 0;
    frontIr.left.lpfData = 0;
    frontIr.right.lpfData = 0;

    frontIr.center.old_lpfData = 0;
    frontIr.left.old_lpfData = 0;
    frontIr.right.old_lpfData = 0;

    frontIr.center.slope = 0;
    frontIr.left.slope = 0;
    frontIr.right.slope = 0;

    frontIr.center.state = FRONT_STATE_OPEN;
    frontIr.left.state = FRONT_STATE_OPEN;
    frontIr.right.state = FRONT_STATE_OPEN;
}

tFrontIr CFrontSensors::getFrontSensor()
{
    return frontIr;
}

void CFrontSensors::setFrontSensor(tSysFront sysFrontIr)
{
    frontIr.center.raw_data = sysFrontIr.center;
    frontIr.left.raw_data = sysFrontIr.left;
    frontIr.right.raw_data = sysFrontIr.right;

    if(frontIr.center.lpfData == 0) frontIr.center.lpfData = sysFrontIr.center;
    else                            frontIr.center.lpfData = LPF_RAW(frontIr.center.lpfData,sysFrontIr.center);

    if(frontIr.left.lpfData == 0)   frontIr.left.lpfData = sysFrontIr.left;
    else                            frontIr.left.lpfData = LPF_RAW(frontIr.left.lpfData,sysFrontIr.left);

    if(frontIr.right.lpfData == 0)  frontIr.right.lpfData = sysFrontIr.right;
    else                            frontIr.right.lpfData = LPF_RAW(frontIr.right.lpfData,sysFrontIr.right);

    if(frontIr.center.old_lpfData == 0) frontIr.center.old_lpfData = frontIr.center.lpfData;
    if(frontIr.center.slope == 0)   frontIr.center.slope = frontIr.center.lpfData-frontIr.center.old_lpfData; 
    else                            frontIr.center.slope = LPF_SLOPE(frontIr.center.slope,(frontIr.center.lpfData-frontIr.center.old_lpfData));                          

    if(frontIr.left.old_lpfData == 0) frontIr.left.old_lpfData = frontIr.left.lpfData;
    if(frontIr.left.slope == 0) frontIr.left.slope = (s16)(frontIr.left.lpfData-frontIr.left.old_lpfData);
    else                        frontIr.left.slope = LPF_SLOPE(frontIr.left.slope,(frontIr.left.lpfData-frontIr.left.old_lpfData));                       

    if(frontIr.right.old_lpfData == 0) frontIr.right.old_lpfData = frontIr.right.lpfData;
    if(frontIr.right.slope == 0) frontIr.right.slope = frontIr.right.lpfData-frontIr.right.old_lpfData;
    else                         frontIr.right.slope = LPF_SLOPE(frontIr.right.slope,(frontIr.right.lpfData-frontIr.right.old_lpfData));

    //if(frontIr.center.state != 0xffff && frontIr.left.state != 0xffff && frontIr.right.state != 0xffff)
    {
        if(frontIr.center.state != FRONT_STATE_OPEN) frontIr.center.maxSlope =  frontIr.center.maxSlope > frontIr.center.slope ? frontIr.center.maxSlope: frontIr.center.slope;
        else                                                                    frontIr.center.maxSlope = 0;   

        if(frontIr.left.state != FRONT_STATE_OPEN)   frontIr.left.maxSlope =    frontIr.left.maxSlope > frontIr.left.slope ? frontIr.left.maxSlope: frontIr.left.slope;
        else                                                                    frontIr.left.maxSlope = 0;

        if(frontIr.right.state != FRONT_STATE_OPEN)  frontIr.right.maxSlope =   frontIr.right.maxSlope > frontIr.right.slope ? frontIr.right.maxSlope: frontIr.right.slope;
        else                                                                    frontIr.right.maxSlope = 0;
    }                            

    frontIr.center.old_lpfData = frontIr.center.lpfData;
    frontIr.left.old_lpfData = frontIr.left.lpfData;
    frontIr.right.old_lpfData = frontIr.right.lpfData;
}

FRONT_OSBATALCE_STATE CFrontSensors::frontStateMachine(tFrontData frontData)
{
    FRONT_OSBATALCE_STATE ret = FRONT_STATE_OPEN;

    switch (frontData.state)
    {
    case FRONT_STATE_OPEN :
        if(frontData.lpfData >= IR_FRONT_OPENVALUE) 
        {
            ret = FRONT_STATE_APPROACH;
            // if(frontData.slope >= IR_FRONT_OBSTACLE_SLOPE && frontData.maxSlope >= IR_FRONT_CHECK_MAXSLOPE)     ret = FRONT_STATE_OBSTACLE;
            // else                                                                                                ret = FRONT_STATE_APPROACH;
        }

        /* code */
        break;
    case FRONT_STATE_APPROACH :
        /* code */
        if(frontData.lpfData < IR_FRONT_OPENVALUE)                                                              ret = FRONT_STATE_OPEN;
        else if(frontData.slope >= IR_FRONT_OBSTACLE_SLOPE && frontData.maxSlope >= IR_FRONT_CHECK_MAXSLOPE)    ret = FRONT_STATE_OBSTACLE;
        else                                                                                                    ret = FRONT_STATE_APPROACH;

        break;
    case FRONT_STATE_OBSTACLE :
        ret = FRONT_STATE_OPEN;
        // if(frontData.lpfData < IR_FRONT_OPENVALUE) ret = FRONT_STATE_OPEN;
        // else                                       ret = FRONT_STATE_OBSTACLE;
        break;        
    
    default:
        ret = FRONT_STATE_OPEN;
        break;
    }

    return ret;
}

RSF_OBSTACLE_MASK CFrontSensors::getFrontApproachMask()
{
    return mask.approach;
}
RSF_OBSTACLE_MASK CFrontSensors::getFrontObstacleMask()
{
    RSF_OBSTACLE_MASK ret;
    ret.value = 0;

    if(pre_mask.value)
    {
        //eblog(LOG_LV_NECESSARY,  "getFrontObstacleMask :" << SC<int>(pre_mask.value));
        ret.value = pre_mask.value;
        pre_mask.value = 0;
    }

    return ret;
}

void CFrontSensors::updateFrontSensor(tSysFront sysFrontIr)
{
    mask.approach.value = 0;
    mask.obstacle.value = 0;

    setFrontSensor(sysFrontIr); // hhryu230717 : set을 직진일 때만 하면, 데이터가 업데이트되지 않음. 마스킹만 직진에서 하도록 변경

    frontIr.center.state = frontStateMachine(frontIr.center);
    frontIr.left.state = frontStateMachine(frontIr.left);
    frontIr.right.state = frontStateMachine(frontIr.right);

    if(frontIr.center.state == 1)
    {
        mask.approach.b.fleft_center = 1;
        mask.approach.b.fright_center = 1;
    }        
    else if(frontIr.center.state == 2)
    {
        mask.obstacle.b.fleft_center = 1;
        mask.obstacle.b.fright_center = 1;
        //eblog (LOG_LV_NECESSARY, " Front IR - center obs mask true - left.lpfData : "<<SC<int>(frontIr.left.lpfData)<<" frontIr.center.lpfData : "<<SC<int>(frontIr.center.lpfData)<<" frontIr.right.lpfData : "<<SC<int>(frontIr.right.lpfData));
    }
    
    if(frontIr.left.state == 1)
    {
        mask.approach.b.fleft_side = 1;
    }        
    else if(frontIr.left.state == 2)
    {
        mask.obstacle.b.fleft_side = 1;
        //eblog (LOG_LV_NECESSARY, " Front IR - left obs mask true - left.lpfData : "<<SC<int>(frontIr.left.lpfData)<<" frontIr.center.lpfData : "<<SC<int>(frontIr.center.lpfData)<<" frontIr.right.lpfData : "<<SC<int>(frontIr.right.lpfData));
    }

    if(frontIr.right.state == 1)
    {
        mask.approach.b.fright_side = 1;
    }        
    else if(frontIr.right.state == 2)
    {
        mask.obstacle.b.fright_side = 1;
        //eblog (LOG_LV_NECESSARY, " Front IR - right obs mask true - left.lpfData : "<<SC<int>(frontIr.left.lpfData)<<" frontIr.center.lpfData : "<<SC<int>(frontIr.center.lpfData)<<" frontIr.right.lpfData : "<<SC<int>(frontIr.right.lpfData));
    }

    if(mask.obstacle.value)
    {
        pre_mask.value = mask.obstacle.value;
    }
}


CWheeltrap::CWheeltrap(/* args */)
{
    initWheeltrapSensor();
}
CWheeltrap::~CWheeltrap()
{
    
}
void CWheeltrap::initWheeltrapSensor()
{
    errorCount = 0;
    trapDetectionCount = 0;
    lineartrapCount = 0;
    angulartrapCount = 0;
    poseChangeThreshold = 0.001;
    trapDetectionThreshold = 1;
    ctrlStartTime = 0;
    movingStartTime = 0;
    movingDistance = 0;
    mask.value = 0;
    pre_mask.value = 0;
    type = 0;
}

double CWheeltrap::calculateDistance(const tSysPose& pose1, const tSysPose& pose2) {
    return std::sqrt(std::pow(pose1.x - pose2.x, 2) + std::pow(pose1.y - pose2.y, 2));
}

void CWheeltrap::updateWheelTrapData(tSysWheelMotor sysWheelData, tSysPose sysPose)
{
    mask.value = 0;
    // int deltaLeft = std::abs(sysWheelData.leftEncoder - lastWheelData.leftEncoder);
    // int deltaRight = std::abs(sysWheelData.rightEncoder - lastWheelData.rightEncoder);
    //bool isPoseChanged = std::abs(distance) > poseChangeThreshold;
    //주행 테스트시에는 트랩 조건을 블럭킹 합니다. 특히 습식 상태
    if(MOTION.isRunning())
    {
#if defined (USED_RBT_PLUS) && (USED_RBT_PLUS == 0) // hjkim240501 - 트랩 활성화는 interface lib 0.3 version 부터 적용가능 (시스템 velocity 데이터 필요)   
        double runTime = 0, movingTime = 0, refDistance = 0, DesLineVel = 0, DesAngluaVel = 0, realLineVel = (double)sysWheelData.lineVelocity/1000 , realAngularVel = (double)sysWheelData.angularVelocity/1000, refDistancePerSec = 0;

        DesLineVel = MOTION.getControlVelocity().v;//(double)sysWheelData.lineVelocity/1000;lineVel = 0.1;//
        DesAngluaVel = MOTION.getControlVelocity().w;//(double)sysWheelData.angularVelocity/1000;angluaVel = 0;

        //eblog(LOG_LV_NECESSARY, "트랩 미정 - distance"<< distance<<" , ? : "<< isPoseChanged << " sysLEnc : " << sysWheelData.leftEncoder << " sysREnc : " << sysWheelData.rightEncoder << "sys V : " << sysWheelData.lineVelocity << " sys W : " << sysWheelData.angularVelocity);
        if(DesLineVel >= 0.07 || DesAngluaVel >= DEG2RAD(20)) //&& realLineVel >= 0.1
        {
            runTime = SYSTEM_TOOL.getSystemTime()-ctrlStartTime;
            movingDistance = utils::math::distanceTwoPoint(lastSysPose, sysPose);
            if(runTime >= 0.1)
            {
                double checkLineVel = realLineVel/DesLineVel*100;
                double checkAnglurVel = realAngularVel/DesAngluaVel*100;
                movingTime = SYSTEM_TOOL.getSystemTime()-movingStartTime;
                // refDistance = (lineVel*movingTime + angluaVel*0.5*pow(movingTime,2));
                // refDistancePerSec = lineVel*1 + angluaVel*0.5*pow(1,2);
                
                if(checkLineVel <= CONFIG.standWheelTrap) //hjkim240529 : 트랩 기준 % 낮춤 (트랩 오 판단)
                {
                    if(++lineartrapCount >= 100)
                    {
                        ceblog(LOG_LV_NECESSARY,RED, "직선 주행 트랩 확정!!!! ");
                        mask.b.fright_side = 1;
                        mask.b.fleft_side = 1;
                        lineartrapCount = 0;
                        errorCount++;
                    }
                    //eblog(LOG_LV_NECESSARY, "checkLineVel  : "<< checkLineVel << "%" "Des V : " << DesLineVel << " SYS V : " << realLineVel << "count : "<< lineartrapCount << " runTime : " << runTime << " movingTime : " << movingTime);
                }
                else
                {
                    lineartrapCount = 0;
                    errorCount = 0;
                }

                if(checkAnglurVel <= CONFIG.standWheelTrap) //hjkim240529 : 트랩 기준 % 낮춤 (트랩 오 판단)
                {
                    if(++angulartrapCount >= 200)
                    {
                        ceblog(LOG_LV_NECESSARY,RED, "회전 주행 트랩 확정!!!! ");
                        mask.b.fright_side = 1;
                        mask.b.fleft_side = 1;
                        angulartrapCount = 0;
                        errorCount++;
                    }
                    //eblog(LOG_LV_NECESSARY, "checkAngularVel  : "<< checkAnglurVel << "%" "Des V : " << DesLineVel << " SYS W : " << realAngularVel << "count : "<< angulartrapCount << " runTime : " << runTime << " movingTime : " << movingTime);
                }
                else
                {
                    angulartrapCount = 0;
                    errorCount = 0;
                }
            }
            else
            {
                movingStartTime = SYSTEM_TOOL.getSystemTime();
            }
        }
        else
        {
            movingDistance = 0;
            ctrlStartTime = SYSTEM_TOOL.getSystemTime();
        }
        
        if(errorCount >= 5){
            errorCount = 0;
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::WHEEL_WRONG_MOP)));
            ceblog(LOG_LV_NECESSARY, YELLOW, " 휠 구속 에러!!");
        }
        if(mask.value)   pre_mask.value = mask.value;
        lastWheelData = sysWheelData;
        lastSysPose = sysPose;
    #endif
    }
}

RSF_OBSTACLE_MASK CWheeltrap::getWheelTrapMask()
{
    RSF_OBSTACLE_MASK ret;
    ret.value = 0;
    if(pre_mask.value)
    {
        eblog(LOG_LV_OBSTACLE,  " 트랩 :  " << enumToString(pre_mask));
        ret = pre_mask;
        pre_mask.value = 0;
    }

    return ret;
}



CObstacle::CObstacle()
{
    initObstacleSensor();
}

CObstacle::~CObstacle() {}

void CObstacle::update(CExternData* pExternData)
{
	//hjkim 230906 : TODOLIST : CExternData 데이터에서 데이터가 업데이트가 안되는 경우. 장애물 정보가 계속해서 이전 정보를 가지고 있음.
	// ex) 범퍼가 눌림 감지 된 이후 범퍼 업데이트 flag가 셋팅이 안되면 범퍼 정보가 계속 살아 있음.
	// 이렇게 변경된 이유는 둔턱 감지 상태에서 범퍼 눌림을 계속해서 모니터링해야하는데. 범퍼가 실제로 눌려있는데도 update가 모니터보다 느린경우 범퍼데이터가 초기화 되버리기 때문에 둔턱회피에서 오동작 할수 있음.
	// 장애물 정보 업데이트 유무에 따라 제어 중 장애물 처리 싱크를 맞춰야 한다.
    //if(get_system_time()-updateLidarTime >= 0.1)
    if ( pExternData->systemData.isUpdateCliffData() )
    {        
        cliff.updateCliffSensor(pExternData->systemData.useCliffData(),
        pExternData->systemData.useCliffActionState());
        obstacle.cliff = cliff.getCliffMask();
    }

    if ( pExternData->systemData.isUpdateBumperData() )
    {
        bumper.updateBumperSensor(pExternData->systemData.useBumperData());
        obstacle.bumper = bumper.getBumperMask();
    }

    if ( pExternData->systemData.isUpdateFrontIRData() )
    {
        front.updateFrontSensor(pExternData->systemData.useFrontIRData());
        obstacle.front.obstacle = front.getFrontObstacleMask();
        obstacle.front.approach = front.getFrontApproachMask();
        obstacle.ir = front.getFrontSensor();
    }

    if ( pExternData->systemData.isUpdateImuData() )
    {
        imu.updateImuSensor(pExternData->systemData.useImuData());
        obstacle.imu = imu.getImuSensor();
    }

    if ( pExternData->systemData.isUpdateTofData() )
    {
        tof.updateTofSensorData(pExternData->systemData.useTofData(), pExternData->systemData.getCliffData());
        obstacle.tof.knoll = tof.getTofKnoll();
        obstacle.tof.leftwall = tof.getTofLeftWall();
        obstacle.tof.rightwall = tof.getTofRightWall();
        obstacle.tof.lcliff = tof.getLeftCliff();
        obstacle.tof.rcliff = tof.getRightCliff();
        obstacle.tof.leftCalibAvg = cliff.getLeftTofCalibAverage();
        obstacle.tof.rightCalibAvg = cliff.getRightTofCalibAverage();
        // hhryu231102 : tilt state를 판단해서 낙하 에러....
        if(!pExternData->systemData.getPowerData().adaptor_in)
        {
           tof.checkAndSendErrorMessage(pExternData->systemData.getTiltingData()); 
        }
    }

    lidar.updateLidarSensor(pExternData->rosData.lidar.getLidarDist());
    obstacle.lidar = lidar.getLidarMask();
    obstacle.left_wall = lidar.getLidarLeftWallMask();
    obstacle.right_wall = lidar.getLidarRightWallMask();
    updateLidarTime = get_system_time();

    if(pExternData->systemData.isUpdateWheelMotorData())
    {
        wheeltrap.updateWheelTrapData( pExternData->systemData.useWheelMotorData(), 
        pExternData->systemData.getSysPoseData());
        obstacle.wheel = wheeltrap.lastWheelData;
        obstacle.trap = wheeltrap.getWheelTrapMask();
    }

    if( pExternData->rosData.isUpdateFilterImu())
    {
        filterImu = pExternData->rosData.getFilterImu();
        pExternData->rosData.setUpdateFilterImu(false);
    }

    checkRobotLiftUpDown(pExternData->systemData.getTiltingData());
    buildObstacleData();
}

void CObstacle::initObstacleSensor()
{
    lidar.initLidarSensor();
    cliff.initCliffSensor();
    bumper.initBumperSensor();
    tof.initTofSensor();
    front.initFrontSensor();
    wheeltrap.initWheeltrapSensor();
    acc.value = 0;
    updateLidarTime = 0;
    obstacle.isLift = false;
    obstacle.approach = false;
}

int CObstacle::checkLidarSensorDetectedCount(int startDegree, int endDegree)
{
    return 0;//lidar.checkLidarSensorDetectedCount(startDegree, endDegree);
}


RSU_OBSTACLE_DATA* CObstacle::getObstacleData()
{
    return &obstacle;
}

bool CObstacle::getApproach()
{
    return obstacle.approach;
}
tSysIMU CObstacle::getFilterImuData()
{
    return filterImu;
}

void CObstacle::buildObstacleData()
{
    if(MOTION.isRunning())
    {
        if((obstacle.lidar.obstacle.value & 0x3C) || obstacle.front.approach.value /* || obstacle.tof.knoll.rangeAvg < TOF_CENTER_APPROACH*/)
        {
            // if(!obstacle.approach){
            //     //eblog(LOG_LV_NECESSARY,  "LIDAR APPROACH :" << SC<int>(obstacle.lidar.obstacle.value) << "FRONT APPROACH :" << SC<int>(obstacle.front.approach.value));
            // }
            obstacle.approach = true;
        }
        else
        {
            obstacle.approach = false;
        }
    }
}

bool CObstacle::isCliffAccumulate()
{
    return cliff.isAccumulateComplete();
}

void CObstacle::initCliffAccumulate()
{
    cliff.initItem();
}

void CObstacle::checkRobotLiftUpDown(tSysTilting tilt)
{
    if(!obstacle.isLift)
    {
        if(isRobotLifting(tilt))
        {
            safeCnt = 0;
            obstacle.isLift = true;
            ceblog(LOG_LV_NECESSARY,RED, "제품 들림 감지!! 제품을 안들었는데 이로그가 뜬다면 김환주 책임에게 알려주세요!!");
            MOTION.startStopOnMap(tProfile(),true);
            
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_ERROR, tMsgError(E_ERROR_TYPE::LIFT_CLIFF)));
            //ceblog(LOG_LV_NECESSARY, YELLOW, "Error Detected\t Cliff [" << left.rangeAvg << ", " << right.rangeAvg << "]");
        } 
    }
    else
    {
        if(isRobotOntheFloor(tilt))
        {
            ceblog(LOG_LV_NECESSARY,RED, "제품 내려놓음 감지!!");
            obstacle.isLift = false;
        } 
    }
}

bool CObstacle::isRobotLifting(tSysTilting tilt)
{
    bool ret = false;
    
    if(abs(imu.data.Ay) >= CONFIG.limitAccelY || abs(imu.data.Ax) >= CONFIG.limitAccelX)//&& (tof.lcliff.rangeAvg >= CONFIG.limitCliff || tof.rcliff.rangeAvg >= CONFIG.limitCliff)//tof.leftwall.rangeAvg >= CONFIG.limitWall || tof.rightwall.rangeAvg >= CONFIG.limitWall)
    {
        if(++suspectCnt2 >= 100)
        {
            suspectCnt2 = 0;
            #if defined (DRIVE_TEST_TASK) && (DRIVE_TEST_TASK == 0)
            ceblog(LOG_LV_NECESSARY,RED, "CASE-1 제품 들림 감지!! roll : " << (int)imu.data.Groll << "pitch : " << (int)imu.data.Gpitch << "Az : " << (int)imu.data.Az <<
            " Ax : " << (int)imu.data.Ax << " Ay : " << (int)imu.data.Ay << " tof LCliff : " << tof.left.rangeAvg <<" RCliff : " << tof.right.rangeAvg <<" center :"<<  tof.knoll.rangeAvg <<" LWall :"<< tof.leftwall.rangeAvg <<" RWall :"<< tof.rightwall.rangeAvg);
            #endif
            ret = true;
        }
        // else
        // {
        //     ceblog(LOG_LV_NECESSARY,YELLOW, "CASE-1 제품 들림 의심!!! roll : " << (int)imu.data.Groll << "pitch : " << (int)imu.data.Gpitch << "Az : " << (int)imu.data.Az <<
        //     " Ax : " << (int)imu.data.Ax << " Ay : " << (int)imu.data.Ay << " tof LCliff : " << tof.left.rangeAvg <<" RCliff : " << tof.right.rangeAvg << " suspectCnt2 :"<< suspectCnt2);
        // }
    }
    else
    {
        suspectCnt2 = 0;
    }
    
    if(!ROBOT_CONTROL.isTiltRunning())
    {
        if(tilt.state == E_SYS_TILT_STATE::TILTED_DOWN)
        {
            if(abs(imu.data.Gpitch) >= CONFIG.limitPitch || abs(imu.data.Groll) >= CONFIG.limitRoll || 
            (abs(imu.data.Az) >= CONFIG.limitAccelZ && (abs(imu.data.Gpitch)+abs(imu.data.Groll) >= 250)))//&& (tof.lcliff.rangeAvg >= CONFIG.limitCliff || tof.rcliff.rangeAvg >= CONFIG.limitCliff)//tof.leftwall.rangeAvg >= CONFIG.limitWall || tof.rightwall.rangeAvg >= CONFIG.limitWall)
            {
                if(++suspectCnt >= 10)
                {
                    suspectCnt = 0;
            #if defined (DRIVE_TEST_TASK) && (DRIVE_TEST_TASK == 0)
                    ceblog(LOG_LV_NECESSARY,RED, "CASE-2 제품 들림 감지!! roll : " << (int)imu.data.Groll << "pitch : " << (int)imu.data.Gpitch << "Az : " << (int)imu.data.Az <<
                    " Ax : " << (int)imu.data.Ax << " Ay : " << (int)imu.data.Ay << " tof LCliff : " << tof.left.rangeAvg <<" RCliff : " << tof.right.rangeAvg <<" center :"<<  tof.knoll.rangeAvg <<" LWall :"<< tof.leftwall.rangeAvg <<" RWall :"<< tof.rightwall.rangeAvg);
            #endif
                    ret = true;
                }
                // else
                // {
                //     ceblog(LOG_LV_NECESSARY,YELLOW, "CASE-2 제품 들림 의심!!! roll : " << (int)imu.data.Groll << "pitch : " << (int)imu.data.Gpitch << "Az : " << (int)imu.data.Az <<
                //     " Ax : " << (int)imu.data.Ax << " Ay : " << (int)imu.data.Ay << " tof LCliff : " << tof.left.rangeAvg <<" RCliff : " << tof.right.rangeAvg << " suspectCnt :"<< suspectCnt);
                // }
            }
            else
            {
                suspectCnt = 0;
            }
        }
        else if(tilt.state == E_SYS_TILT_STATE::TILTED_UP)
        {
            if(abs(imu.data.Gpitch) >= CONFIG.limitPitch || abs(imu.data.Groll) >= CONFIG.limitRoll || 
            (abs(imu.data.Az) >= CONFIG.limitAccelZ && (abs(imu.data.Gpitch)+abs(imu.data.Groll) >= 250)))//&& (tof.lcliff.rangeAvg >= CONFIG.limitCliff || tof.rcliff.rangeAvg >= CONFIG.limitCliff)//tof.leftwall.rangeAvg >= CONFIG.limitWall || tof.rightwall.rangeAvg >= CONFIG.limitWall)
            {
                if(++suspectCnt >= 10)
                {
                    suspectCnt = 0;
                    #if defined (DRIVE_TEST_TASK) && (DRIVE_TEST_TASK == 0)
                    ceblog(LOG_LV_NECESSARY,RED, "CASE-3 제품 들림 감지!! roll : " << (int)imu.data.Groll << "pitch : " << (int)imu.data.Gpitch << "Az : " << (int)imu.data.Az <<
                    " Ax : " << (int)imu.data.Ax << " Ay : " << (int)imu.data.Ay << " tof LCliff : " << tof.left.rangeAvg <<" RCliff : " << tof.right.rangeAvg <<" center :"<<  tof.knoll.rangeAvg <<" LWall :"<< tof.leftwall.rangeAvg <<" RWall :"<< tof.rightwall.rangeAvg);
                    #endif
                    ret = true;
                }
                // else
                // {
                //     ceblog(LOG_LV_NECESSARY,YELLOW, "CASE-3 제품 들림 의심!!! roll : " << (int)imu.data.Groll << "pitch : " << (int)imu.data.Gpitch << "Az : " << (int)imu.data.Az <<
                //     " Ax : " << (int)imu.data.Ax << " Ay : " << (int)imu.data.Ay << " tof LCliff : " << tof.left.rangeAvg <<" RCliff : " << tof.right.rangeAvg << " suspectCnt :"<< suspectCnt);
                // }
            }
            else
            {
                suspectCnt = 0;
            }
        }
        // else
        // {
        //     ceblog(LOG_LV_NECESSARY,RED, "틸팅 상태 오류!!");
        // }
    }

    return ret;
}

bool CObstacle::isRobotOntheFloor(tSysTilting tilt)
{
    bool ret = false;
    int checkAccelY = CONFIG.standAccelY;
    if(tilt.state == E_SYS_TILT_STATE::TILTED_UP) checkAccelY = CONFIG.tilUpstandAccelY;

    if(abs(imu.data.Ax) <= CONFIG.standAccelX && abs(imu.data.Ay) <= checkAccelY && imu.data.Az >= CONFIG.standAccelZ  && imu.data.Az <= CONFIG.limitAccelZ && 
    abs(imu.data.Groll) <= CONFIG.standRoll && abs(imu.data.Gpitch) <= CONFIG.standPitch)
    {
        if(++safeCnt >= 30)
        {
            safeCnt = 0;
            ret = true;
        }
    }
    else{
        safeCnt = 0;
    } 

    return ret;
}
