/**
 * @file signals.cpp
 * @author hhryu (hhryu@everybot.net)
 * @brief TS450 rsf_Signals.c를 포팅한 파일.
 * @version 0.1
 * @date 2023-01-18
 * @copyright Copyright (c) 2023
 *
 */
#include "coreData/serviceData/signals.h"
#include "eblog.h"
#include "systemTool.h"
#include "externData/externData.h"
#include "signals.h"
#include <algorithm>
#include <utils.h>

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0           // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)                                     \
    {                                                            \
        printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time); \
    }                                                            \
    0
/******************************************************/
u8 number = 0;
CSignal::CSignal()
{
    receiveData_ = {0}; // cppcheck230316 : Message: Member variable 'CSignal::receiveData_' is not initialized in the constructor.
    cradleData_.ackFan = false;
    cradleData_.checkTime = 0;
    signalData_ = {0};
    signalData_.signalCheckStartTime = std::numeric_limits<double>::infinity();
    signalData_.signalCheckEndTime = std::numeric_limits<double>::infinity();
    sigTime.debugFrontLeft = 0.0;
    sigTime.debugFrontRight = 0.0;
    sigTime.debugSideLeft = 0.0;
    sigTime.debugSideRight = 0.0;
    // _signalArray = {std::numeric_limits<u16>::infinity()};
    for(u8 idx = 0; idx < RECEIVER_IDX; idx++)
    {
        for (u8 i = 0; i < arraySize; ++i) {
            _signalArray[idx].receiver[i] = std::numeric_limits<u8>::max();
        }
    }
    for (u8 i = 0; i < RECEIVER_IDX; i++)
    {
        sigList[i].dataTimePairs.clear();
    }
    setCheckSignalState(E_CHECK_SIGNAL::IDLE);
    eblog(LOG_LV_NECESSARY, "");
}
CSignal::~CSignal()
{
    eblog(LOG_LV_NECESSARY, "");
}

void CSignal::setCheckSignalState(E_CHECK_SIGNAL set)
{
    checkSignalState = set;
}

E_CHECK_SIGNAL CSignal::getCheckSignalState()
{
    return checkSignalState;
}

void CSignal::update(CExternData *pExternData)
{
    // ceblog(LOG_LV_DOCKING, BLUE, "update time [" << SYSTEM_TOOL.getSystemTime() << "]");
    std::list<tSysSignal> sig;
    sig.clear();
    
    if(pExternData->systemData.isUpdateSignalData())
    {
        sig = pExternData->systemData.useSignalData();
    }

    updateSignal(sig,pExternData->rosData.getSlamPose());
}

bool CSignal::getFanAck()
{
    bool ret = cradleData_.ackFan;
    cradleData_.ackFan = false;
    return ret;
}

void CSignal::setSignalCheckStart(bool set)
{
    signalData_.bSignalCheckStart = set;
}

bool CSignal::getSignalCheckStart()
{
    return signalData_.bSignalCheckStart;
}

bool CSignal::getSignalCheckComplete()
{
    return signalData_.bSignalCheckComplete;
}

void CSignal::setSignalCheckComplete(bool set)
{
    signalData_.bSignalCheckComplete = set;
}

tSignalArray* CSignal::getSignalArray(u8 receiver)
{
    return &_signalArray[receiver];
}

void CSignal::updateSignal(const std::list<tSysSignal>& sysSignal, tPose robotPose)
{
#if 0 // 시그널 가공 전 디버깅 용
    debugSignal(sysSignal);
#endif
    updateSignalList(sysSignal,robotPose);
    //debugSignalPrint();
#if 0 // 시그널 가공 후 디버깅 용
    ceblog(LOG_LV_TEST, BLUE, "IDX_RECEIVER_FRONT_LEFT의 값 bin : " << BINARY(receiverDetected(IDX_RECEIVER_FRONT_LEFT)));
    buildTryDockAngle();
    debugSignalPrint();
#endif
}

void CSignal::debugSignal(const std::list<tSysSignal>& sysSignal)
{
    double present = SYSTEM_TOOL.getSystemTime();
    double diff = 0;
    /* debug */
    for(tSysSignal sig : sysSignal)
    {
        for(int idx = 0; idx < 4; idx++)
        {
            int receiveCount = sig.ChargerIR[idx].count; // Queue data 개수 체크
            
            if (receiveCount < SYS_MAX_SIGNAL_COUNT)
            {
                if (receiveCount > 0) // 담을 정보가 있을 시
                {
                    for(u32 i = 0; i < receiveCount; i++)
                    {
                        u8 data = sig.ChargerIR[idx].Data[i]; // 충전기로부터 받은 신호의 정보
                        std::string receiverColor;
                        std::string signalColor;

                        if (idx == 0){
                            receiverColor = BLUE;
                            diff = present - sigTime.debugFrontLeft;
                            sigTime.debugFrontLeft = present;
                        } else if (idx == 1){
                            receiverColor = MAGENTA;
                            diff = present - sigTime.debugFrontRight;
                            sigTime.debugFrontRight = present;
                        } else if (idx == 2){
                            receiverColor = GREEN;
                            diff = present - sigTime.debugSideLeft;
                            sigTime.debugSideLeft = present;                        
                        } else if (idx == 3){
                            receiverColor = CYN;
                            diff = present - sigTime.debugSideRight;
                            sigTime.debugSideRight = present;                        
                        }

                        switch (data)
                        {
                        case SIGNAL_CENTER_SHORT:
                            signalColor = BLUE;
                            break;
                        case SIGNAL_CENTER_LONG:
                            signalColor = BOLDBLUE;
                            break;
                        case SIGNAL_RIGHT_CENTER_SHORT:
                            signalColor = MAGENTA;
                            break;
                        case SIGNAL_RIGHT_CENTER_LONG:
                            signalColor = BOLDMAGENTA;
                            break;
                        case SIGNAL_LEFT_CENTER_SHORT:
                            signalColor = GREEN;
                            break;
                        case SIGNAL_LEFT_CENTER_LONG:
                            signalColor = BOLDGREEN;
                            break;
                        case SIGNAL_RIGHT_SIDE_SHORT:
                            signalColor = CYN;
                            break;
                        case SIGNAL_RIGHT_SIDE_LONG:
                            signalColor = BOLDCYAN;
                            break;
                        case SIGNAL_LEFT_SIDE_SHORT:
                            signalColor = GRAY;
                            break;
                        case SIGNAL_LEFT_SIDE_LONG:
                            signalColor = BOLDBLACK;
                            break;
                        default:
                            break;
                        }

                        if (diff >= 0.4)
                        {
                            ceblog (LOG_LV_DOCKING, RED, receiverColor.c_str() << "detector[" <<  SC<int>(idx) << signalColor.c_str() << "]\tdata[" << data << RED << "]\tdiffTime : " << diff << "\ttime : "<< PRECISION(5) << SYSTEM_TOOL.getSystemTime());
                        }
                        else if (diff > 0.2)
                        {
                            // ceblog (LOG_LV_DOCKING, YELLOW, receiverColor.c_str() << "detector[" <<  SC<int>(idx) << signalColor.c_str() << "]\tdata[" << data <<  YELLOW <<"]\tdiffTime : " << diff << "\ttime : "<< PRECISION(5) << SYSTEM_TOOL.getSystemTime());    
                        }
                        else
                        {
                            // ceblog (LOG_LV_DOCKING, GREEN, receiverColor.c_str() << "detector[" <<  SC<int>(idx) << signalColor.c_str() << "]\tdata[" << data <<  YELLOW <<"]\tdiffTime : " << diff << "\ttime : "<< PRECISION(5) << SYSTEM_TOOL.getSystemTime());    
                        }
                    }
                }
            }
        }

        {
            // ceblog (LOG_LV_DOCKING, BOLDBLACK, "No signal \ttime : "<< PRECISION(5) << SYSTEM_TOOL.getSystemTime());
        }
    }
}

/**
 * @brief 매 디스패쳐 주기 마다 인터페이스에서 쌓아 둔 신호를 받아온다. 이 때, ??시간 만큼 지난 데이터는 삭제한다.
 * @param sysSignal 
 * 
 * @note 연산시간 ms
 * @date 2023-01-09
 * @author hhryu
 */
void CSignal::updateSignalList(const std::list<tSysSignal>& sysSignal,tPose robotPose)
{
    double currentTime = SYSTEM_TOOL.getSystemTime();

    if(!sysSignal.empty())
    {
        if(sysSignal.size() > 2)
        {
            ceblog((LOG_LV_DOCKING | LOG_LV_NECESSARY | LOG_LV_ERROR), BOLDRED, " APPLICATION DATA UPDATE DELAY!!! SYS_SIGNAL Q_SIZE : " << (int)sysSignal.size());
        }
        for(tSysSignal sig : sysSignal)
        {
            for (u8 receiver = 0; receiver < RECEIVER_IDX; receiver++)
            {
                u32 count = sig.ChargerIR[receiver].count;
                for (u32 i = 0; i < count; ++i)
                {
                    double timeStamp = sig.ChargerIR[receiver].timeStamp[i];
                    if (currentTime - timeStamp < SIGNAL_LIFE_TIME)
                    {
                        u8 data = sig.ChargerIR[receiver].Data[i];
                        if (data != ACK_FAN)
                        {
                            sigList[receiver].dataTimePairs.push_back(std::make_pair(data, timeStamp));
                        }
                        else
                        {
                            cradleData_.ackFan = true;
                        }
                        updateTriggerSignalList(receiver, data, true);
                    }
                }
            }
        }
    }
    updateTriggerSignalList(0, 0, false);
    for (u8 receiver = 0; receiver < RECEIVER_IDX; receiver++)
    {
        auto &dataTimePairs = sigList[receiver].dataTimePairs;
        auto removeBegin = std::remove_if(dataTimePairs.begin(), dataTimePairs.end(), [&](const std::pair<u8, double> &pair)
                                            { return currentTime - pair.second > SIGNAL_LIFE_TIME; });
        dataTimePairs.erase(removeBegin, dataTimePairs.end());
    }
    
    updateSignalCheckPose(robotPose);
    // for (const auto& pair : sigList[0].dataTimePairs)
    // {
    //     u8 data = pair.first;
    //     double time = pair.second;        
    //     ceblog(LOG_LV_DOCKING, BLUE, "data : " << data << "\ttime : " << PRECISION(2) << time << "\tsize : " << sigList[0].dataTimePairs.size());
    // }
    // ceblog(LOG_LV_DOCKING, RED, "===END=== \ttime : " << PRECISION(2) << SYSTEM_TOOL.getSystemTime());
}

void CSignal::updateTriggerSignalList(u8 receiver, u8 data, bool bSignal)
{
    if (SYSTEM_TOOL.getSystemTime() - signalData_.signalCheckStartTime >= getCheckSignalEndTime()
    && getCheckSignalState() != E_CHECK_SIGNAL::IDLE
    && getCheckSignalState() != E_CHECK_SIGNAL::START)
    { // 200ms 이상.
        setCheckSignalState(E_CHECK_SIGNAL::END);
    }

    switch (getCheckSignalState())
    {
    case E_CHECK_SIGNAL::IDLE:
        break;
    case E_CHECK_SIGNAL::START: // 시간 및 데이터 초기화.
        signalData_.signalCheckStartTime = SYSTEM_TOOL.getSystemTime();
        clearSignalVector();
        setCheckSignalState(E_CHECK_SIGNAL::ING);
        // ceblog(LOG_LV_DOCKING, GREEN, "signal Check Start [" << PRECISION(5) << SYSTEM_TOOL.getSystemTime() << "]");
        break;
    case E_CHECK_SIGNAL::ING: // 정보 담음.
        if (bSignal)
        {
            _signalVector[receiver].receiver.push_back(data);
        }
        setSignalCheckComplete(false);
        // ceblog(LOG_LV_DOCKING, CYN, "signal Checking~ [" << PRECISION(5) << SYSTEM_TOOL.getSystemTime() << "]");
        break;
    case E_CHECK_SIGNAL::END:
        if (bSignal)
        {
            _signalVector[receiver].receiver.push_back(data);
        }
        // setCheckSignalState(E_CHECK_SIGNAL::IDLE);
        // ceblog(LOG_LV_DOCKING, YELLOW, "signal Check END [" << PRECISION(5) << SYSTEM_TOOL.getSystemTime() << "]");
        break;
    default:
        break;
    }

    return;
}

/**
 * @brief vector는 계속 업데이트를 하지 않는다.
 * 
 * @param sysSignal 
 * 
 * @note 연산시간 ms
 * @date 2023-01-09
 * @author hhryu
 */
void CSignal::updateVector(const std::list<tSysSignal>& sysSignal)
{
    if (SYSTEM_TOOL.getSystemTime() - signalData_.signalCheckStartTime >= getCheckSignalEndTime()
    && getCheckSignalState() != E_CHECK_SIGNAL::IDLE
    && getCheckSignalState() != E_CHECK_SIGNAL::START)
    { // 200ms 이상.
        setCheckSignalState(E_CHECK_SIGNAL::END);
    }

    switch (getCheckSignalState())
    {
    case E_CHECK_SIGNAL::IDLE:
        break;
    case E_CHECK_SIGNAL::START: // 시간 및 데이터 초기화.
        signalData_.signalCheckStartTime = SYSTEM_TOOL.getSystemTime();
        clearSignalVector();
        setCheckSignalState(E_CHECK_SIGNAL::ING);
        // ceblog(LOG_LV_DOCKING, GREEN, "signal Check Start [" << PRECISION(5) << SYSTEM_TOOL.getSystemTime() << "]");
        break;
    case E_CHECK_SIGNAL::ING: // 정보 담음.
        pushSignalVector(sysSignal);
        setSignalCheckComplete(false);
        // ceblog(LOG_LV_DOCKING, CYN, "signal Checking~ [" << PRECISION(5) << SYSTEM_TOOL.getSystemTime() << "]");
        break;
    case E_CHECK_SIGNAL::END:
        signalData_.signalCheckStartTime = SYSTEM_TOOL.getSystemTime();
        pushSignalVector(sysSignal);
        // setCheckSignalState(E_CHECK_SIGNAL::IDLE);
        ceblog(LOG_LV_DOCKING, YELLOW, "signal Check END [" << PRECISION(5) << SYSTEM_TOOL.getSystemTime() << "]");
        break;
    default:
        break;
    }

    return;
}

void CSignal::clearSignalVector()
{
    for(u8 index = 0; index < RECEIVER_IDX; index++)
    {
        if (!_signalVector[index].receiver.empty())
        {
            _signalVector[index].receiver.clear();
        }
    }    
}

void CSignal::pushSignalVector(const std::list<tSysSignal>& sysSignal)
{
    if(!sysSignal.empty())
    {
        for(tSysSignal sig : sysSignal)
        {
            for (u8 index = 0; index < RECEIVER_IDX; index++)
            {
                s32 receiveCount = sig.ChargerIR[index].count;
                while (receiveCount-- >= 0)
                {
                    u8 data = sig.ChargerIR[index].Data[receiveCount];
                    _signalVector[index].receiver.push_back(data);
                }
            }
        }
    }

    return;
}

void CSignal::setCheckSignalEndTime(double time)
{
    if (time < SIGNAL_LIFE_TIME)
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_ERROR|LOG_LV_DOCKING), RED, "[ERROR] " << time << "초는 신호 관리 주기인 " << SIGNAL_LIFE_TIME << "초보다 낮을 수 없습니다.");
    }
    signalData_.signalCheckEndTime = time;
}

double CSignal::getCheckSignalEndTime()
{
    return signalData_.signalCheckEndTime;
}

u32 CSignal::countSignalVector(u8 receiver, u8 signal)
{
    u32 count = 0;

    if(receiver == IDX_RECEIVER_VOID)
    {
        ceblog(LOG_LV_DOCKING, BOLDGREEN, "Error!! your checking receiver is VOID");
    }
    else if(receiver == IDX_RECEIVER_ANYTHING)
    {
        for(u8 idx = 0; idx < RECEIVER_IDX; idx++)
        {
            for (const auto& value : _signalVector[idx].receiver) 
            {
                if(signal == 0 || signal == SIGNAL_BLANK)
                {
                    if(signal == 0)
                    {
                        // ceblog(LOG_LV_DOCKING, BOLDGREEN, "Error!! your checking Signal is VOID");
                    }
                    else
                    {
                        ceblog(LOG_LV_DOCKING, BOLDGREEN, "Error!! your checking Signal is BLANK");
                    }
                }
                else if (signal == SIGNAL_ANYTHING && value) ++count;
                else if (value == static_cast<u8>(signal)) ++count;
            }
        }
    }
    else
    {
        u8 idx = static_cast<u8>(receiver);
        for (const auto& value : _signalVector[idx].receiver)
        {
            if(signal == 0 || signal == SIGNAL_BLANK)
            {
                if(signal == 0)
                {
                    ceblog((LOG_LV_DOCKING | LOG_LV_ERROR), BOLDRED, "Error!! your checking Signal is VOID");
                }
                else
                {
                    ceblog((LOG_LV_DOCKING | LOG_LV_ERROR), BOLDRED, "Error!! your checking Signal is BLANK");
                }
            }
            else if (signal == SIGNAL_ANYTHING && value)  ++count;
            else if (value == static_cast<u8>(signal)) ++count;
        }
    }

    if(count && (signal == SIGNAL_LEFT_CENTER_SHORT || signal == SIGNAL_RIGHT_CENTER_SHORT || signal == SIGNAL_CENTER_SHORT))
    {
        ceblog(LOG_LV_DOCKING,CYN, "멈춤 신호 수집 시 리시버 " << SC<s32>(receiver) << "에 감지되는 " << SC<u8>(signal) << "의 개수 : " << SC<s32>(count) << "\ttime : " << SYSTEM_TOOL.getSystemTime() );
    }
    
    return count;
}

void CSignal::checkAngleParameterBySignal(double *angleK , double *min, double *max)
{
    // bool bNarrow = false, bNormal = false, bWide = false, bExtraWide = false, bWideRange = false, bExtreme = false;
    // bool bBeyondExtreme = false;  // -50도 ~ 50도를 벗어나는 경우
    if ((countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))
    || (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT)))
    {
        *angleK *= 1;
        *min = -5;
        *max = 5;
    }
    else if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))
    {
        *angleK *= 2;
        *min = -10;
        *max = 10;
    }
    else if ((countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT) && !countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT)) 
    || (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT) && !countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT)))
    {
        *angleK *= 2.7;
        *min = -15;
        *max = 15;
    }
    else if ((countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT) && !countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT))
    || (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT) && !countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)))
    {
        *angleK *= 3.5;
        *min = -20;
        *max = 20;
    }
    else if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT)
    || countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))
    {
        *angleK *= 5;
        *min = -30;
        *max = 30;
    }
    else if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT)
    || countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))
    {
        *angleK *= 8;
        *min = -50;
        *max = 50;
    }


    if ((countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG))
    || (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG)))
    {
        *angleK *= 1;
        *min = -5;
        *max = 5;
    }
    else if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG))
    {
        *angleK *= 2;
        *min = -10;
        *max = 10;
    }
    else if ((countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) && !countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG)) 
    || (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG) && !countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG)))
    {
        *angleK *= 2.7;
        *min = -15;
        *max = 15;
    }
    else if ((countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG) && !countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG))
    || (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG) && !countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG) && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG)))
    {
        *angleK *= 3.5;
        *min = -20;
        *max = 20;
    }
    else if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG)
    || countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG))
    {
        *angleK *= 5;
        *min = -30;
        *max = 30;
    }
    else if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG)
    || countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG))
    {
        *angleK *= 8;
        *min = -50;
        *max = 50;
    }
    return;
}

double CSignal::getAngleByShortSignal(double angleK, double sideK, double centerLRK, double centerK, double min, double max)
{
    double ret = 0;
    if (max > 10)
    {
        ret -= (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT)    * angleK * sideK);
        ret += (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT)   * angleK * sideK);
        ret -= (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT)   * angleK * sideK);
        ret += (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT)  * angleK * sideK);
    }
    ret -= (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT)      * angleK * centerLRK);
    ret += (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT)     * angleK * centerLRK);
    ret -= (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT)     * angleK * centerLRK);
    ret += (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)    * angleK * centerLRK);
    
    ret += (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT)           * angleK * centerK);
    ret -= (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT)          * angleK * centerK);
    ret = utils::math::clamp(ret, min, max);
    if(!countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING) 
    || !countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING))
    {
        // 편심인 경우 각도를 두배로.
        if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING))  ret = 40;
        if (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING)) ret = -40;
    }
    return ret;
}

double CSignal::getAngleByLongSignal(double angleK, double sideK, double centerLRK, double centerK, double min, double max)
{
    double ret = 0;
    if (max > 10)
    {
        ret -= (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG)    * angleK * sideK);
        ret += (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG)   * angleK * sideK);
        ret -= (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG)   * angleK * sideK);
        ret += (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG)  * angleK * sideK);
    }
    ret -= (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG)      * angleK * centerLRK);
    ret += (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG)     * angleK * centerLRK);
    ret -= (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG)     * angleK * centerLRK);
    ret += (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG)    * angleK * centerLRK);

    ret += (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG)           * angleK * centerK);
    ret -= (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG)          * angleK * centerK);
    ret = utils::math::clamp(ret, min, max);
    if(!countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING)
    || !countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING))
    {
        // 편심인 경우 각도를 두배로 --> 잘못된 결과가 나올 수 있다, 편심인 경우에는 회전필요
        if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING))  ret = 40;
        if (countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING)) ret = -40;
    }
    return ret;
}

double CSignal::getAngleBySignal(double angleK, double min, double max)
{
    double ret = 0;
    const double sideK = 1, centerLRK = 1, centerK = 1.5; // 사이드, 센터 좌우, 센터 각자에 대해 가중치를 정함.
    const double k = 1; // max나 min을 무시하고 값을 올려주는 비례상수
    if (!countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_SHORT_ANYTHING) && !countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_SHORT_ANYTHING))
    {
        // 이 때는 롱을 참고하도록 해보자.
        if (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_ANYTHING) || countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_ANYTHING))
        {
            ret += getAngleByLongSignal(angleK, sideK, centerLRK, centerK, min, max);
            ret *= k;
        }
        else if (countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING) && countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING))
        {
            ceblog(LOG_LV_DOCKING, RED, "신호 판단 불가-양쪽 사이드 수신부에 신호 들어옴");
            ret = std::numeric_limits<double>::infinity(); // 신호없음
        }
        else if (countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING))
        {
            ceblog(LOG_LV_DOCKING, YELLOW, "오른쪽 사이드만 신호 들어옴 값 : " << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_ANYTHING)));
            ret = std::numeric_limits<double>::infinity(); // 신호없음
        }
        else if(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING))
        {
            ceblog(LOG_LV_DOCKING, YELLOW, "왼쪽 사이드만 신호 들어옴 값 : " << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_ANYTHING)));
            ret = std::numeric_limits<double>::infinity(); // 신호없음
        }
        else
        {
            ceblog(LOG_LV_DOCKING, YELLOW, "아무 신호도 없음");
            ret = std::numeric_limits<double>::infinity();
        }
        // ceblog(LOG_LV_DOCKING, RED, "Front[0,0]!! Angle[" << tryDockAngle << "]\tleftSideCNT[" << leftSideCount_.b.ls << "," << leftSideCount_.b.lc << ", " << leftSideCount_.b.sc << ", " << leftSideCount_.b.rc << ", " << leftSideCount_.b.rs << "]\trightSideCNT[" << rightSideCount_.b.ls << ", " << rightSideCount_.b.lc << ", " << rightSideCount_.b.sc << ", " << rightSideCount_.b.rc << ", " << rightSideCount_.b.rs << "]\tangleK,min,max[" << angleK << ", " << min << ", " << max << "]\ttime[" <<PRECISION(5) << SYSTEM_TOOL.getSystemTime() << "]");
    }
    else
    {
        ret += getAngleByShortSignal(angleK, sideK, centerLRK, centerK, min, max);
        ret *= k;
    }
    return ret;
}

/**
 * @brief try dock할 때의 크래들과 로봇의 각도를 신호를 통해 추정.
 * 
 * @return double 
 * 
 * @note 연산시간 ms
 * @date 2023-MM-DD
 * @author hhryu
 */
double CSignal::buildTryDockAngle()
{
    double tryDockAngle = 0; // std::numeric_limits<double>::infinity();
    double angleK = 2;//1.29;
    double min = 0, max = 0;
    bool bNoSignal = false;
    
    checkAngleParameterBySignal(&angleK,&min,&max);
    tryDockAngle = getAngleBySignal(angleK,min,max);
    ceblog(LOG_LV_DOCKING, CYN, "Try dock Angle[" << tryDockAngle << "]");
    return DEG2RAD(tryDockAngle);
}

/**
 * @brief try dock할 때의 크래들과 로봇의 각도를 신호를 통해 추정.
 * 
 * @return double 
 * 
 * @note 연산시간 ms
 * @date 2023-MM-DD
 * @author hhryu
 */
double CSignal::buildTryDockAngle(tPose robotPose)
{
    double tryDockAngle = 0;    // std::numeric_limits<double>::infinity();
    double angleK = 2;          //1.29;
    double min = 0, max = 0;
    bool bNoSignal = false;
    checkAngleParameterBySignal(&angleK,&min,&max);
    tryDockAngle = getAngleBySignal(angleK,min,max);
    double angle = RAD2DEG(robotPose.angle);
    std::string color;
    
    if (abs(tryDockAngle) < std::numeric_limits<double>::epsilon()) color = WHITE;
    else if (abs(tryDockAngle) >= 40)                               color = RED;
    else if (tryDockAngle > 0)                                      color = YELLOW;
    else if (tryDockAngle < 0)                                      color = GREEN;

#if 0
    ceblog((LOG_LV_DOCKING|LOG_LV_TEST), color.c_str(), angle << "," << tryDockAngle << "," << (tryDockAngle+angle) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG)) << ","
    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG)) << ","
    << robotPose.x << "," << robotPose.y << ","
    << SYSTEM_TOOL.getSystemTime());
#endif

    return tryDockAngle;
}

/**
 * @brief 신호가 감지되었을 때의 좌표를 저장한다.
 *
 * @param robotPose
 *
 * @note 연산시간 ms
 * @date 2023-10-18
 * @author hhryu
 */
void CSignal::updateSignalCheckPose(tPose robotPose)
{
    if (countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_CENTER_SHORT))
    {
        if (!signalCheckPoseData_.bCenter){
            ceblog(LOG_LV_DOCKING, BLUE, "Center신호를 통한 충전기 업데이트 좌표[" << robotPose.x << ", " << robotPose.y << "]");
        }
        signalCheckPoseData_.bCenter = true;
        signalCheckPoseData_.centerPose = robotPose;
    }
    else if (!(signalCheckPoseData_.bCenter))
    {
        bool bCenterCheck = \
        ((countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT)
        && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))
        || (countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG)
        && countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG)));
        if (bCenterCheck)
        {
            if (!signalCheckPoseData_.bCenter){
                ceblog(LOG_LV_DOCKING, BLUE, "Center신호를 통한 충전기 업데이트 좌표[" << robotPose.x << ", " << robotPose.y << "]");
            }
            signalCheckPoseData_.bCenter = true;
            signalCheckPoseData_.centerPose = robotPose;
        }
    }
    
    if (countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_SHORT)
    || countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_LEFT_CENTER_LONG))
    {
        if (!signalCheckPoseData_.bLeftSide){
            ceblog(LOG_LV_DOCKING, BLUE, "LeftSide신호를 통한 충전기 업데이트 좌표[" << robotPose.x << ", " << robotPose.y << "]");
        }
        signalCheckPoseData_.bLeftSide = true;
        signalCheckPoseData_.leftSidePose = robotPose;
    }
    else if (countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_SHORT)
    || countSignalDetected(IDX_RECEIVER_ANYTHING, SIGNAL_RIGHT_CENTER_LONG))
    {
        if (!signalCheckPoseData_.bRightSide){
            ceblog(LOG_LV_DOCKING, BLUE, "RightSide신호를 통한 충전기 업데이트 좌표[" << robotPose.x << ", " << robotPose.y << "]");
        }
        signalCheckPoseData_.bRightSide = true;
        signalCheckPoseData_.rightSidePose = robotPose;
    }
}

/**
 * @brief 신호가 감지되었을 때의 좌표 반환.
 *
 * @return tSignalCheckPose
 *
 * @note 연산시간 ms
 * @date 2023-10-18
 * @author hhryu
 */
tSignalCheckPose CSignal::getSignalCheckPose()
{
    return signalCheckPoseData_;
}

u16 CSignal::processReceiver(const tSignalList& list)
{
    u16 ret = 0;
    for (const auto& pair : list.dataTimePairs)
    {
        u8 data = pair.first;
        switch (data)
        {
            case SIGNAL_CENTER_SHORT:
                ret |= IS_SIGNAL_CENTER_SHORT;
                break;
            case SIGNAL_RIGHT_CENTER_SHORT:
                ret |= IS_SIGNAL_RIGHT_CENTER_SHORT;
                break;
            case SIGNAL_LEFT_CENTER_SHORT:
                ret |= IS_SIGNAL_LEFT_CENTER_SHORT;
                break;
            case SIGNAL_RIGHT_SIDE_SHORT:
                ret |= IS_SIGNAL_RIGHT_SIDE_SHORT;
                break;
            case SIGNAL_LEFT_SIDE_SHORT:
                ret |= IS_SIGNAL_LEFT_SIDE_SHORT;
                break;
            case SIGNAL_CENTER_LONG:
                ret |= IS_SIGNAL_CENTER_LONG;
                break;
            case SIGNAL_RIGHT_CENTER_LONG:
                ret |= IS_SIGNAL_RIGHT_CENTER_LONG;
                break;
            case SIGNAL_LEFT_CENTER_LONG:
                ret |= IS_SIGNAL_LEFT_CENTER_LONG;
                break;
            case SIGNAL_RIGHT_SIDE_LONG:
                ret |= IS_SIGNAL_RIGHT_SIDE_LONG;
                break;
            case SIGNAL_LEFT_SIDE_LONG:
                ret |= IS_SIGNAL_LEFT_SIDE_LONG;
                break;
        default:
            break;
        }
    }
    return ret;
}

u16 CSignal::searchShortSignal(const tSignalList& recevierList)
{
    u16 ret = 0;
    for (const auto& pair : recevierList.dataTimePairs)
    {
        u8 data = pair.first;
        switch (data)
        {
            case SIGNAL_CENTER_SHORT:
                ret |= IS_SIGNAL_CENTER_SHORT;
                break;
            case SIGNAL_RIGHT_CENTER_SHORT:
                ret |= IS_SIGNAL_RIGHT_CENTER_SHORT;
                break;
            case SIGNAL_LEFT_CENTER_SHORT:
                ret |= IS_SIGNAL_LEFT_CENTER_SHORT;
                break;
            case SIGNAL_RIGHT_SIDE_SHORT:
                ret |= IS_SIGNAL_RIGHT_SIDE_SHORT;
                break;
            case SIGNAL_LEFT_SIDE_SHORT:
                ret |= IS_SIGNAL_LEFT_SIDE_SHORT;
                break;
        default:
            break;
        }
    }
    return ret;
}

u16 CSignal::searchLongSignal(const tSignalList& recevierList)
{
    u16 ret = 0;
    for (const auto& pair : recevierList.dataTimePairs)
    {
        u8 data = pair.first;
        switch (data)
        {
            case SIGNAL_CENTER_LONG:
                ret |= IS_SIGNAL_CENTER_LONG;
                break;
            case SIGNAL_RIGHT_CENTER_LONG:
                ret |= IS_SIGNAL_RIGHT_CENTER_LONG;
                break;
            case SIGNAL_LEFT_CENTER_LONG:
                ret |= IS_SIGNAL_LEFT_CENTER_LONG;
                break;
            case SIGNAL_RIGHT_SIDE_LONG:
                ret |= IS_SIGNAL_RIGHT_SIDE_LONG;
                break;
            case SIGNAL_LEFT_SIDE_LONG:
                ret |= IS_SIGNAL_LEFT_SIDE_LONG;
                break;
        default:
            break;
        }
    }
    return ret;
}

/**
 * @brief 원하는 리시버를 인자로 넣어주면, 그 리시버가 받은 신호를 return한다.
 * Ex) if (receiverDetected(IDX_RECEIVER_FRONT_LEFT) & IS_SIGNAL_LEFT_CENTER_SHORT){}
 * 위는 왼쪽 리시버가 왼쪽 센터 숏을 받고 있다는 뜻.
 * Ex2) if (receiverDetected(IDX_RECEIVER_FRONT_LEFT) & (IS_SIGNAL_LEFT_CENTER_SHORT | IS_SIGNAL_LEFT_CENTER_LONG)){}
 * @param receiver 
 * @return u16 
 * 
 * @note 연산시간 ms
 * @date 2024-04-16
 * @author hhryu
 */
u16 CSignal::receiverDetected(u8 receiver)
{
    u16 ret = 0;

    if (receiver != IDX_RECEIVER_ANYTHING && receiver != IDX_RECEIVER_VOID)
    {
        ret = processReceiver(sigList[receiver]);
    }
    else if (receiver == IDX_RECEIVER_ANYTHING)
    {
        for (u8 i = IDX_RECEIVER_FRONT_LEFT; i <= IDX_RECEIVER_SIDE_RIGHT; i++)
        {
            ret = processReceiver(sigList[i]);
        }
    }
    else
    {
        ceblog(LOG_LV_ERROR, RED, "ERROR");
    }

    return ret;
}
u16 CSignal::getReceivedShortSignalData(u8 receiver)
{
    u16 ret = 0;

    if (receiver != IDX_RECEIVER_ANYTHING && receiver != IDX_RECEIVER_VOID)
    {
        ret = searchShortSignal(sigList[receiver]);
    }
    else
    {
        ceblog(LOG_LV_ERROR, RED, "receiver error!! you should select only one receiver!!");
    }

    return ret;
}

u16 CSignal::getReceivedLongSignalData(u8 receiver)
{
    u16 ret = 0;

    if (receiver != IDX_RECEIVER_ANYTHING && receiver != IDX_RECEIVER_VOID)
    {
        ret = searchLongSignal(sigList[receiver]);
    }
    else
    {
        ceblog(LOG_LV_ERROR, RED, "receiver error!! you should select only one receiver!!");
    }

    return ret;
}

u16 CSignal::signalConvertingBitFlag(u8 signal)
{
    u16 bitFlag = 0;
    switch (signal)
    {
    case SIGNAL_CENTER_SHORT:
        bitFlag = IS_SIGNAL_CENTER_SHORT;
        break;
    case SIGNAL_RIGHT_CENTER_SHORT:
        bitFlag = IS_SIGNAL_RIGHT_CENTER_SHORT;
        break;
    case SIGNAL_LEFT_CENTER_SHORT:
        bitFlag = IS_SIGNAL_LEFT_CENTER_SHORT;
        break;
    case SIGNAL_RIGHT_SIDE_SHORT:
        bitFlag = IS_SIGNAL_RIGHT_SIDE_SHORT;
        break;
    case SIGNAL_LEFT_SIDE_SHORT:
        bitFlag = IS_SIGNAL_LEFT_SIDE_SHORT;
        break;
    case SIGNAL_CENTER_LONG:
        bitFlag = IS_SIGNAL_CENTER_LONG;
        break;
    case SIGNAL_RIGHT_CENTER_LONG:
        bitFlag = IS_SIGNAL_RIGHT_CENTER_LONG;
        break;
    case SIGNAL_LEFT_CENTER_LONG:
        bitFlag = IS_SIGNAL_LEFT_CENTER_LONG;
        break;
    case SIGNAL_RIGHT_SIDE_LONG:
        bitFlag = IS_SIGNAL_RIGHT_SIDE_LONG;
        break;
    case SIGNAL_LEFT_SIDE_LONG:
        bitFlag = IS_SIGNAL_LEFT_SIDE_LONG;
        break;
    default:
        // Handle error or other cases
        break;
    }
    return bitFlag;
}

/**
 * @brief 리시버와 flag화 된 signal을 받아서
 * 있는지 없는지 검사.
 * 
 * @param receiver 
 * @param checkSignal 
 * @return true 
 * @return false 
 * 
 * @note 연산시간 ms
 * @date 2023-MM-DD
 * @author hhryu
 */
bool CSignal::countBitSignalDetectedCASE1(u8 receiver, u16 checkSignal)
{
    u16 dataCount = 0;

    if (receiver != IDX_RECEIVER_ANYTHING && receiver != IDX_RECEIVER_VOID)
    {
        for (const auto& pair : sigList[receiver].dataTimePairs)
        {
            u8 data = pair.first;
            u16 flagData = signalConvertingBitFlag(data);

            if (flagData & checkSignal || checkSignal == SIGNAL_ANYTHING)
            {
                ++dataCount;
            }
            else if (checkSignal == SIGNAL_SHORT_ANYTHING)
            {
                if(flagData & IS_SIGNAL_CENTER_SHORT
                || flagData & IS_SIGNAL_RIGHT_CENTER_SHORT
                || flagData & IS_SIGNAL_LEFT_CENTER_SHORT
                || flagData & IS_SIGNAL_RIGHT_SIDE_SHORT
                || flagData & IS_SIGNAL_LEFT_SIDE_SHORT){
                    ++dataCount;
                }
            }
            else if (checkSignal == SIGNAL_LONG_ANYTHING)
            {
                if(flagData & IS_SIGNAL_CENTER_LONG
                || flagData & IS_SIGNAL_RIGHT_CENTER_LONG
                || flagData & IS_SIGNAL_LEFT_CENTER_LONG
                || flagData & IS_SIGNAL_RIGHT_SIDE_LONG
                || flagData & IS_SIGNAL_LEFT_SIDE_LONG){
                    ++dataCount;
                }
            }
        }
    }
    else
    {
        if (receiver == IDX_RECEIVER_ANYTHING)
        {
            for (u8 idx = 0; idx < RECEIVER_IDX; idx++)
            {
                for (const auto& pair : sigList[idx].dataTimePairs)
                {
                    u8 data = pair.first;
                    u16 flagData = signalConvertingBitFlag(data);
                    if (flagData & checkSignal || checkSignal == SIGNAL_ANYTHING)
                    {
                        ++dataCount;
                    }
                    else if (checkSignal == SIGNAL_SHORT_ANYTHING)
                    {
                        if(flagData & IS_SIGNAL_CENTER_SHORT
                        || flagData & IS_SIGNAL_RIGHT_CENTER_SHORT
                        || flagData & IS_SIGNAL_LEFT_CENTER_SHORT
                        || flagData & IS_SIGNAL_RIGHT_SIDE_SHORT
                        || flagData & IS_SIGNAL_LEFT_SIDE_SHORT){
                            ++dataCount;
                        }
                    }
                    else if (checkSignal == SIGNAL_LONG_ANYTHING)
                    {
                        if(flagData & IS_SIGNAL_CENTER_LONG
                        || flagData & IS_SIGNAL_RIGHT_CENTER_LONG
                        || flagData & IS_SIGNAL_LEFT_CENTER_LONG
                        || flagData & IS_SIGNAL_RIGHT_SIDE_LONG
                        || flagData & IS_SIGNAL_LEFT_SIDE_LONG){
                            ++dataCount;
                        }
                    }                    
                }
            }
        }else{
            ceblog(LOG_LV_ERROR, RED, "ERROR");
        }
    }

    return SC<bool>(dataCount);
}

bool CSignal::isFlagSig(u8 receiver, u16 checkSig)
{
    bool bRet = false;
    for (const auto& pair : sigList[receiver].dataTimePairs)
    {
        u8 data = pair.first;
        u16 flagData = signalConvertingBitFlag(data);
        if (flagData & checkSig) bRet = true;
    }
    return bRet;
}

u8 CSignal::countBitSignalDetectedCASE2(u8 receiver, u16 checkSignal)
{
    u16 dataCount = 0;
    if (checkSignal == SIGNAL_ANYTHING)         checkSignal = 0b1111111111; // IS_SIGNAL_CENTER_SHORT | IS_SIGNAL_CENTER_SHORT 등.... 으로 하는게 가독성이 더 좋을까요?
    if (checkSignal == SIGNAL_SHORT_ANYTHING)   checkSignal = 0b0000011111;
    if (checkSignal == SIGNAL_LONG_ANYTHING)    checkSignal = 0b1111100000;
    if (receiver != IDX_RECEIVER_ANYTHING && receiver != IDX_RECEIVER_VOID)
    {
        if (checkSignal & IS_SIGNAL_CENTER_SHORT)                   { dataCount += isFlagSig(receiver, IS_SIGNAL_CENTER_SHORT); }
        if (checkSignal & IS_SIGNAL_RIGHT_CENTER_SHORT)             { dataCount += isFlagSig(receiver, IS_SIGNAL_RIGHT_CENTER_SHORT); }
        if (checkSignal & IS_SIGNAL_LEFT_CENTER_SHORT)              { dataCount += isFlagSig(receiver, IS_SIGNAL_LEFT_CENTER_SHORT); }
        if (checkSignal & IS_SIGNAL_RIGHT_SIDE_SHORT)               { dataCount += isFlagSig(receiver, IS_SIGNAL_RIGHT_SIDE_SHORT); }
        if (checkSignal & IS_SIGNAL_LEFT_SIDE_SHORT)                { dataCount += isFlagSig(receiver, IS_SIGNAL_LEFT_SIDE_SHORT); }
        if (checkSignal & IS_SIGNAL_CENTER_LONG)                    { dataCount += isFlagSig(receiver, IS_SIGNAL_CENTER_LONG); }
        if (checkSignal & IS_SIGNAL_RIGHT_CENTER_LONG)              { dataCount += isFlagSig(receiver, IS_SIGNAL_RIGHT_CENTER_LONG); }
        if (checkSignal & IS_SIGNAL_LEFT_CENTER_LONG)               { dataCount += isFlagSig(receiver, IS_SIGNAL_LEFT_CENTER_LONG); }
        if (checkSignal & IS_SIGNAL_RIGHT_SIDE_LONG)                { dataCount += isFlagSig(receiver, IS_SIGNAL_RIGHT_SIDE_LONG); }
        if (checkSignal & IS_SIGNAL_LEFT_SIDE_LONG)                 { dataCount += isFlagSig(receiver, IS_SIGNAL_LEFT_SIDE_LONG); }
    }
    else
    {
        if (receiver == IDX_RECEIVER_ANYTHING)
        {
            for (u8 idx = 0; idx < RECEIVER_IDX; idx++)
            {
                if (checkSignal & IS_SIGNAL_CENTER_SHORT)           { dataCount += isFlagSig(idx, IS_SIGNAL_CENTER_SHORT); }
                if (checkSignal & IS_SIGNAL_RIGHT_CENTER_SHORT)     { dataCount += isFlagSig(idx, IS_SIGNAL_RIGHT_CENTER_SHORT); }
                if (checkSignal & IS_SIGNAL_LEFT_CENTER_SHORT)      { dataCount += isFlagSig(idx, IS_SIGNAL_LEFT_CENTER_SHORT); }
                if (checkSignal & IS_SIGNAL_RIGHT_SIDE_SHORT)       { dataCount += isFlagSig(idx, IS_SIGNAL_RIGHT_SIDE_SHORT); }
                if (checkSignal & IS_SIGNAL_LEFT_SIDE_SHORT)        { dataCount += isFlagSig(idx, IS_SIGNAL_LEFT_SIDE_SHORT); }
                if (checkSignal & IS_SIGNAL_CENTER_LONG)            { dataCount += isFlagSig(idx, IS_SIGNAL_CENTER_LONG); }
                if (checkSignal & IS_SIGNAL_RIGHT_CENTER_LONG)      { dataCount += isFlagSig(idx, IS_SIGNAL_RIGHT_CENTER_LONG); }
                if (checkSignal & IS_SIGNAL_LEFT_CENTER_LONG)       { dataCount += isFlagSig(idx, IS_SIGNAL_LEFT_CENTER_LONG); }
                if (checkSignal & IS_SIGNAL_RIGHT_SIDE_LONG)        { dataCount += isFlagSig(idx, IS_SIGNAL_RIGHT_SIDE_LONG); }
                if (checkSignal & IS_SIGNAL_LEFT_SIDE_LONG)         { dataCount += isFlagSig(idx, IS_SIGNAL_LEFT_SIDE_LONG); }
            }
        }else{
            ceblog(LOG_LV_ERROR, RED, "ERROR");
        }
    }

    return dataCount;
}

u8 CSignal::countBitSignalDetectedCASE3(u8 receiver, u16 checkSignal)
{
    u16 dataCount = 0;

    if (receiver != IDX_RECEIVER_ANYTHING && receiver != IDX_RECEIVER_VOID)
    {
        for (const auto& pair : sigList[receiver].dataTimePairs)
        {
            u8 data = pair.first;
            u16 flagData = signalConvertingBitFlag(data);

            if (flagData &checkSignal || checkSignal == SIGNAL_ANYTHING)
            {
                ++dataCount;
            }
            else if (checkSignal == SIGNAL_SHORT_ANYTHING)
            {
                if(flagData & IS_SIGNAL_CENTER_SHORT
                || flagData & IS_SIGNAL_RIGHT_CENTER_SHORT
                || flagData & IS_SIGNAL_LEFT_CENTER_SHORT
                || flagData & IS_SIGNAL_RIGHT_SIDE_SHORT
                || flagData & IS_SIGNAL_LEFT_SIDE_SHORT){
                    ++dataCount;
                }
            }
            else if (checkSignal == SIGNAL_LONG_ANYTHING)
            {
                if(flagData & IS_SIGNAL_CENTER_LONG
                || flagData & IS_SIGNAL_RIGHT_CENTER_LONG
                || flagData & IS_SIGNAL_LEFT_CENTER_LONG
                || flagData & IS_SIGNAL_RIGHT_SIDE_LONG
                || flagData & IS_SIGNAL_LEFT_SIDE_LONG){
                    ++dataCount;
                }
            }
        }
    }
    else
    {
        if (receiver == IDX_RECEIVER_ANYTHING)
        {
            for (u8 idx = 0; idx < RECEIVER_IDX; idx++)
            {
                for (const auto& pair : sigList[idx].dataTimePairs)
                {
                    u8 data = pair.first;
                    u16 flagData = signalConvertingBitFlag(data);
                    if (flagData & checkSignal || checkSignal == SIGNAL_ANYTHING)
                    {
                        ++dataCount;
                    }
                    else if (checkSignal == SIGNAL_SHORT_ANYTHING)
                    {
                        if(flagData & IS_SIGNAL_CENTER_SHORT
                        || flagData & IS_SIGNAL_RIGHT_CENTER_SHORT
                        || flagData & IS_SIGNAL_LEFT_CENTER_SHORT
                        || flagData & IS_SIGNAL_RIGHT_SIDE_SHORT
                        || flagData & IS_SIGNAL_LEFT_SIDE_SHORT){
                            ++dataCount;
                        }
                    }
                    else if (checkSignal == SIGNAL_LONG_ANYTHING)
                    {
                        if(flagData & IS_SIGNAL_CENTER_LONG
                        || flagData & IS_SIGNAL_RIGHT_CENTER_LONG
                        || flagData & IS_SIGNAL_LEFT_CENTER_LONG
                        || flagData & IS_SIGNAL_RIGHT_SIDE_LONG
                        || flagData & IS_SIGNAL_LEFT_SIDE_LONG){
                            ++dataCount;
                        }
                    }                    
                }
            }
        }else{
            ceblog(LOG_LV_ERROR, RED, "ERROR");
        }
    }

    return dataCount;
}

u8 CSignal::countSignalDetected(u8 receiver, u8 signal)
{
    u8 dataCount = 0;

    if (receiver != IDX_RECEIVER_ANYTHING && receiver != IDX_RECEIVER_VOID)
    {
        for (const auto& pair : sigList[receiver].dataTimePairs)
        {
            u8 data = pair.first;
            if (signal == data || signal == SIGNAL_ANYTHING)
            {
                ++dataCount;
            }
            else if (signal == SIGNAL_SHORT_ANYTHING)
            {
                if(data == SIGNAL_CENTER_SHORT
                || data == SIGNAL_RIGHT_CENTER_SHORT
                || data == SIGNAL_LEFT_CENTER_SHORT
                || data == SIGNAL_RIGHT_SIDE_SHORT
                || data == SIGNAL_LEFT_SIDE_SHORT){
                    ++dataCount;
                }
            }
            else if (signal == SIGNAL_LONG_ANYTHING)
            {
                if(data == SIGNAL_CENTER_LONG
                || data == SIGNAL_RIGHT_CENTER_LONG
                || data == SIGNAL_LEFT_CENTER_LONG
                || data == SIGNAL_RIGHT_SIDE_LONG
                || data == SIGNAL_LEFT_SIDE_LONG){
                    ++dataCount;
                }
            }
        }
    }
    else
    {
        if (receiver == IDX_RECEIVER_ANYTHING)
        {
            for (u8 idx = 0; idx < RECEIVER_IDX; idx++)
            {
                for (const auto& pair : sigList[idx].dataTimePairs)
                {
                    u8 data = pair.first;
                    if (signal == data || signal == SIGNAL_ANYTHING)
                    {
                        ++dataCount;
                    }
                    else if (signal == SIGNAL_SHORT_ANYTHING)
                    {
                        if(data == SIGNAL_CENTER_SHORT
                        || data == SIGNAL_RIGHT_CENTER_SHORT
                        || data == SIGNAL_LEFT_CENTER_SHORT
                        || data == SIGNAL_RIGHT_SIDE_SHORT
                        || data == SIGNAL_LEFT_SIDE_SHORT){
                            ++dataCount;
                        }
                    }
                    else if (signal == SIGNAL_LONG_ANYTHING)
                    {
                        if(data == SIGNAL_CENTER_LONG
                        || data == SIGNAL_RIGHT_CENTER_LONG
                        || data == SIGNAL_LEFT_CENTER_LONG
                        || data == SIGNAL_RIGHT_SIDE_LONG
                        || data == SIGNAL_LEFT_SIDE_LONG){
                            ++dataCount;
                        }
                    }                    
                }
            }
        }
        else
        {
            ceblog(LOG_LV_ERROR, RED, "ERROR");
        }
    }

    return dataCount;
}

void CSignal::debugSignalPrint()
{
    std::string color[RECEIVER_IDX][0xFF];

    for (int i = 0; i < RECEIVER_IDX; ++i)
    {
        for (int j = 0; j < 0xFF; ++j)
        {
            switch (countSignalDetected(i, j))
            {
            case 0:
                color[i][j] = BOLDBLACK;
                break;
            case 1:
                color[i][j] = WHITE;
                break;
            case 2:
                color[i][j] = CYN;
                break;
            case 3:
                color[i][j] = YELLOW;
                break;                                            
            default:
                color[i][j] = RED;
                break;
            }
        }
    }

    if(countSignalDetected(IDX_RECEIVER_ANYTHING,SIGNAL_SHORT_ANYTHING))
    {
        ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, "|Short Signal\tSL[" 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_SIDE_SHORT].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_SHORT))         << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_CENTER_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_SHORT))       << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_CENTER_SHORT].c_str()           << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT))            << BOLDMAGENTA <<", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_CENTER_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_SIDE_SHORT].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_SHORT))        << BOLDMAGENTA <<"] FL["
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_SIDE_SHORT].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT))        << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_CENTER_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_CENTER_SHORT].c_str()          << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT))           << BOLDMAGENTA <<", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_CENTER_SHORT].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_SIDE_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT))       << BOLDMAGENTA <<"] FR["
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_SIDE_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT))       << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_CENTER_SHORT].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_CENTER_SHORT].c_str()         << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))          << BOLDMAGENTA <<", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_CENTER_SHORT].c_str()   << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))    << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_SIDE_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))      << BOLDMAGENTA <<"] SR["
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_SIDE_SHORT].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_SHORT))        << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_CENTER_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_CENTER_SHORT].c_str()          << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT))           << BOLDMAGENTA <<", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_CENTER_SHORT].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_SIDE_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))       << BOLDMAGENTA 
        <<"] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
    }

    if(countSignalDetected(IDX_RECEIVER_ANYTHING,SIGNAL_LONG_ANYTHING))
    {
        ceblog((LOG_LV_NECESSARY|LOG_LV_DOCKING), BOLDBLUE, "|Long Signal\tSL[" 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_SIDE_LONG].c_str()         << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_LONG))          << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_CENTER_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_LONG))        << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_CENTER_LONG].c_str()            << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG))             << BOLDBLUE <<", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_CENTER_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_LONG))       << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_SIDE_LONG].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_LONG))         << BOLDBLUE <<"] FL["
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_SIDE_LONG].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG))         << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_CENTER_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG))       << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_CENTER_LONG].c_str()           << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG))            << BOLDBLUE <<", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_CENTER_LONG].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG))      << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_SIDE_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG))        << BOLDBLUE <<"] FR["
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_SIDE_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG))        << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_CENTER_LONG].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG))      << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_CENTER_LONG].c_str()          << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG))           << BOLDBLUE <<", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_CENTER_LONG].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG))     << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_SIDE_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG))       << BOLDBLUE <<"] SR["
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_SIDE_LONG].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_LONG))         << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_CENTER_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_LONG))       << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_CENTER_LONG].c_str()           << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG))            << BOLDBLUE <<", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_CENTER_LONG].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_LONG))      << BOLDBLUE << ", " 
        << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_SIDE_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_LONG))        << BOLDBLUE 
        <<"] time[" << SYSTEM_TOOL.getSystemTime()<<"]");
    }
    
    // if (!utils::isMaxValue(buildTryDockAngle()))
    // {
    //     ceblog((LOG_LV_NECESSARY|LOG_LV_DOCKING), BOLDBLACK, "충전기와 로봇 추정 상대 degree:" << WHITE << buildTryDockAngle());
    // }

    return;
}

void CSignal::debugSignalPrint(const std::string& str)
{
    std::string color[RECEIVER_IDX][0xFF];

    for (int i = 0; i < RECEIVER_IDX; ++i)
    {
        for (int j = 0; j < 0xFF; ++j)
        {
            switch (countSignalDetected(i, j))
            {
            case 0:
                color[i][j] = BOLDBLACK;
                break;
            case 1:
                color[i][j] = WHITE;
                break;
            case 2:
                color[i][j] = CYN;
                break;
            case 3:
                color[i][j] = YELLOW;
                break;                                            
            default:
                color[i][j] = RED;
                break;
            }
        }
    }

    ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, "|Short Signal\tSL[" 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_SIDE_SHORT].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_SHORT))         << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_CENTER_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_SHORT))       << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_CENTER_SHORT].c_str()           << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT))            << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_CENTER_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_SIDE_SHORT].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_SHORT))        << BOLDMAGENTA <<"] FL["
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_SIDE_SHORT].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT))        << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_CENTER_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_CENTER_SHORT].c_str()          << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT))           << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_CENTER_SHORT].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_SIDE_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT))       << BOLDMAGENTA <<"] FR["
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_SIDE_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT))       << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_CENTER_SHORT].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_CENTER_SHORT].c_str()         << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))          << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_CENTER_SHORT].c_str()   << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))    << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_SIDE_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))      << BOLDMAGENTA <<"] SR["
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_SIDE_SHORT].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_SHORT))        << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_CENTER_SHORT].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_CENTER_SHORT].c_str()          << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT))           << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_CENTER_SHORT].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_SIDE_SHORT].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))       << BOLDMAGENTA 
    <<"] time[" << SYSTEM_TOOL.getSystemTime()<<"] msg[" << str << "]");

    ceblog((LOG_LV_NECESSARY|LOG_LV_DOCKING), BOLDBLUE, "|Long Signal\tSL[" 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_SIDE_LONG].c_str()         << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_LONG))          << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_CENTER_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_LONG))        << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_CENTER_LONG].c_str()            << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG))             << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_CENTER_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_LONG))       << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_SIDE_LONG].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_LONG))         << BOLDBLUE <<"] FL["
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_SIDE_LONG].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG))         << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_CENTER_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG))       << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_CENTER_LONG].c_str()           << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG))            << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_CENTER_LONG].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG))      << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_SIDE_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG))        << BOLDBLUE <<"] FR["
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_SIDE_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG))        << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_CENTER_LONG].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG))      << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_CENTER_LONG].c_str()          << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG))           << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_CENTER_LONG].c_str()    << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG))     << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_SIDE_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG))       << BOLDBLUE <<"] SR["
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_SIDE_LONG].c_str()        << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_LONG))         << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_CENTER_LONG].c_str()      << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_LONG))       << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_CENTER_LONG].c_str()           << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG))            << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_CENTER_LONG].c_str()     << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_LONG))      << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_SIDE_LONG].c_str()       << DEC(countSignalDetected(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_LONG))        << BOLDBLUE 
    <<"] time[" << SYSTEM_TOOL.getSystemTime()<<"] msg[" << str << "]");

    return;
}


void CSignal::debugTriggerSignalPrint(const std::string& str)
{
    std::string color[RECEIVER_IDX][0xFF];

    for (int i = 0; i < RECEIVER_IDX; ++i)
    {
        for (int j = 0; j < 0xFF; ++j)
        {
            switch (countSignalVector(i, j))
            {
            case 0:
                color[i][j] = BOLDBLACK;
                break;
            case 1:
                color[i][j] = WHITE;
                break;
            case 2:
                color[i][j] = CYN;
                break;
            case 3:
                color[i][j] = YELLOW;
                break;                                            
            default:
                color[i][j] = RED;
                break;
            }
        }
    }

    ceblog((LOG_LV_NECESSARY | LOG_LV_DOCKING), BOLDMAGENTA, "|Short Signal\tSL[" 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_SIDE_SHORT].c_str()        << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_SHORT))         << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_CENTER_SHORT].c_str()      << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_SHORT))       << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_CENTER_SHORT].c_str()           << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_SHORT))            << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_CENTER_SHORT].c_str()     << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_SIDE_SHORT].c_str()       << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_SHORT))        << BOLDMAGENTA <<"] FL["
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_SIDE_SHORT].c_str()       << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_SHORT))        << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_CENTER_SHORT].c_str()     << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_CENTER_SHORT].c_str()          << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_SHORT))           << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_CENTER_SHORT].c_str()    << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_SIDE_SHORT].c_str()      << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_SHORT))       << BOLDMAGENTA <<"] FR["
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_SIDE_SHORT].c_str()      << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_SHORT))       << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_CENTER_SHORT].c_str()    << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_CENTER_SHORT].c_str()         << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_SHORT))          << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_CENTER_SHORT].c_str()   << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))    << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_SIDE_SHORT].c_str()     << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))      << BOLDMAGENTA <<"] SR["
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_SIDE_SHORT].c_str()       << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_SHORT))        << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_CENTER_SHORT].c_str()     << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_SHORT))      << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_CENTER_SHORT].c_str()          << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_SHORT))           << BOLDMAGENTA <<", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_CENTER_SHORT].c_str()    << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_SHORT))     << BOLDMAGENTA << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_SIDE_SHORT].c_str()      << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_SHORT))       << BOLDMAGENTA 
    <<"] time[" << SYSTEM_TOOL.getSystemTime()<<"] msg[" << str << "]");

    ceblog((LOG_LV_NECESSARY|LOG_LV_DOCKING), BOLDBLUE, "|Long Signal\tSL[" 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_SIDE_LONG].c_str()         << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_SIDE_LONG))          << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_LEFT_CENTER_LONG].c_str()       << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_LEFT_CENTER_LONG))        << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_CENTER_LONG].c_str()            << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_CENTER_LONG))             << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_CENTER_LONG].c_str()      << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_CENTER_LONG))       << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_LEFT][SIGNAL_RIGHT_SIDE_LONG].c_str()        << DEC(countSignalVector(IDX_RECEIVER_SIDE_LEFT, SIGNAL_RIGHT_SIDE_LONG))         << BOLDBLUE <<"] FL["
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_SIDE_LONG].c_str()        << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_SIDE_LONG))         << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_LEFT_CENTER_LONG].c_str()      << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_LEFT_CENTER_LONG))       << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_CENTER_LONG].c_str()           << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_CENTER_LONG))            << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_CENTER_LONG].c_str()     << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_CENTER_LONG))      << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_LEFT][SIGNAL_RIGHT_SIDE_LONG].c_str()       << DEC(countSignalVector(IDX_RECEIVER_FRONT_LEFT, SIGNAL_RIGHT_SIDE_LONG))        << BOLDBLUE <<"] FR["
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_SIDE_LONG].c_str()       << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_SIDE_LONG))        << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_LEFT_CENTER_LONG].c_str()     << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_LEFT_CENTER_LONG))      << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_CENTER_LONG].c_str()          << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_CENTER_LONG))           << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_CENTER_LONG].c_str()    << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_CENTER_LONG))     << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_FRONT_RIGHT][SIGNAL_RIGHT_SIDE_LONG].c_str()      << DEC(countSignalVector(IDX_RECEIVER_FRONT_RIGHT, SIGNAL_RIGHT_SIDE_LONG))       << BOLDBLUE <<"] SR["
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_SIDE_LONG].c_str()        << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_SIDE_LONG))         << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_LEFT_CENTER_LONG].c_str()      << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_LEFT_CENTER_LONG))       << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_CENTER_LONG].c_str()           << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_CENTER_LONG))            << BOLDBLUE <<", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_CENTER_LONG].c_str()     << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_CENTER_LONG))      << BOLDBLUE << ", " 
    << color[IDX_RECEIVER_SIDE_RIGHT][SIGNAL_RIGHT_SIDE_LONG].c_str()       << DEC(countSignalVector(IDX_RECEIVER_SIDE_RIGHT, SIGNAL_RIGHT_SIDE_LONG))        << BOLDBLUE 
    <<"] time[" << SYSTEM_TOOL.getSystemTime()<<"] msg[" << str << "]");

    return;
}
