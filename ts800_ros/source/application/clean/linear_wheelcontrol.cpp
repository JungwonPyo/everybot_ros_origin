#include "linear_wheelcontrol.h"
#include "eblog.h"
#include <math.h>
#include "control/motionPlanner/motionPlanner.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

CLinearWheelControl::CLinearWheelControl(/* args */)
{
    CStopWatch __debug_sw;

    rsuInitLinearCtrl();
    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}
CLinearWheelControl::~CLinearWheelControl()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV,  "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

void CLinearWheelControl::getFreq()
{
    CStopWatch __debug_sw;

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief 컨트롤러 사용전 반드시 1회 호출.
 * 
 */
void CLinearWheelControl::rsuInitLinearCtrl()
{
    CStopWatch __debug_sw;

    io_linear_xd = 0.;
    io_linear_yd = 0.;
    dtTerm = 0;        
    
    TIME_CHECK_END(__debug_sw.getTime());
}

double CLinearWheelControl::rsuLinearController ( tPoint currentPoint, double currentAngleRad, tPoint targetPoint, double freq )
{
    CStopWatch __debug_sw;

    double xd, yd, xdd, ydd; //, thd;
    double v;	// 산출된 속도.
    double w;	//산출된 각속도.
    double dt;	// 미분. 
    s32 linearSpeed = 0; //각속도를 좌,우 휠속도로 환산 하기위한 초기 기중 속도..
    s32   l, r;

    xd = targetPoint.x;
    yd = targetPoint.y;

    dt = (double)(1.0/(double)freq); // 목표 좌표에 얼만큼 빨리 도달할지를 정할수 있어용. cheol.

    /*
    360 - currentAngle : 시계방향 각도에서 반시계 방향 각도로 변환.
    +90 : rsu_LineIoLinearization() 에서 연산 되는 식은 x축 방향으로 헤딩이 초기화 됨.
    cheol.        
    currentAngle = 360 - currentAngle + 90;
    */
    
    xdd = (xd - io_linear_xd)/dt; // x 축 2차 미분.
    ydd = (yd - io_linear_yd)/dt;// y 축 2차 미분.
    //xdd = (xd - io_linear_xd);
    //ydd = (yd - io_linear_yd);

    
    io_linear_xd = xd; //x축 1차 미분 저장 
    io_linear_yd = yd; //y축 1차 미분 저장     

    dtTerm += dt;	// 미분 시간을 저장 한다.


    if (dtTerm < 3*dt) 
    {        
        return 0;//현재 좌표 x,y 를 미분 xd,yd 다시 미분 xdd,ydd 를 얻기  위해 3회 이전에는 리턴 한다. cheol
    }
            
    //속도, 각속도 를 획득. 
    rsuLineIoLinearization ( &v, &w, xd, yd, xdd, ydd, currentAngleRad, currentPoint);

  
       
    //MOTION.startCurveVelocity(v,w);
    
    
    TIME_CHECK_END(__debug_sw.getTime());
    return w*(-1);
}

void CLinearWheelControl::rsuLineIoLinearization (double *v, double *w, double xd, double yd, double xdd, double ydd, double currentAngleRad, tPoint currentPoint)
{
    CStopWatch __debug_sw;

    double       theta; //radian value    . cheol.
    double b = 220; // b 점은 바퀴보다 앞에 있어야 한다. RP1000 은 (로봇 지름 / 2) 로 결정 하겠다.. cheol 
    double y1=0,y2=0,y1d=0,y2d=0,y1dd=0,y2dd=0,k1=0,k2=0,u1=0,u2=0; 

    theta = currentAngleRad;

    y1 = currentPoint.x + b*cos(theta);
    y2 = currentPoint.y + b*sin(theta);

    y1d = xd;
    y2d = yd;

    y1dd = xdd;
    y2dd = ydd;

    // k1,k2 > 0 일 때 제어기의 궤적 추종 오차가 0으로 수렴함 . cheol.
    k1 = 1.5; 
    k2 = 1.5; 

    //
    u1 = y1dd + k1*(y1d - y1);
    u2 = y2dd + k2*(y2d - y2);

    *v =  u1*cos(theta)   + u2*sin(theta);
    *w = -u1*sin(theta)/b + u2*cos(theta)/b;
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief TODO : 목표 좌표 생성
 * 
 * @param currentPoint 
 * @param xd 
 * @param yd 
 */
void CLinearWheelControl::rsuGetLineIoDesiredPos (tPoint currentPoint, double *xd, double *yd )
{
    CStopWatch __debug_sw;

    //TODO : 현제 좌표를 이요하여 목표 좌표를 생성한다.        

    TIME_CHECK_END(__debug_sw.getTime());
}

