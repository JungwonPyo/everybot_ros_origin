#include "coordinate.h"
#include "eblog.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

tPoint getAreaCornerPoint(tPoint areaCenter, double width, double height, E_AREA_CORNERTYPE type)
{
    switch (type)
    {
    case E_AREA_CORNERTYPE::NONE:
        eblog(LOG_LV, "Area Corner Type is Wrong !!! None");
        return tPoint(0.0, 0.0);
        break;
    case E_AREA_CORNERTYPE::BOTTOM_LEFT:
        return tPoint(areaCenter.x - width/2, areaCenter.y + height/2);
        break;
    case E_AREA_CORNERTYPE::BOTTOM_RIGHT:
        return tPoint(areaCenter.x - width/2, areaCenter.y - height/2);
        break;
    case E_AREA_CORNERTYPE::TOP_LEFT:
        return tPoint(areaCenter.x + width/2, areaCenter.y + height/2);
        break;
    case E_AREA_CORNERTYPE::TOP_RIGHT:
        return tPoint(areaCenter.x + width/2, areaCenter.y - height/2);
        break;
    
    default:
        eblog(LOG_LV, "area Corner Type is Wrong !!! default");
        return tPoint(0.0, 0.0);
        break;
    }
}

/**
 * @brief bresenhan 알고리즘에 따라 두 좌표 사이의 점들을 리턴
 * 
 * @param startPoint 
 * @param endPoint 
 * @param resolution 좌표의 해상도 (단위 m/cell)
 * @return std::list<tPoint> 
 */
std::list<tPoint> generatePointsWithBresenham(tPoint startPoint, tPoint endPoint, double resolution)
{
    std::list<tPoint> linePoints;

    int startX  = startPoint.x / resolution;
    int startY  = startPoint.y / resolution;
    int endX    = endPoint.x / resolution;
    int endY    = endPoint.y / resolution;
    
    int dX = endX - startX;
    int dY = endY - startY;

    // if startX == endX, then it does not matter what we set here
    signed char const ix((dX > 0) - (dX < 0));
    dX = std::abs(dX) * 2;

    // if startY == endY, then it does not matter what we set here
    signed char const iy((dY > 0) - (dY < 0));
    dY = std::abs(dY) * 2;

    linePoints.emplace_back(tPoint(startX*resolution, startY*resolution));

    if (dX >= dY)
    {
        // error may go below zero
        int error(dY - (dX >> 1));
        while (startX != endX)
        {
            if ((0 <= error) && (error || (0 < ix)))
            {
                error -= dX;
                startY += iy;
            }
            // else do nothing
            error += dY;
            startX += ix;
            linePoints.emplace_back(tPoint(startX*resolution, startY*resolution));
        }
    }
    else
    {
        // error may go below zero
        int error(dX - (dY >> 1));

        while (startY != endY)
        {
            if ((0 <= error) && (error || (0 < iy)))
            {
                error -= dY;
                startX += ix;
            }
            // else do nothing
            error += dX;
            startY += iy;
            linePoints.emplace_back(tPoint(startX*resolution, startY*resolution));
        }
    }

    return linePoints;
}