#include "imgProcessor.h"
#include "rosParameter.h"
#include "eblog.h"
#include "define.h"
#include "systemTool.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 1 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 150.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/
using namespace cv;

CImgProcessor::CImgProcessor()
{
    CStopWatch __debug_sw;
    
    eblog(LOG_LV, "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}

CImgProcessor::~CImgProcessor()
{
    CStopWatch __debug_sw;

    eblog(LOG_LV, "");
    
    TIME_CHECK_END(__debug_sw.getTime());
}



/**
 * @brief cleanMap Data 를 Mat 파일로 변환
 * E_CELL_TYPE들을 Gray image 화 시킨다.
 * icbaek - 23-01-03     
 * @param src       cleanMap sorce
 * @param x     cleanMap width
 * @param y    cleanMap height
 * @param mask  남기고 싶은 비트
 * @return Mat 
 */
Mat CImgProcessor::convertCleanMap2cvImg(unsigned char *src, int x, int y, unsigned char addMask, unsigned char deleteMask)
{
    CStopWatch __debug_sw;

    unsigned char srcTemp[x*y];
    int idx = 0;
    for (int xCnt = 0; xCnt < x; xCnt++)
    {
        for (int yCnt = 0; yCnt < y; yCnt++)
        {
            if((src[idx] & addMask) && !(src[idx] & deleteMask))
            {
                srcTemp[idx] = 110;
            }
            else
            {
                srcTemp[idx] = 0;
            }
            idx++;
        }
    }

    Mat cvImg = Mat(x,y,CV_8U,srcTemp);

    TIME_CHECK_END(__debug_sw.getTime());
    return cvImg;
}


/**
 * @brief cleanMap Data 를 Mat 파일로 변환
 * E_CELL_TYPE들을 Gray image 화 시킨다.
 * icbaek - 23-01-03     
 * @param src       cleanMap sorce
 * @param x     cleanMap width
 * @param y    cleanMap height
 * @return Mat 
 */
Mat CImgProcessor::convertCleanMap2cvImg(unsigned char *src, int x, int y)
{
    CStopWatch __debug_sw;

    unsigned char srcTemp[x*y]; 
    int idx = 0; 
    for (int xCnt = 0; xCnt < x; xCnt++) 
    { 
        for (int yCnt = 0; yCnt < y; yCnt++) 
        {
            srcTemp[idx] = GRAY_LV_UNKNOWN_AREA;

            if(src[idx] == (unsigned char)CELL_TYPE_KNOWN)
            {
                srcTemp[idx] = GRAY_LV_KNOWN_AREA;
            }
            if (src[idx] == (unsigned char)CELL_TYPE_WALL)
            {
                srcTemp[idx] = GRAY_LV_KNOWN_WALL;
            }
            idx++; 
        } 
    } 

    Mat cvImg = Mat(x,y,CV_8U,srcTemp);
    
    TIME_CHECK_END(__debug_sw.getTime());
    return cvImg; 
}

/**
 * @brief 확률 그리드 지도를 Mat 형식으로 변환
 * 
 * @param pSrc 
 * @param srcWidth 
 * @param srcHeight 
 * @param threshVal 
 * @return Mat
 * @date 23.09.01
 * @note 연산시간 : 0.9ms ~ 57.9ms (연산시간 변동 요인은 아직 미확인.)
 */
Mat CImgProcessor::convertGridMap2Mat(s8 *pSrc, int srcWidth, int srcHeight, int threshVal)
{
    CStopWatch __debug_sw;

    //SLAM 에서 나온 확률 그리드 값을 이미지로 변환하기위해 데이터 타입 변환
    int size = srcWidth * srcHeight;
    unsigned char ocuupancyCvtbuf[size];    
    s8 occupy = 0; 
    for (int i = 0;i < size; i++)
    {
        occupy = pSrc[i];

        if (occupy == -1 || occupy == 255 )
        {
            ocuupancyCvtbuf[i] = GRAY_LV_UNKNOWN_AREA;
        }
        else if ( occupy <= 10 ) // 10% 이하만 빈 영역으로 처리 22.06.22 jspark
        {
            ocuupancyCvtbuf[i] = GRAY_LV_KNOWN_AREA;
        }
        else if( occupy >= threshVal ) //임계점 이상이면 벽으로 간주 , 임계점 기준은 감이다.
        {
            ocuupancyCvtbuf[i] = GRAY_LV_KNOWN_WALL;
        }                                
        else{
            ocuupancyCvtbuf[i] = GRAY_LV_UNKNOWN_WALL;
        }
    }
    
    
    Mat occupancyCvt = Mat(srcHeight, srcWidth, CV_8UC1, ocuupancyCvtbuf);
    
    TIME_CHECK_END(__debug_sw.getTime());
    return occupancyCvt.clone();
}


/**
 * @brief simplify grid map 을 mat 파일로...
 * 
 * @param pSrc 
 * @param srcWidth 
 * @param srcHeight 
 * @param threshVal 
 * @return Mat 
 */
Mat CImgProcessor::convertSimplifyGridMap2Mat(unsigned char *pSrc, int srcWidth, int srcHeight)
{
    CStopWatch __debug_sw;
    
    Mat map = Mat(srcHeight, srcWidth, CV_8U, pSrc);
    
    TIME_CHECK_END(__debug_sw.getTime());
    return map;
}

/**
 * @brief simplify grid map 청소영역을 포함하여....
 * 
 * @param pSrc 
 * @param srcWidth 
 * @param srcHeight 
 * @param threshVal 
 * @return Mat 
 */
Mat CImgProcessor::convertSimplifyGridMap2Mat(unsigned char *pSrc, std::list<tPoint> cleaned, 
    int srcWidth, int srcHeight, double resolution, double orgX, double orgY)
{
    CStopWatch __debug_sw;
    
    Mat map = Mat(srcHeight, srcWidth, CV_8U, pSrc);
    
    cv::Point prevPoint;
    auto x = 0, y = 0;
    for (const auto& element : cleaned) {
        
        x = (element.x - orgX) / resolution;
        y = (element.y - orgY) / resolution;
        //y = srcHeight - y;  // ros 와 cv 는 y 축이 다르다.

        if (!prevPoint.x && !prevPoint.y){
            prevPoint = cv::Point(x, y);
        }
        else{
            cv::line(map, prevPoint, cv::Point(x, y), cv::Scalar(0), 4);
            prevPoint = cv::Point(x, y);
        }
    }

    // eblog(LOG_LV_NECESSARY, "called");
    
    TIME_CHECK_END(__debug_sw.getTime());
    return map.clone();
}


std::pair<double, double> CImgProcessor::getAreaCoverage(cv::Mat& img, 
    const std::vector<cv::Point>& contour, 
    const std::list<cv::Point>& cleanpath, int thickness) {
    // 마스크 이미지 생성
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    
    // contour로 경계 만들기
    const cv::Point* pts = &contour[0];
    int npts = contour.size();
    cv::fillPoly(mask, &pts, &npts, 1, cv::Scalar(255));

    // cleanpath를 두께가 있는 선으로 그리기
    cv::Mat pathMask = cv::Mat::zeros(img.size(), CV_8UC1);
    for (auto it = cleanpath.begin(); it != std::prev(cleanpath.end()); ++it) {
        cv::line(pathMask, *it, *std::next(it), cv::Scalar(255), thickness);
    }

    // 두 마스크의 교집합 계산
    cv::Mat filledArea;
    cv::bitwise_and(mask, pathMask, filledArea);

    // 채움 비율 계산
    double filledRatio = 100.0 * cv::countNonZero(filledArea) / cv::countNonZero(mask);
    // 채움 면적 계산
    double pixelArea = 0.0025;//1픽셀 단위 m^2
    double filledSize = cv::countNonZero(filledArea) * pixelArea ; 

    return std::make_pair(filledRatio, filledSize);
}

/**
 * @brief area 에 청소 영역을 넣어 본다.
 * 
 * @param img 
 * @param contour 
 * @param cleanpath 
 * @param thickness 
 */
double CImgProcessor::fillAreaCleanPath(cv::Mat& img, 
    const std::vector<cv::Point>& contour, 
    const std::list<cv::Point>& cleanpath, int thickness,
    std::vector<std::vector<cv::Point>>& uncleandContours) {
    // 마스크 이미지 생성
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    
    // contour로 경계 만들기
    const cv::Point* pts = &contour[0];
    int npts = contour.size();
    cv::fillPoly(mask, &pts, &npts, 1, cv::Scalar(255));

    // cleanpath를 두께가 있는 선으로 그리기
    cv::Mat pathMask = cv::Mat::zeros(img.size(), CV_8UC1);
    for (auto it = cleanpath.begin(); it != std::prev(cleanpath.end()); ++it) {
        cv::line(pathMask, *it, *std::next(it), cv::Scalar(255), thickness);
    }

    // 두 마스크의 교집합 계산
    cv::Mat filledArea;
    cv::bitwise_and(mask, pathMask, filledArea);

    // 채움 비율 계산
    double filledRatio = 100.0 * cv::countNonZero(filledArea) / cv::countNonZero(mask);
    
    // 채워지지 않은 영역의 컨투어 추출
    cv::Mat unfilledArea;
    cv::bitwise_xor(mask, filledArea, unfilledArea);
    cv::findContours(unfilledArea, uncleandContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 결과 출력
    eblog(LOG_LV_NECESSARY,"Filled Ratio: " << filledRatio << "%" );
    eblog(LOG_LV_NECESSARY, "Unfilled Contours: " << uncleandContours.size() );

    // 결과 이미지 생성 
    // cv::Mat result;
    // img.copyTo(result);
    // cv::drawContours(result, uncleandContours, -1, cv::Scalar(0,255,0), 2);

    return filledRatio;
}


/**
 * @brief Mat 이미지를 y축 반전하여 gridmap 으로 변환
 * 
 * @param src 
 * @return std::vector<unsigned char>
 * @date 23.09.01
 * @note 연산시간 : 0.10ms ~ 0.20ms
 */
std::vector<unsigned char> CImgProcessor::convertMat2GridMap(Mat src)
{
    CStopWatch __debug_sw;

    // Mat 객체를 1차원 char 배열로 변환
    int width = src.cols;
    int height = src.rows;
    int dataSize = src.rows * src.cols * src.channels();
    std::vector<unsigned char> ret(dataSize);

    for( int h=0; h<height; h++)
    {
        int srcOffset = h*width;
        int destOffset = (height-h-1)*width;
        std::memcpy(ret.data() + destOffset, src.data + srcOffset, sizeof(unsigned char)*width);
    }

    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}

/**
 * @brief Get the Bounding Rect object
 * icbaek - 23-01-03
 * area를 구하기 위한 방법 중 최소 사각형으로 구할 때 이함수를 사용
 * Mat 이미지를 입력하면 그안에 얼만큼의 area 가 있는지 판단해 list로 
 * 반환 한다.     * 
 * @param antinoiseImg Mat 이미지 ( 노이즈가 제거된 raw cleanMap 을 넣는다.)
 * @param orgX 검사하고싶은 x 좌표
 * @param orgY 검사하고 싶은 y  좌표
 * @param length 사각형 크기 (정사각형)
 * @return std::list<Rect> 검사 영역에서 찾은 사각형들.
 */
std::list<Rect> CImgProcessor::getBoundingRect(Mat antinoiseImg, int orgX, int orgY, int length)
{
    CStopWatch __debug_sw;

    std::list<Rect> retRect;
    std::vector< std::vector<Point> > contours; // list of contour points

    // 이미지에서 좌표 크기 만큼을 ROI 로 따낸다.
    Mat roiAnti = antinoiseImg(Rect(orgX ,orgY, length, length));
    
    //contours 를 찾는다.
    findContours(roiAnti,contours,RETR_TREE,CHAIN_APPROX_SIMPLE);
    
    const int kDeleteSize = 20; // 30cm 로정함, 로봇의 크기 //CHECK 20->40
    for (int i = 0; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours[i]);

        //std::cout << "find area : " << rect.area() << std::endl;

        // 면적 이상일때 이상이고 with, height 의 크기가 kDeleteSize 보다 커야 한다.
        // 로봇 크기보다 작으면 ㄹ 패턴하기 어렵다.
        if ( rect.width > kDeleteSize && rect.height > kDeleteSize)
        {
            if (rect.area() > MIN_AREA )
            {
                rect.x += orgX;
                rect.y += orgY;
                retRect.emplace_back(rect);
            }            
        }
        
        TIME_CHECK_END(__debug_sw.getTime());
        return retRect;
    }
}

/**
 * @brief 영역분할 알고리즘을 아직 못 만들어서 유사하게 임시로 쓸 함수.
 * 
 * @param src 
 * @return std::list<Rect> 
 */
std::list<Rect> CImgProcessor::fakeBsp(Mat src, int kDeleteSize)
{
    CStopWatch __debug_sw;

    // 침식과 팽창 알고리즘을 위한 커널 생성
    Mat kernel = getStructuringElement(MORPH_RECT, Size(20, 20));

    // 침식
    Mat eroded;
    erode(src, eroded, kernel);
    
    // 팽창        
    Mat dilated;
    dilate(eroded, dilated, kernel);

    Mat img_threshold;    
    threshold(dilated, img_threshold, 230, 255, THRESH_BINARY);

    // Find contours
    std::vector<std::vector<Point> > contours;
    findContours(img_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    
    std::list<Rect> retRect;
    eblog(LOG_LV_NECESSARY, "contours.size : "<< contours.size() );
    for (size_t i = 0; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours[i]);

        // 사각형 축소 비율 설정 
        double scale = 1.1; //0.9;//0.8; // % 축소 

        // 사각형 축소 
        cv::Point new_tl(cvRound(rect.x + rect.width * (1 - scale) / 2), cvRound(rect.y + rect.height * (1 - scale) / 2)); 
        cv::Size new_size(cvRound(rect.width * scale), cvRound(rect.height * scale)); 
        cv::Rect new_rect(new_tl, new_size); 

        // 면적 이상일때 이상이고 with, height 의 크기가 kDeleteSize 보다 커야 한다.
        // 로봇 크기보다 작으면 ㄹ 패턴하기 어렵다.
        if ( rect.width > kDeleteSize && rect.height > kDeleteSize)
        {
            //if (rect.area() > MIN_AREA )  //거리로 만 판단하자..
            {
                retRect.emplace_back(new_rect);
            }
        }
        else
        {
            eblog(LOG_LV_NECESSARY, "길이 탈락, kDeleteSize : "<< kDeleteSize 
                <<" , rect.width : " << rect.width << " , rect.height : " << rect.height );
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
    return retRect;
}


// 컨투어의 중심점을 계산하는 함수
cv::Point2f CImgProcessor::computeContourCenter(const std::vector<cv::Point>& contour) {
    cv::Moments moments = cv::moments(contour, false);
    if (moments.m00 == 0) {
        return cv::Point2f(0, 0);
    }
    return cv::Point2f(static_cast<float>(moments.m10 / moments.m00), static_cast<float>(moments.m01 / moments.m00));
}

// 컨투어 크기를 조절하는 함수
// scale : 1.0 : 원본
// scale : 1.5 : 150% 확대
// scale : 0.5 : 50% 축소
std::vector<std::vector<cv::Point>> CImgProcessor::resizeContour(const std::vector<std::vector<cv::Point>>& contours, float scale) {
    std::vector<std::vector<cv::Point>> resizedContours; // 결과를 저장할 벡터

    for (const auto& contour : contours) {
        std::vector<cv::Point> resizedContour;
        cv::Point2f center = computeContourCenter(contour); // 각 컨투어의 중심 계산

        for (const cv::Point& pt : contour) {
            cv::Point2f ptF(pt.x, pt.y); // cv::Point를 cv::Point2f로 변환
            cv::Point2f direction = ptF - center; // 중심으로부터의 방향 및 거리
            cv::Point2f scaledDirection = direction * scale; // 스케일 조정
            cv::Point scaledPoint(static_cast<int>(center.x + scaledDirection.x), static_cast<int>(center.y + scaledDirection.y)); // 새 위치 계산
            resizedContour.push_back(scaledPoint);
        }

        resizedContours.push_back(resizedContour); // 리사이징된 컨투어 추가
    }

    return resizedContours; // 수정된 컨투어들의 집합 반환
}

/**
 * @brief 영역 분할을 사각형이 아닌 모양으로 하는 함수
 * 
 * @param src 
 * @return std::list<Rect> 
 */
std::vector<std::vector<cv::Point>> CImgProcessor::fakeBspV2(
    Mat src, int kDeleteSize)
{
    CStopWatch __debug_sw;

    // 침식과 팽창 알고리즘을 위한 커널 생성
    Mat kernel = getStructuringElement(MORPH_RECT, Size(20, 20));

    // 침식
    Mat eroded;
    erode(src, eroded, kernel);
    
    // 팽창        
    Mat dilated;
    dilate(eroded, dilated, kernel);

    Mat img_threshold;    
    threshold(dilated, img_threshold, 230, 255, THRESH_BINARY);

    // Find contours
    std::vector<std::vector<Point> > contours;
    findContours(img_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    eblog(LOG_LV_NECESSARY,"이미지처리전 방개수(컨투어) : "<<contours.size());

    Mat srcColor;
    cvtColor(src, srcColor, COLOR_GRAY2BGR);
    std::vector<std::vector<cv::Point>> approxContours(contours.size());
    // 각 윤곽선에 대해 RDP 알고리즘을 적용합니다.
    for(size_t i = 0; i < contours.size(); i++) {
        // 근사 정밀도를 설정합니다. 이 값은 실험을 통해 최적의 값을 찾아야 할 수도 있습니다.
        double epsilon = 0.011 * cv::arcLength(contours[i], true);
        cv::approxPolyDP(contours[i], approxContours[i], epsilon, true);
    }
    eblog(LOG_LV_NECESSARY,"RDP 후 방개수(컨투어) : "<<approxContours.size());

#if 0 //원본에서 컨투어 줄이기
    std::vector<std::vector<cv::Point>> reSizeContours = resizeContour(approxContours,0.80);    

    eblog(LOG_LV_NECESSARY,"리사이즈 방개수(컨투어) : "<<reSizeContours.size());

    TIME_CHECK_END(__debug_sw.getTime());
    return reSizeContours;
#else
    
    return approxContours;
#endif
}


/**
 * @brief 영역 분할을 사각형으로 하되 맵 안의 장애물을 포함하는 사각형을 만드는 함수
 * 
 * @param src 
 * @param kDeleteSize 
 * @return std::list<std::list<tPoint>> 
 */
std::list<Rect> CImgProcessor::fakeBspV3(Mat src, int kDeleteSize)
{
    // 침식과 팽창 알고리즘을 위한 커널 생성
    Mat kernel = getStructuringElement(MORPH_RECT, Size(20, 20));

    Mat img_threshold;    
    threshold(src, img_threshold, 230, 255, THRESH_BINARY);

    // Find contours
    std::vector<std::vector<Point> > filteredContours;
    std::vector<std::vector<Point> > contours;
    findContours(img_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    
    for(auto & contour :contours)
    {
        if(cv::contourArea(contour) > 300.0){
            filteredContours.push_back(contour);
        }
    }

    Mat imgContour= cv::Mat::zeros(src.size(), CV_8UC1);
    imgContour.setTo(cv::Scalar(255));

    drawContours(imgContour, filteredContours, -1, Scalar(0), 2);

    // 벽 이어 버리기.
    Mat eroded;
    erode(imgContour, eroded, kernel);
    Mat dilated;
    dilate(eroded, dilated, kernel);

    //외각 영역 언노운으로 처리
    cv::floodFill(dilated, cv::Point(0, 0), GRAY_LV_UNKNOWN_AREA); 

    //최외각이 잡히는거 막기
    threshold(dilated, img_threshold, 230, 255, THRESH_BINARY);

    std::vector<std::vector<Point> > contoursRe;
    findContours(img_threshold, contoursRe, RETR_TREE, CHAIN_APPROX_SIMPLE);

    std::list<Rect> retRect;
    
    for (size_t i = 0; i < contoursRe.size(); i++)
    {
        Rect rect = boundingRect(contoursRe[i]);

        // 사각형 축소 비율 설정 
        double scale = 1.1;

        // 사각형 축소 
        cv::Point new_tl(cvRound(rect.x + rect.width * (1 - scale) / 2), cvRound(rect.y + rect.height * (1 - scale) / 2)); 
        cv::Size new_size(cvRound(rect.width * scale), cvRound(rect.height * scale)); 
        cv::Rect new_rect(new_tl, new_size); 

        // 면적 이상일때 이상이고 with, height 의 크기가 kDeleteSize 보다 커야 한다.
        // 로봇 크기보다 작으면 ㄹ 패턴하기 어렵다.
        if ( rect.width > kDeleteSize && rect.height > kDeleteSize)
        {
            //
            if(rect.width < src.cols * 0.9 && rect.height < src.rows * 0.9){
                retRect.emplace_back(new_rect);
            }
        }
    }

    return retRect;
}


std::vector<std::vector<cv::Point>> CImgProcessor::fakeBspV4(Mat src, int kDeleteSize)
{
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

        // 침식
    Mat eroded;
    dilate(src, eroded, kernel);
    
    // 팽창        
    Mat dilated;
    erode(eroded, dilated, kernel);
    
    Mat img_threshold;    
    threshold(dilated, img_threshold, 230, 255, THRESH_BINARY);

    // Find contours
    std::vector<std::vector<Point> > contours;
    findContours(img_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);


    // 결과 이미지 준비
    Mat resultImageBgr;
    cvtColor(src, resultImageBgr, COLOR_GRAY2BGR);


    std::vector<Point> detect;
    for (size_t i = 0; i < contours.size(); i++) {
        // 볼록 껍질 찾기
        std::vector<int> hull;
        convexHull(contours[i], hull, false);

        // 볼록성 결함 찾기
        std::vector<Vec4i> defects;
        if (hull.size() > 3) {
            convexityDefects(contours[i], hull, defects);
        }

        // 결함을 통한 돌출부위 분석 및 표시
        for (const Vec4i& d : defects) {
            int startIndex = d[0]; // 시작 인덱스
            int endIndex = d[1]; // 끝 인덱스
            int farthestIndex = d[2]; // 가장 멀리 있는 결함 점의 인덱스
            float depth = d[3] / 256.0; // 결함의 깊이

            // 예를 들어, 결함의 깊이가 특정 값보다 큰 경우만 표시
            if (depth > 5) { // 깊이 기준은 상황에 따라 조정
                // line(resultImage, contours[i][startIndex], contours[i][endIndex], Scalar(0, 255, 0), 2);
                // circle(resultImage, contours[i][farthestIndex], 3, Scalar(0, 0, 255), 1);
                detect.push_back(contours[i][farthestIndex]);
            }
        }
    }
#if 1        
    std::cout << "Distance between point :  " << detect.size() << std::endl;

    for (size_t i = 0; i < detect.size(); ++i) {
        for (size_t j = i + 1; j < detect.size(); ++j) {
            Point p1 = detect[i];
            Point p2 = detect[j];
            double distance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
            if (distance >= 3 && distance <= 25){
                line(resultImageBgr, p1, p2, Scalar(0, 0, 0), 5);                                  
            }
                std::cout << "Distance between point " << i << " and point " << j << ": " << distance << std::endl;
        }
    }
#endif


    Mat resultImage;
    cvtColor(resultImageBgr, resultImage, COLOR_BGR2GRAY);


    cv::floodFill(resultImage, cv::Point(0, 0), GRAY_LV_UNKNOWN_AREA); 

    threshold(resultImage, resultImage, 230, 255, THRESH_BINARY);

    // Find contours
    std::vector<std::vector<Point> > contoursRe;
    findContours(resultImage, contoursRe, RETR_TREE, CHAIN_APPROX_SIMPLE);
    eblog(LOG_LV_NECESSARY,"이미지처리전 방개수(컨투어) : "<<contoursRe.size());

    // 필터링된 컨투어를 저장할 벡터
    std::vector<std::vector<Point>> filteredContours;

    // 최소 크기 설정
    double minArea = 100.0;  // 최소 면적, 필요에 따라 조정

    for (const auto& contour : contoursRe) {
        double area = contourArea(contour);
        if (area > minArea) {
            filteredContours.push_back(contour);
        }
    }

    eblog(LOG_LV_NECESSARY,"사이즈 필터 방개수(컨투어) : "<<filteredContours.size());

    Mat srcColor;
    cvtColor(src, srcColor, COLOR_GRAY2BGR);
    std::vector<std::vector<cv::Point>> approxContours(filteredContours.size());
    // 각 윤곽선에 대해 RDP 알고리즘을 적용합니다.
    for(size_t i = 0; i < filteredContours.size(); i++) {
        // 근사 정밀도를 설정합니다. 이 값은 실험을 통해 최적의 값을 찾아야 할 수도 있습니다.
        double epsilon = 0.011 * cv::arcLength(filteredContours[i], true);
        cv::approxPolyDP(filteredContours[i], approxContours[i], epsilon, true);
    }
    eblog(LOG_LV_NECESSARY,"RDP 후 방개수(컨투어) : "<<approxContours.size());

    return approxContours;
    

}

std::list<std::list<tPoint>> CImgProcessor::findDoor(Mat src)
{
    std::list<std::list<tPoint>> ret;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(6, 6));

    // 침식
    Mat eroded;
    dilate(src, eroded, kernel);
    
    // 팽창        
    Mat dilated;
    erode(eroded, dilated, kernel);
    
    Mat img_threshold;    
    threshold(dilated, img_threshold, 230, 255, THRESH_BINARY);

    // Find contours
    std::vector<std::vector<Point> > contours;
    findContours(img_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    
    std::vector<Point> detect;
    for (size_t i = 0; i < contours.size(); i++) {
        // 볼록 껍질 찾기
        std::vector<int> hull;
        convexHull(contours[i], hull, false);

        // 볼록성 결함 찾기
        std::vector<cv::Vec4i> defects;
        if (hull.size() > 3) {
            convexityDefects(contours[i], hull, defects);
        }

        // 결함을 통한 돌출부위 분석 및 표시
        for (const cv::Vec4i& d : defects) {
            int startIndex = d[0]; // 시작 인덱스
            int endIndex = d[1]; // 끝 인덱스
            int farthestIndex = d[2]; // 가장 멀리 있는 결함 점의 인덱스
            float depth = d[3] / 256.0; // 결함의 깊이

            // 예를 들어, 결함의 깊이가 특정 값보다 큰 경우만 표시
            if (depth > 5) { // 깊이 기준은 상황에 따라 조정
                // line(resultImage, contours[i][startIndex], contours[i][endIndex], Scalar(0, 255, 0), 2);
                //circle(resultImage, contours[i][farthestIndex], 3, Scalar(0, 0, 255), 1);
                detect.push_back(contours[i][farthestIndex]);
            }
        }
    }

    eblog(LOG_LV_NECESSARY, "검출 포인트 : "<< detect.size() );

    for (size_t i = 0; i < detect.size(); ++i) {
        for (size_t j = i + 1; j < detect.size(); ++j) {
            std::list<tPoint> rectlist;
            Point p1 = detect[i];
            Point p2 = detect[j];
            double distance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
            if (distance >= 10 && distance <= 25){
                //line(resultImage, p1, p2, Scalar(0, 255, 0), 1);

                // 직선의 방향 벡터 계산
                Point direction = p2 - p1;
                // 단위 벡터로 변환
                Point unitDirection = direction * (1.0 / norm(direction));
                // 직선에 수직인 벡터 계산
                Point normal(-unitDirection.y, unitDirection.x);

                // 사각형의 네 꼭짓점 계산
                Point rectPoints[4];
                int thickness = 3;
                rectPoints[0] = p1 + normal * thickness; // 상단 왼쪽
                rectPoints[1] = p2 + normal * thickness; // 상단 오른쪽
                rectPoints[2] = p2 - normal * thickness; // 하단 오른쪽
                rectPoints[3] = p1 - normal * thickness; // 하단 왼쪽

                for (int k = 0; k <4; k++)
                {
                    rectlist.push_back(tPoint(rectPoints[k].x, rectPoints[k].y));
                }
                ret.push_back(rectlist);
            }            
        }
    }
    return ret;
}

std::list<RotatedRect> CImgProcessor::checkAreaMinRect(Mat antinoiseImg, int orgX, int orgY, int length)
{
    CStopWatch __debug_sw;

    std::list<RotatedRect> retRect;
    std::vector< std::vector<Point> > contours; // list of contour points

    // 이미지에서 좌표 크기 만큼을 ROI 로 따낸다.
    Mat roiAnti = antinoiseImg(Rect(orgX ,orgY, length, length));
    
    //contours 를 찾는다.
    findContours(roiAnti,contours,RETR_TREE,CHAIN_APPROX_SIMPLE);
    
    const int kDeleteSize = 6; // 30cm 로정함, 로봇의 크기
    for (int i = 0; i < contours.size(); i++)
    {            
        RotatedRect rect = minAreaRect(contours[i]);

        //std::cout << "find area : " << rect.area() << std::endl;

        // 면적 이상일때...            
        if (rect.size.area() > MIN_AREA )
        {                
            rect.center.x = orgX;
            rect.center.y = orgY;
            retRect.emplace_back(rect);
        }            
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return retRect;
}

/**
 * @brief Mat 파일을 unsigned char 로 바꾼다.
 * 
 * @param src Mat image 원본
 * @param dest Mat image 를 담을 곳
 */
void CImgProcessor::mat2uchar(Mat src, unsigned char * dest)
{
    CStopWatch __debug_sw;

    int numOfLines = src.rows; // number of lines in the image
    int numOfPixels = src.cols; // number of pixels per a line
    int idx = 0;
    for( int r=0; r<numOfLines; r++ )
    {
        for( int c=0; c<numOfPixels; c++ )
        {            
            dest[idx++] = src.at<uchar>( r, c );
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

void CImgProcessor::mat2s8(Mat src, s8* dest)
{
    CStopWatch __debug_sw;

    int numOfLines = src.rows; // number of lines in the image
    int numOfPixels = src.cols; // number of pixels per a line
    int idx = 0;
    
    for( int r=0; r<numOfLines; r++ )
    {
        for( int c=0; c<numOfPixels; c++ )
        {
            dest[idx++] = src.at<s8>( r, c );
        }
    }

    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cleanMap 을 바운딩 렉트한다.
 * 확장된 영역만 가지고 온다.
 * 이미지를 전송할때 유리하다.
 * @param src 
 * @return Mat 
 */
Mat CImgProcessor::getCleanMapBoundingRect(unsigned char *src)
{
    CStopWatch __debug_sw;
    
    Mat crop = convertCleanMap2cvImg(src, 1000, 1000);

    std::list<Rect> retRect;
    std::vector< std::vector<Point> > contours; // list of contour points
            
    findContours(crop,contours,RETR_TREE,CHAIN_APPROX_SIMPLE);
    
    for (int i = 0; i < contours.size(); i++)
    {
        Rect rect = boundingRect(contours[i]);

        //std::cout << "find area : " << rect.area() << std::endl;

        // TODO : 면적 이상일때 를 가장 큰 면적 하나로 만들어야 함.
        if (rect.area() > 10000)
        {
            retRect.emplace_back(rect);
        }
    }
    crop = crop(retRect.front());

    TIME_CHECK_END(__debug_sw.getTime());
    return crop;
}

/**
 * @brief 확률 그리드맵의 노이즈를 제거 한다.
 * 
 * @param input 
 * @return Mat 
 */
Mat CImgProcessor::antinoise(Mat input)
{
    CStopWatch __debug_sw;

    Mat open;
    erode(input, open, Mat()); // 원 영상을 침식        
    erode(open, open, Mat()); // 원 영상을 침식        
    erode(open, open, Mat()); // 원 영상을 침식        
    erode(open, open, Mat()); // 원 영상을 침식
    

    dilate(open, open, Mat()); // 침식된 영상에 대한 팽창        
    dilate(open, open, Mat()); // 침식된 영상에 대한 팽창        
    dilate(open, open, Mat()); // 침식된 영상에 대한 팽창        
    dilate(open, open, Mat()); // 침식된 영상에 대한 팽창
    //Canny(open, open, 50, 150);
    
    TIME_CHECK_END(__debug_sw.getTime());
    return open;
}

/**
 * @brief 이미지 채널 수 확인 함수
 * 
 * @param img 
 */
void CImgProcessor::channelCheck(cv::Mat img)
{
    CStopWatch __debug_sw;

    eblog(LOG_LV, "channel : "<<img.channels());
    
    TIME_CHECK_END(__debug_sw.getTime());

}


/**
 * @brief 확률 그리드의 벽을 선으로 바꾸기
 * 
 * @param pSrc 
 * @param srcWidth 
 * @param srcHeight 
 * @return Mat
 * @date 23.09.01
 * @note 연산시간 : 15.3.6ms ~ 310.6ms (연산시간 변동 요인은 미확인.)
 */
Mat CImgProcessor::wallLineFit(s8 *pSrc, int srcWidth, int srcHeight )
{
    double startTick = SYSTEM_TOOL.getSystemTime();
    CStopWatch __debug_sw;

    int openingKernalSize = ROS_CONFIG.simplifyGridmap.openingKernalSize;
    int wallThickness = ROS_CONFIG.simplifyGridmap.wallThickness;

    // eblog(LOG_LV_NECESSARY, "==== parameter change ===");
    // eblog(LOG_LV_NECESSARY, "Opening 커널 크기 : "<<openingKernalSize);
    // eblog(LOG_LV_NECESSARY, "          벽 두께 : "<<wallThickness);
    // eblog(LOG_LV_NECESSARY, "=========================");

    // 이미지 읽기
    Mat src = convertGridMap2Mat(pSrc, srcWidth, srcHeight, 40);    // 40   hyjoe . kernel 4로 했을 때 확률값 40이 가장 이상적으로 보여짐 (일단...)

    // 침식과 팽창 알고리즘을 위한 커널 생성
    Mat kernel = getStructuringElement(MORPH_RECT, Size(openingKernalSize, openingKernalSize));

#if 0
    // 침식
    Mat eroded;
    erode(src, eroded, kernel);
    
    // 팽창
    Mat dilated;
    dilate(eroded, dilated, kernel);
#endif

    // 벽 영역 추출 및 팽창
    Mat inv;
    Mat img_wall = (src == GRAY_LV_KNOWN_WALL);   
    bitwise_not(img_wall, inv);

    Mat erodeWall;
    erode(inv,erodeWall,kernel);

    Mat wall_threshold;
    threshold(erodeWall, wall_threshold, 100, 255, THRESH_BINARY);
    
    // threshold -> canny 검출을 위해.
    Mat img_threshold;    
    threshold(src, img_threshold, 230, 255, THRESH_BINARY);

    // Canny 엣지 검출
    Mat edges;
    Canny(img_threshold, edges, 80, 200, 3);

    // 벽 찾기 (미확인 영역) : 엣지에서 벽역영을 제외시켜 찾는다.
    Mat wall_unknown;
    wall_threshold.copyTo(wall_unknown,edges);

    // 벽 찾기 (확인 영역) : 엣지에서 벽역역을 빼고 찾는다.
    Mat wall_known;
    subtract(edges, wall_unknown, wall_known);

#if 0
    //벽 (확인 영역) 굵게 만들기
    kernel = getStructuringElement(MORPH_RECT, Size(EXPANSION_SIZE, EXPANSION_SIZE));
    dilate(wall_known, wall_known, kernel);
    dilate(wall_unknown, wall_unknown, kernel);

    //합성 : 이 이미지를 쓸지 말지 고민하다가 지금은 안쓰는 중.
    Mat sum;
    bitwise_xor(wall_known, wall_unknown, sum);
#endif

    // 이미지 level 정리.
    Mat result = (src == GRAY_LV_KNOWN_AREA);
    cv::floodFill(result, cv::Point(0, 0), GRAY_LV_UNKNOWN_AREA); 
    result.setTo(GRAY_LV_UNKNOWN_WALL, wall_unknown);
    result.setTo(GRAY_LV_KNOWN_WALL, wall_known);

    erode(result, result, kernel); // dstar벽 넓히기 위해 벽확장.


#if 1   // 주기 체크 디버깅.
    double endTick = SYSTEM_TOOL.getSystemTime();
    double diffTick = (endTick - startTick) * 1000;

    // if (diffTick > 20.0)
    {
        std::cout << "wallLineFit cycle: " << diffTick << " ms" << std::endl;
    }
#endif
    
    TIME_CHECK_END(__debug_sw.getTime());
    return result.clone();
}
/**
 * @brief 충전기 그리기.
 * 
 * @param pSrc 
 * @param srcWidth 
 * @param srcHeight 
 * @param dockingzonePixel 
 * @return Mat 
 */
Mat CImgProcessor::wallLineFit(s8 *pSrc, int srcWidth, int srcHeight, std::list<cv::Point> dockingzonePixel ){
    Mat ret;
    ret = wallLineFit(pSrc, srcWidth, srcHeight);

    // 이미지 경계 확인 및 좌표 조정
    int maxX = ret.cols - 1;
    int maxY = ret.rows - 1;
    std::vector<cv::Point> contourVec;
    for (auto& point : dockingzonePixel) {
        int x = std::max(0, std::min(point.x, maxX)); // 0과 maxX 사이로 x 좌표 조정
        int y = std::max(0, std::min(point.y, maxY)); // 0과 maxY 사이로 y 좌표 조정
        contourVec.push_back(cv::Point(x, y));
    }

    const cv::Point* elementPoints[1] = { &contourVec[0] };
    int numberOfPoints[1] = { static_cast<int>(contourVec.size()) };

    // 다각형 내부를 0으로 채우기
    cv::fillPoly(ret, elementPoints, numberOfPoints, 1, cv::Scalar(0), 8);

    return ret.clone();

}

/**
 * @brief 낙하영역을 검정 처리
 * 
 * @param org 
 * @param cliffPoints 
 */
void CImgProcessor::drawCliffPoint(cv::Mat &org, std::list<cv::Point> cliffPoints ){        

    // 이미지 경계 확인
    int maxX = org.cols - 1;
    int maxY = org.rows - 1;

    int rectWidth = 7;
    int rectHeight = 7;

    // 각 cliffPoints 점마다 사각형 그리기
    for (const auto& point : cliffPoints) {
        int x = std::max(0, std::min(point.x, maxX)); // x 좌표 조정
        int y = std::max(0, std::min(point.y, maxY)); // y 좌표 조정

        int halfWidth = rectWidth / 2;
        int halfHeight = rectHeight / 2;

        // 사각형 그리기
        cv::rectangle(org, 
                      cv::Point(std::max(0, x - halfWidth), std::max(0, y - halfHeight)),
                      cv::Point(std::min(maxX, x + halfWidth), std::min(maxY, y + halfHeight)),
                      cv::Scalar(0), cv::FILLED);
    }
}

Mat CImgProcessor::convertGridMap2MatMod(s8 *pSrc, int srcWidth, int srcHeight)
{
    //SLAM 에서 나온 확률 그리드 값을 이미지로 변환하기위해 데이터 타입 변환
    int size = srcWidth * srcHeight;
    s8 ocuupancyCvtbuf[size];    
    s8 occupy = 0; 
    for (int i = 0;i < size; i++)
    {
        occupy = pSrc[i];
        if (occupy == -1)
        {
            ocuupancyCvtbuf[i] = UNKNWON_REGION;
        }
        else if( occupy >= WALL_THRESHOLD ) //임계점 이상이면 벽으로 간주 , 임계점 기준은 감이다.
        {
            ocuupancyCvtbuf[i] = WALL_REGION;
        }                                
        else{
            ocuupancyCvtbuf[i] = EMPTY_REGION;
        }
        
    }
    
    Mat occupancyCvt = Mat(srcHeight, srcWidth, CV_8U, ocuupancyCvtbuf);
    return occupancyCvt.clone();
}

Mat CImgProcessor::where(const Mat& condition, const Mat& x, const Mat& y) {
    // Ensure the matrices have the same size and type
    if (condition.size() != x.size() || condition.size() != y.size() || condition.type() != CV_8UC1) {
        throw std::invalid_argument("Input matrices must have the same size and condition must be of type CV_8UC1.");
    }

    // Create an output matrix with the same size and type as x and y
    Mat result = Mat::zeros(x.size(), x.type());

    // Use the condition mask to copy elements from x or y
    x.copyTo(result, condition);        // Copy elements from x where condition is true
    y.copyTo(result, ~condition);       // Copy elements from y where condition is false

    return result;
}


Mat CImgProcessor::linemap(s8 *pSrc, int srcWidth, int srcHeight)
{
    double startTick = SYSTEM_TOOL.getSystemTime();

    Mat girdMat = convertGridMap2MatMod(pSrc, srcWidth, srcHeight);

    // return girdMat.clone();

    Mat gridMatOnlyWall = girdMat.clone();
    gridMatOnlyWall.setTo(0, gridMatOnlyWall==EMPTY_REGION);

    Mat gridMatOnlyValidArea = girdMat.clone();
    gridMatOnlyValidArea.setTo(0, gridMatOnlyWall==WALL_REGION);

    // return gridMatOnlyWall;

    Mat kernelForWall = getStructuringElement(MORPH_RECT, Size(5, 5));

    Mat gridMatOnlyWallMorph;
    // morphologyEx(gridMatOnlyWall, gridMatOnlyWallMorph, MORPH_OPEN, kernel);
    morphologyEx(gridMatOnlyWall, gridMatOnlyWallMorph, MORPH_CLOSE, kernelForWall);
    // morphologyEx(gridMatOnlyWall, gridMatOnlyWallMorph, MORPH_HITMISS, kernel);

    // Mat kernelForArea = (Mat_<char>(3, 3) <<
    //     0, -1,  0,
    //     -1,  1, -1,
    //     0, -1,  0);
    Mat kernelForArea = getStructuringElement(MORPH_RECT, Size(7, 7));

    Mat gridMatOnlyValidAreaMorph;
    morphologyEx(gridMatOnlyValidArea, gridMatOnlyValidAreaMorph, MORPH_OPEN, kernelForArea);

    // Mat blurred;
    // GaussianBlur(src, blurred, Size(5, 5), 0);

    // Mat gridMatOnlyWallMorphEdges;
    // Canny(gridMatOnlyWallMorph, gridMatOnlyWallMorphEdges, 50, 150);

    // std::vector<Vec2f> lines;
    // // HoughLines(gridMatOnlyWallMorph, lines, 1, CV_PI / 180, 150); 
    // // HoughLines(girdMat, lines, 1, CV_PI / 180, 150); 
    // HoughLines(gridMatOnlyWallMorphEdges, lines, 1, CV_PI / 180, 20); 

    // Mat lineImage = Mat::zeros(gridMatOnlyWallMorph.size(), CV_8UC3);
    // for (size_t i = 0; i < lines.size(); i++) {
    //     float rho = lines[i][0];
    //     float theta = lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a * rho, y0 = b * rho;
    //     pt1.x = cvRound(x0 + 1000 * (-b));
    //     pt1.y = cvRound(y0 + 1000 * (a));
    //     pt2.x = cvRound(x0 - 1000 * (-b));
    //     pt2.y = cvRound(y0 - 1000 * (a));
    //     line(lineImage, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
    // }

    // std::vector<Vec2f> filtered_lines = nonMaximumSuppressionForLineInfo(
    //     lines, 
    //     2, 
    //     3, 
    //     0.1
    //     );

    // Mat lineImageFiltered = Mat::zeros(gridMatOnlyWallMorph.size(), CV_8UC3);
    // for (size_t i = 0; i < filtered_lines.size(); i++) {
    //     float rho = filtered_lines[i][0];
    //     float theta = filtered_lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a * rho, y0 = b * rho;
    //     pt1.x = cvRound(x0 + 1000 * (-b));
    //     pt1.y = cvRound(y0 + 1000 * (a));
    //     pt2.x = cvRound(x0 - 1000 * (-b));
    //     pt2.y = cvRound(y0 - 1000 * (a));
    //     line(lineImageFiltered, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
    // }

    // // threshold -> canny 검출을 위해.
    // Mat img_threshold;    
    // threshold(src, img_threshold, 230, 255, THRESH_BINARY);

    // // Canny 엣지 검출
    // Mat edges;
    // Canny(gridMatOnlyWall, edges, 80, 200, 3);

    std::vector<Vec4i> lines;
    HoughLinesP(gridMatOnlyWall, lines, 1, CV_PI / 90, 20, 3, 3);

    // cout << "Number of detected lines:" << lines.size() << endl;

    Mat lineImage = Mat::zeros(gridMatOnlyWallMorph.size(), CV_8UC3);
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i eachline = lines[i];
        line(lineImage, Point(eachline[0], eachline[1]), Point(eachline[2], eachline[3]), Scalar(0, 0, 255), 3);
    }
    
    Mat lineImageBinary;
    cvtColor(lineImage, lineImageBinary, COLOR_BGR2GRAY);
    threshold(lineImageBinary, lineImageBinary, 50, 255, THRESH_BINARY);

    Mat compensatedOnlyWall = where(lineImageBinary > gridMatOnlyWall, lineImageBinary, gridMatOnlyWall);

    // threshold(gridMatOnlyValidAreaMorph, gridMatOnlyValidAreaMorph, 50, EMPTY_REGION, THRESH_BINARY);
    Mat gridMatOnlyValidAreaMorphRefined = where(gridMatOnlyValidAreaMorph >= EMPTY_REGION, gridMatOnlyValidArea, Mat::zeros(gridMatOnlyValidArea.size(), gridMatOnlyValidArea.type()));
    Mat compensatedGrid = where(compensatedOnlyWall >= gridMatOnlyValidAreaMorph, compensatedOnlyWall, gridMatOnlyValidAreaMorphRefined);

    // vector<Vec4i> filteredLines = nonMaximumSuppression(lines, 3, 10, 0.5);

    // cout << "Number of filtered lines:" << filteredLines.size() << endl;
    // Mat lineImageFiltered = Mat::zeros(gridMatOnlyWall.size(), CV_8UC3);
    // for (size_t i = 0; i < filteredLines.size(); i++) {
    //     Vec4i eachline = filteredLines[i];
    //     line(lineImageFiltered, Point(eachline[0], eachline[1]), Point(eachline[2], eachline[3]), Scalar(0, 0, 255), 2);
    // }

    // Mat gridMatOnlyWallColor, gridMatOnlyWallMorphColor, gridMatOnlyWallMorphEdgesColor, result;
    // cvtColor(gridMatOnlyWall, gridMatOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyWallMorph, gridMatOnlyWallMorphColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyWallMorphEdges, gridMatOnlyWallMorphEdgesColor, COLOR_GRAY2BGR);
    // hconcat(gridMatOnlyWallColor, gridMatOnlyWallMorphColor, result);
    // hconcat(result, gridMatOnlyWallMorphEdgesColor, result);
    // hconcat(result, lineImage, result);

    // Mat gridMatOnlyWallColor, gridMatOnlyValidAreaColor, gridMatOnlyValidAreaMorphColor, result;
    // cvtColor(gridMatOnlyWall, gridMatOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyValidArea, gridMatOnlyValidAreaColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyValidAreaMorph, gridMatOnlyValidAreaMorphColor, COLOR_GRAY2BGR);
    // hconcat(gridMatOnlyWallColor, gridMatOnlyValidAreaColor, result);
    // hconcat(result, gridMatOnlyValidAreaMorphColor, result);

    // Mat gridMatOnlyWallColor, compensatedOnlyWallColor, compensatedGridColor, result;
    // cvtColor(gridMatOnlyWall, gridMatOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(compensatedOnlyWall, compensatedOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(compensatedGrid, compensatedGridColor, COLOR_GRAY2BGR);
    // hconcat(gridMatOnlyWallColor, lineImage, result);
    // hconcat(result, compensatedOnlyWallColor, result);
    // hconcat(result, compensatedGridColor, result);

    Mat girdMatColor, compensatedGridColor, result;
    Mat compensatedGrid_conv1 = where(
        compensatedGrid == UNKNWON_REGION, 
        Mat::ones(compensatedGrid.size(), compensatedGrid.type()) * GRAY_LV_UNKNOWN_AREA, 
        Mat::zeros(compensatedGrid.size(), compensatedGrid.type()));
    Mat compensatedGrid_conv2 = where(
        compensatedGrid == EMPTY_REGION, 
        Mat::ones(compensatedGrid.size(), compensatedGrid.type()) * GRAY_LV_KNOWN_AREA, 
        Mat::zeros(compensatedGrid.size(), compensatedGrid.type()));
    compensatedGrid = where(
        compensatedGrid == WALL_REGION, 
        Mat::zeros(compensatedGrid.size(), compensatedGrid.type()), 
        compensatedGrid_conv1 + compensatedGrid_conv2);

    // cvtColor(girdMat, girdMatColor, COLOR_GRAY2BGR);
    // cvtColor(compensatedGrid, compensatedGridColor, COLOR_GRAY2BGR);
    // hconcat(girdMatColor, compensatedGridColor, result);
    // 이미지 경계 확인 및 좌표 조정
#if 1   // 주기 체크 디버깅.
    double endTick = SYSTEM_TOOL.getSystemTime();
    double diffTick = (endTick - startTick) * 1000;

    // if (diffTick > 20.0)
    {
        std::cout << "linemap cycle: " << diffTick << " ms" << std::endl;
    }

#endif

    return compensatedGrid.clone();
}

Mat CImgProcessor::linemap(s8 *pSrc, int srcWidth, int srcHeight, std::list<cv::Point> dockingzonePixel)
{
    double startTick = SYSTEM_TOOL.getSystemTime();

    Mat girdMat = convertGridMap2MatMod(pSrc, srcWidth, srcHeight);

    // return girdMat.clone();

    Mat gridMatOnlyWall = girdMat.clone();
    gridMatOnlyWall.setTo(0, gridMatOnlyWall==EMPTY_REGION);

    Mat gridMatOnlyValidArea = girdMat.clone();
    gridMatOnlyValidArea.setTo(0, gridMatOnlyWall==WALL_REGION);

    // return gridMatOnlyWall;

    Mat kernelForWall = getStructuringElement(MORPH_RECT, Size(5, 5));

    Mat gridMatOnlyWallMorph;
    // morphologyEx(gridMatOnlyWall, gridMatOnlyWallMorph, MORPH_OPEN, kernel);
    morphologyEx(gridMatOnlyWall, gridMatOnlyWallMorph, MORPH_CLOSE, kernelForWall);
    // morphologyEx(gridMatOnlyWall, gridMatOnlyWallMorph, MORPH_HITMISS, kernel);

    // Mat kernelForArea = (Mat_<char>(3, 3) <<
    //     0, -1,  0,
    //     -1,  1, -1,
    //     0, -1,  0);
    Mat kernelForArea = getStructuringElement(MORPH_RECT, Size(7, 7));

    Mat gridMatOnlyValidAreaMorph;
    morphologyEx(gridMatOnlyValidArea, gridMatOnlyValidAreaMorph, MORPH_OPEN, kernelForArea);

    // Mat blurred;
    // GaussianBlur(src, blurred, Size(5, 5), 0);

    // Mat gridMatOnlyWallMorphEdges;
    // Canny(gridMatOnlyWallMorph, gridMatOnlyWallMorphEdges, 50, 150);

    // std::vector<Vec2f> lines;
    // // HoughLines(gridMatOnlyWallMorph, lines, 1, CV_PI / 180, 150); 
    // // HoughLines(girdMat, lines, 1, CV_PI / 180, 150); 
    // HoughLines(gridMatOnlyWallMorphEdges, lines, 1, CV_PI / 180, 20); 

    // Mat lineImage = Mat::zeros(gridMatOnlyWallMorph.size(), CV_8UC3);
    // for (size_t i = 0; i < lines.size(); i++) {
    //     float rho = lines[i][0];
    //     float theta = lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a * rho, y0 = b * rho;
    //     pt1.x = cvRound(x0 + 1000 * (-b));
    //     pt1.y = cvRound(y0 + 1000 * (a));
    //     pt2.x = cvRound(x0 - 1000 * (-b));
    //     pt2.y = cvRound(y0 - 1000 * (a));
    //     line(lineImage, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
    // }

    // std::vector<Vec2f> filtered_lines = nonMaximumSuppressionForLineInfo(
    //     lines, 
    //     2, 
    //     3, 
    //     0.1
    //     );

    // Mat lineImageFiltered = Mat::zeros(gridMatOnlyWallMorph.size(), CV_8UC3);
    // for (size_t i = 0; i < filtered_lines.size(); i++) {
    //     float rho = filtered_lines[i][0];
    //     float theta = filtered_lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a * rho, y0 = b * rho;
    //     pt1.x = cvRound(x0 + 1000 * (-b));
    //     pt1.y = cvRound(y0 + 1000 * (a));
    //     pt2.x = cvRound(x0 - 1000 * (-b));
    //     pt2.y = cvRound(y0 - 1000 * (a));
    //     line(lineImageFiltered, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
    // }

    // // threshold -> canny 검출을 위해.
    // Mat img_threshold;    
    // threshold(src, img_threshold, 230, 255, THRESH_BINARY);

    // // Canny 엣지 검출
    // Mat edges;
    // Canny(gridMatOnlyWall, edges, 80, 200, 3);

    std::vector<Vec4i> lines;
    HoughLinesP(gridMatOnlyWall, lines, 1, CV_PI / 90, 20, 3, 3);

    // cout << "Number of detected lines:" << lines.size() << endl;

    Mat lineImage = Mat::zeros(gridMatOnlyWallMorph.size(), CV_8UC3);
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i eachline = lines[i];
        line(lineImage, Point(eachline[0], eachline[1]), Point(eachline[2], eachline[3]), Scalar(0, 0, 255), 3);
    }
    
    Mat lineImageBinary;
    cvtColor(lineImage, lineImageBinary, COLOR_BGR2GRAY);
    threshold(lineImageBinary, lineImageBinary, 50, 255, THRESH_BINARY);

    Mat compensatedOnlyWall = where(lineImageBinary > gridMatOnlyWall, lineImageBinary, gridMatOnlyWall);

    // threshold(gridMatOnlyValidAreaMorph, gridMatOnlyValidAreaMorph, 50, EMPTY_REGION, THRESH_BINARY);
    Mat gridMatOnlyValidAreaMorphRefined = where(gridMatOnlyValidAreaMorph >= EMPTY_REGION, gridMatOnlyValidArea, Mat::zeros(gridMatOnlyValidArea.size(), gridMatOnlyValidArea.type()));
    Mat compensatedGrid = where(compensatedOnlyWall >= gridMatOnlyValidAreaMorph, compensatedOnlyWall, gridMatOnlyValidAreaMorphRefined);

    // vector<Vec4i> filteredLines = nonMaximumSuppression(lines, 3, 10, 0.5);

    // cout << "Number of filtered lines:" << filteredLines.size() << endl;
    // Mat lineImageFiltered = Mat::zeros(gridMatOnlyWall.size(), CV_8UC3);
    // for (size_t i = 0; i < filteredLines.size(); i++) {
    //     Vec4i eachline = filteredLines[i];
    //     line(lineImageFiltered, Point(eachline[0], eachline[1]), Point(eachline[2], eachline[3]), Scalar(0, 0, 255), 2);
    // }

    // Mat gridMatOnlyWallColor, gridMatOnlyWallMorphColor, gridMatOnlyWallMorphEdgesColor, result;
    // cvtColor(gridMatOnlyWall, gridMatOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyWallMorph, gridMatOnlyWallMorphColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyWallMorphEdges, gridMatOnlyWallMorphEdgesColor, COLOR_GRAY2BGR);
    // hconcat(gridMatOnlyWallColor, gridMatOnlyWallMorphColor, result);
    // hconcat(result, gridMatOnlyWallMorphEdgesColor, result);
    // hconcat(result, lineImage, result);

    // Mat gridMatOnlyWallColor, gridMatOnlyValidAreaColor, gridMatOnlyValidAreaMorphColor, result;
    // cvtColor(gridMatOnlyWall, gridMatOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyValidArea, gridMatOnlyValidAreaColor, COLOR_GRAY2BGR);
    // cvtColor(gridMatOnlyValidAreaMorph, gridMatOnlyValidAreaMorphColor, COLOR_GRAY2BGR);
    // hconcat(gridMatOnlyWallColor, gridMatOnlyValidAreaColor, result);
    // hconcat(result, gridMatOnlyValidAreaMorphColor, result);

    // Mat gridMatOnlyWallColor, compensatedOnlyWallColor, compensatedGridColor, result;
    // cvtColor(gridMatOnlyWall, gridMatOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(compensatedOnlyWall, compensatedOnlyWallColor, COLOR_GRAY2BGR);
    // cvtColor(compensatedGrid, compensatedGridColor, COLOR_GRAY2BGR);
    // hconcat(gridMatOnlyWallColor, lineImage, result);
    // hconcat(result, compensatedOnlyWallColor, result);
    // hconcat(result, compensatedGridColor, result);

    Mat girdMatColor, compensatedGridColor, result;
    Mat compensatedGrid_conv1 = where(
        compensatedGrid == UNKNWON_REGION, 
        Mat::ones(compensatedGrid.size(), compensatedGrid.type()) * GRAY_LV_UNKNOWN_AREA, 
        Mat::zeros(compensatedGrid.size(), compensatedGrid.type()));
    Mat compensatedGrid_conv2 = where(
        compensatedGrid == EMPTY_REGION, 
        Mat::ones(compensatedGrid.size(), compensatedGrid.type()) * GRAY_LV_KNOWN_AREA, 
        Mat::zeros(compensatedGrid.size(), compensatedGrid.type()));
    compensatedGrid = where(
        compensatedGrid == WALL_REGION, 
        Mat::zeros(compensatedGrid.size(), compensatedGrid.type()), 
        compensatedGrid_conv1 + compensatedGrid_conv2);

    // cvtColor(girdMat, girdMatColor, COLOR_GRAY2BGR);
    // cvtColor(compensatedGrid, compensatedGridColor, COLOR_GRAY2BGR);
    // hconcat(girdMatColor, compensatedGridColor, result);
    // 이미지 경계 확인 및 좌표 조정
    int maxX = compensatedGrid.cols - 1;
    int maxY = compensatedGrid.rows - 1;
    std::vector<cv::Point> contourVec;
    for (auto& point : dockingzonePixel) {
        int x = std::max(0, std::min(point.x, maxX)); // 0과 maxX 사이로 x 좌표 조정
        int y = std::max(0, std::min(point.y, maxY)); // 0과 maxY 사이로 y 좌표 조정
        contourVec.push_back(cv::Point(x, y));
    }

    const cv::Point* elementPoints[1] = { &contourVec[0] };
    int numberOfPoints[1] = { static_cast<int>(contourVec.size()) };

    // 다각형 내부를 0으로 채우기
    cv::fillPoly(compensatedGrid, elementPoints, numberOfPoints, 1, cv::Scalar(0), 8);

#if 1   // 주기 체크 디버깅.
    double endTick = SYSTEM_TOOL.getSystemTime();
    double diffTick = (endTick - startTick) * 1000;

    // if (diffTick > 20.0)
    {
        std::cout << "linemap cycle: " << diffTick << " ms" << std::endl;
    }

#endif

    return compensatedGrid.clone();
}

/**
 * @brief cleanMap contours 찾기 예제 코드
 * 
 * @param img 
 */
void CImgProcessor::checkAreaContours(Mat img)
{
    CStopWatch __debug_sw;
    
    Mat roiAnti;
    Mat antinoiseImg = antinoise(img);
    Mat imgC3Org, imgC3Temp;
    
    Scalar cBlack(0,0,0);
    std::vector< std::vector<Point> > contours; // list of contour points

    cvtColor(img,imgC3Org,COLOR_GRAY2RGB);
    cvtColor(img,imgC3Temp,COLOR_GRAY2RGB);

    int intervalX = 0, intervalY = 0;
    for (int x = 0; x <1000/40; x++)
    for (int y = 0; y <1000/40; y++)
    {            
        Mat imgContour(40, 40, CV_8UC3, cBlack);
        
        intervalX = x *40;
        intervalY = y *40;
        roiAnti = antinoiseImg(Rect(intervalX ,intervalY ,40,40));            

        Mat imageROI(imgC3Temp ,Rect(intervalX ,intervalY ,40,40));
        
        findContours(roiAnti,contours,RETR_TREE,CHAIN_APPROX_SIMPLE);
        
        Scalar colorC(0,0,255);
        Scalar colorH(255,0,0);

        //if (contourArea(contours) > MIN_AREA)
        {                
            for (int i = 0; i < contours.size(); i++)
            {
                drawContours(imgContour, contours,i,colorC,2);
            }
        }

        imgContour.copyTo(imageROI);
    }

    add(imgC3Org, imgC3Temp, imgC3Org);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

/**
 * @brief cleanmap convex 찾기 예제 코드
 * 
 * @param img 
 */
void CImgProcessor::checkAreaConvex(Mat img)
{
    CStopWatch __debug_sw;

    Mat roiAnti;
    Mat antinoiseImg = antinoise(img);
    Mat imgC3Org, imgC3Temp;
    
    Scalar cBlack(0,0,0);

    std::vector< std::vector<Point> > contours; // list of contour points

    cvtColor(img,imgC3Org,COLOR_GRAY2RGB);
    cvtColor(img,imgC3Temp,COLOR_GRAY2RGB);

    int intervalX = 0, intervalY = 0;
    for (int x = 0; x <1000/40; x++)
    for (int y = 0; y <1000/40; y++)
    {            
        Mat imgContour(40, 40, CV_8UC3, cBlack);
        
        intervalX = x *40;
        intervalY = y *40;
        roiAnti = antinoiseImg(Rect(intervalX ,intervalY ,40,40));            

        Mat imageROI(imgC3Temp ,Rect(intervalX ,intervalY ,40,40));
        
        findContours(roiAnti,contours,RETR_TREE,CHAIN_APPROX_SIMPLE);

        std::vector<std::vector<Point> >hull( contours.size() );
        for( size_t i = 0; i < contours.size(); i++ )
        {
            convexHull( contours[i], hull[i] );
        }
        
        Scalar colorC(0,0,255);
        Scalar colorH(255,0,0);
        //if (contourArea(contours) > MIN_AREA)
        {
            for (int i = 0; i < contours.size(); i++)
            {                
                drawContours( imgContour, hull, (int)i, colorH, 2 );
            }
        }

        imgContour.copyTo(imageROI);
    }

    add(imgC3Org, imgC3Temp, imgC3Org);
    
    TIME_CHECK_END(__debug_sw.getTime());
}

std::list<tPoint> CImgProcessor::convertContourToPolygon(
    std::vector<cv::Point> &contour, tGridmapInfo info)
{
    std::list<tPoint> poly;

    for (auto& point : contour) {
        tPoint tp;            
        tp.x = point.x * info.resolution + info.origin_x;
        tp.y = point.y * info.resolution + info.origin_y;
        poly.push_back(tp);
    }

    return poly;
}



/**
 * @brief 로봇 path 를 cv path 로 변경.
 * 
 */
std::list<cv::Point> CImgProcessor::convertRobotpathToCvpath(
    std::list<tPoint> cleaned, tGridmapInfo info)
{
    CStopWatch __debug_sw;
    
    std::list<cv::Point> ret;
    for (const auto& element : cleaned) {
        cv::Point pt;
        
        pt.x = (element.x - info.origin_x) / info.resolution;
        pt.y = (element.y - info.origin_y) / info.resolution;
        //y = srcHeight - y;  // ros 와 cv 는 y 축이 다르다.

        ret.push_back(pt);
    }
    
    TIME_CHECK_END(__debug_sw.getTime());
    return ret;
}
