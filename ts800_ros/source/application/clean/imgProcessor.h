/**
 * @file imgProcessor.h
 * @author icbaek
 * @brief 
 * @date 2023-08-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "opencv2/opencv.hpp"
#include <vector>
#include "coordinate.h"
#include "gridmap.h"

#define MIN_AREA 200
#define EXPANSION_SIZE 4  // 1 증가할때마다 한 셀씩 모폴로지 ( 1증가 = 5cm)

class CImgProcessor
{
private:
    /* data */
public:
    CImgProcessor();
    ~CImgProcessor();
    cv::Mat convertCleanMap2cvImg(unsigned char *src, int x, int y, unsigned char addMask,  unsigned char deleteMask);
    cv::Mat convertCleanMap2cvImg(unsigned char *src, int x, int y);
    std::list<cv::Rect> getBoundingRect(cv::Mat antinoiseImg, int orgX, int orgY, int length);
    std::list<cv::RotatedRect>  checkAreaMinRect(cv::Mat antinoiseImg, int orgX, int orgY, int length);
    void mat2uchar(cv::Mat src, unsigned char * dest);
    void mat2s8(cv::Mat src, s8* dest);
    cv::Mat getCleanMapBoundingRect(unsigned char *src);
    cv::Mat antinoise(cv::Mat input);
    cv::Mat convertGridMap2Mat(s8 *pSrc, int srcWidth, int srcHeight, int threshVal);
    cv::Mat convertSimplifyGridMap2Mat(unsigned char *pSrc, int srcWidth, int srcHeight);
    cv::Mat convertSimplifyGridMap2Mat(unsigned char *pSrc, std::list<tPoint> cleaned, int srcWidth, int srcHeight, double resolution, double orgX, double orgY);
    std::list<tPoint> convertContourToPolygon(std::vector<cv::Point> &contour, tGridmapInfo info);
    std::list<cv::Point> convertRobotpathToCvpath(std::list<tPoint> cleaned, tGridmapInfo info);
    cv::Mat wallLineFit(s8 *pSrc, int srcWidth, int srcHeight );
    cv::Mat wallLineFit(s8 *pSrc, int srcWidth, int srcHeight, std::list<cv::Point> dockingzonePixel );
    cv::Mat linemap(s8 *pSrc, int srcWidth, int srcHeight);    
    cv::Mat linemap(s8 *pSrc, int srcWidth, int srcHeight , std::list<cv::Point> dockingzonePixel);
    void drawCliffPoint(cv::Mat &org, std::list<cv::Point> cliffPoints );
    void channelCheck(cv::Mat img);
    std::vector<unsigned char> convertMat2GridMap(cv::Mat src);    
    std::list<cv::Rect> fakeBsp(cv::Mat src, int kDeleteSize = 10);
    std::vector<std::vector<cv::Point>> fakeBspV2(cv::Mat src, int kDeleteSize = 10);
    std::list<cv::Rect> fakeBspV3(cv::Mat src, int kDeleteSize = 10);
    std::vector<std::vector<cv::Point>> fakeBspV4(cv::Mat src, int kDeleteSize = 10);
    std::list<std::list<tPoint>> findDoor(cv::Mat src);
    double fillAreaCleanPath(cv::Mat& img, const std::vector<cv::Point>& contour, 
        const std::list<cv::Point>& cleanpath, int thickness, 
        std::vector<std::vector<cv::Point>>& uncleandContours);
    std::pair<double, double> getAreaCoverage(cv::Mat& img, const std::vector<cv::Point>& contour, 
        const std::list<cv::Point>& cleanpath, int thickness);
    
    // 컨투어의 중심점을 계산하는 함수
    cv::Point2f computeContourCenter(const std::vector<cv::Point>& contour);
    // 컨투어 크기를 조절하는 함수
    std::vector<std::vector<cv::Point>> resizeContour(const std::vector<std::vector<cv::Point>>& contours, float scale);

private:
    void checkAreaContours(cv::Mat img);
    void checkAreaConvex(cv::Mat img);
    cv::Mat where(const cv::Mat& condition, const cv::Mat& x, const cv::Mat& y);    
    cv::Mat convertGridMap2MatMod(s8 *pSrc, int srcWidth, int srcHeight);
};

