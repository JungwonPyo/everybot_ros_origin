/**
 * @file observer.h
 * @author jspark
 * @brief 데이터를 가공하는 옵저버 추상 클래스
 * @date 2023-05-04
 */
#pragma once

class CExternData;  // 상호 참조 방지 코드.

class CObserver
{  
public:
    CObserver();
    ~CObserver();
    virtual void update(CExternData* pExternData) = 0;
};