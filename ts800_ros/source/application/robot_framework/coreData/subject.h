/**
 * @file subject.h
 * @author jspark
 * @brief 업데이트할 옵저버들을 등록하고 업데이트 공지를 하는 Subject 클래스
 * @date 2023-05-04
 */
#pragma once

#include "coreData/observer.h"
#include "externData/externData.h"

#include <vector>

class CSubject
{
private:
    std::vector<CObserver*> observers;

public:
    CSubject();
    ~CSubject();
    void attach(class CObserver* observer);
    void detach(class CObserver* observer);
    void notify(CExternData* pExternData);
};