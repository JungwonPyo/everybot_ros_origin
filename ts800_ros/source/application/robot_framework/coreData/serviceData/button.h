/**
 * @file button.h
 * @author jmk1
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <iostream>
#include "commonStruct.h"
#include "interfaceStruct.h"

enum class E_BUTTON_PRESS_TIME{
    NOT_WORK,
    SHORT,
    LONG,
};

enum class E_BUTTON_PRESS_SORT{
    VOID,
    SINGLE_BUTTON,
    MULTI_BUTTON,
};

enum class E_FUNCTION_BUTTON
{
    NONE,
    START_SHORT,
    START_LONG,
    HOME_SHORT,
    HOME_LONG,
    FUNC_SHORT,
    FUNC_LONG,
    START_HOME_SHORT,
    START_HOME_LONG,
    START_FUNC_SHORT,
    START_FUNC_LONG,
    HOME_FUNC_SHORT,
    HOME_FUNC_LONG,
};

static std::string enumToString(E_FUNCTION_BUTTON value) {
    static const std::unordered_map<E_FUNCTION_BUTTON, std::string> enumToStringMap = {
        { E_FUNCTION_BUTTON::NONE, "NONE," },
        { E_FUNCTION_BUTTON::START_SHORT, "START_SHORT," },
        { E_FUNCTION_BUTTON::START_LONG, "START_LONG," },
        { E_FUNCTION_BUTTON::HOME_SHORT, "HOME_SHORT," },
        { E_FUNCTION_BUTTON::HOME_LONG, "HOME_LONG," },
        { E_FUNCTION_BUTTON::FUNC_SHORT, "FUNC_SHORT," },
        { E_FUNCTION_BUTTON::FUNC_LONG, "FUNC_LONG," },
        { E_FUNCTION_BUTTON::START_HOME_SHORT, "START_HOME_SHORT," },
        { E_FUNCTION_BUTTON::START_HOME_LONG, "START_HOME_LONG," },
        { E_FUNCTION_BUTTON::START_FUNC_SHORT, "START_FUNC_SHORT," },
        { E_FUNCTION_BUTTON::START_FUNC_LONG, "START_FUNC_LONG," },
        { E_FUNCTION_BUTTON::HOME_FUNC_SHORT, "HOME_FUNC_SHORT," },
        { E_FUNCTION_BUTTON::START_HOME_LONG, "START_HOME_LONG," },
        { E_FUNCTION_BUTTON::START_HOME_LONG, "START_HOME_LONG," },
        { E_FUNCTION_BUTTON::HOME_FUNC_LONG, "HOME_FUNC_LONG," },

    };

    auto it = enumToStringMap.find(value);
    if (it != enumToStringMap.end()) {
        return it->second;
    } else {
        return "Unknown";
    }
}

typedef struct _tButtonState
{
    _tButtonState() : play{E_BUTTON_PRESS_TIME::NOT_WORK}, home{E_BUTTON_PRESS_TIME::NOT_WORK}, func{E_BUTTON_PRESS_TIME::NOT_WORK}, sort{E_BUTTON_PRESS_SORT::SINGLE_BUTTON} {}
    E_BUTTON_PRESS_TIME play;
    E_BUTTON_PRESS_TIME home;
    E_BUTTON_PRESS_TIME func;
    E_BUTTON_PRESS_SORT sort;
}tButtonState;

typedef struct _tButtonChecker
{
    _tButtonChecker() : startTirgTime{0.0}, runTime{0.0}, trig{false}, shortCheck{false}, longCheck{false}, soundLock{false}, ledLock{false} {}
    double startTirgTime;
    double runTime;
    bool trig;
    bool shortCheck;
    bool longCheck;
    bool btnLock;
    bool ledLock;
    bool soundLock;
}tButtonChecker;

typedef struct _tMultiInput
{
    _tMultiInput() : multiStartTime{0.0}, multiRunTime{0.0}, funcPlayChecker{false}, funcHomeChecker{false}, playHomeChecker{false}, soundLock{false}, ledLock{false}{}
    double multiStartTime;
    double multiRunTime;
    bool funcPlayChecker;
    bool funcHomeChecker;
    bool playHomeChecker;
    bool btnLock;
    bool ledLock;
    bool soundLock;
}tMultiInput;

class CButton
{
private:
    E_FUNCTION_BUTTON buttonKey;
    tButtonChecker playChecker;
    tButtonChecker homeChecker;
    tButtonChecker funcChecker;        
    tButtonState state;
    tButtonState bState;
    tMultiInput multiKey;
    
public:
    CButton();
    ~CButton();

    void initButton();
    void setButtonKey(tSysButton);
    E_BUTTON_PRESS_TIME getPlayButtonState();
    E_BUTTON_PRESS_TIME getHomeButtonState();
    E_BUTTON_PRESS_TIME getFuncButtonState();
    E_FUNCTION_BUTTON getButtonKey();
    E_BUTTON_PRESS_SORT getBtnMode();  

private:
    void checkerClear(tButtonChecker *pChecker);
    void multiButtonClear(tMultiInput *pMulti);
    
    //버튼 동작 기본 함수
    bool isButtonPressed(bool button, tButtonChecker *pChecker);
    E_BUTTON_PRESS_TIME makeButtonState(bool button, tButtonChecker *pChecker);
    E_BUTTON_PRESS_SORT makeButtonMode(tMultiInput *pMulti, tButtonChecker *pFuncChecker, tButtonChecker *pPlayChecker, tButtonChecker *pHomeChecker);

    // 멀티, 싱글 버튼 구분 함수 
    E_BUTTON_PRESS_SORT checkOtherButtonPress(tMultiInput *pMulti, tButtonChecker *pFuncChecker, tButtonChecker *pPlayChecker, tButtonChecker *pHomeChecker);    
    E_BUTTON_PRESS_SORT checkOtherButtonUnpress(tMultiInput *pMulti, tButtonChecker *pFuncChecker, tButtonChecker *pPlayChecker, tButtonChecker *pHomeChecker);   
    
    //멀티 버튼 상태 정해주는 함수
    void makeMultiButtonState(tMultiInput *pMulti);

    //최종 버튼 메시지 전송 함수
    E_FUNCTION_BUTTON makeButtonKeyFunc(tButtonState state, tMultiInput *pMulti);
};
