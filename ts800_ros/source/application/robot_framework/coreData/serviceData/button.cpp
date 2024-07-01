#include "coreData/serviceData/button.h"
#include "eblog.h"
#include "systemTool.h"
#include "userInterface.h"

/********** 시간 측정 관련 define **********************/
#define USE_TIME_CHECK 0 // 함수 시간 측정 사용 유무
#define USE_TIME_WARNING_LIMIT 1.0 // 경고가 되는 시간 기준 (단위 ms) (기준보다 오래걸리면 경고문 출력됩니다.)
// @@@ 시작 측정 종료 매크로 @@@
// time 에는 CStopWatch의 getTime()를 넣어주세요.
#define TIME_CHECK_END(time)    {printTime(USE_TIME_CHECK, USE_TIME_WARNING_LIMIT, time);}0
/******************************************************/

#define BUTTON_LONG_TICK    3     //sec
#define BUTTON_SHORT_TICK   0.03   //sec
#define BUTTON_PRESS_TIME   1     //sec
#define FUNC_PLAY_BUTTON_TIME 3   //sec, 1+2버튼 동작시간
#define FUNC_HOME_BUTTON_TIME  5  //sec 1+3버튼 동작시간
#define PLAY_HOME_BUTTON_TIME  3  //sec 2+3버튼 동작시간

CButton::CButton()
{
    initButton();
    
}

CButton::~CButton()
{
            
}

void CButton::initButton()
{
    state.play = E_BUTTON_PRESS_TIME::NOT_WORK;
    state.home = E_BUTTON_PRESS_TIME::NOT_WORK;
    state.func = E_BUTTON_PRESS_TIME::NOT_WORK;
    buttonKey = E_FUNCTION_BUTTON::NONE;
}

void CButton::setButtonKey(tSysButton sysData)
{
    //tButtonState bState;        

    bState.play = makeButtonState(sysData.buttonStart, &playChecker);    
    bState.home = makeButtonState(sysData.buttonHome, &homeChecker);    
    bState.func = makeButtonState(sysData.buttonFunc, &funcChecker);
    bState.sort = makeButtonMode(&multiKey, &funcChecker, &playChecker, &homeChecker);

    buttonKey = makeButtonKeyFunc(bState, &multiKey); 
}

E_BUTTON_PRESS_TIME CButton::getPlayButtonState(void)
{
    return state.play;
}

E_BUTTON_PRESS_TIME CButton::getHomeButtonState(void)
{
    return state.home;
}

E_BUTTON_PRESS_TIME CButton::getFuncButtonState(void)
{
    return state.func;
}

E_FUNCTION_BUTTON CButton::getButtonKey()
{
    E_FUNCTION_BUTTON ret = buttonKey;
    buttonKey = E_FUNCTION_BUTTON::NONE;
#if 0
    if (ret != E_KEY_FUNCTION::NONEKEY)
        eblog(LOG_LV_NECESSARY,  "E_KEY_FUNCTION : "<<E_KEY_FUNCTION_STR[(int) ret]);

    return E_KEY_FUNCTION::NONEKEY;
#endif
    return ret;
}

E_BUTTON_PRESS_SORT CButton::getBtnMode()
{
    return bState.sort; 
} 

void CButton::checkerClear(tButtonChecker *pChecker)
{
    pChecker->startTirgTime = SYSTEM_TOOL.getSystemTime();
    pChecker->runTime = 0.0;
    pChecker->trig = false;
    pChecker->shortCheck = false;
    pChecker->longCheck = false;
    pChecker->btnLock = false;
    pChecker->ledLock = false;
    pChecker->soundLock = false;
}

void CButton::multiButtonClear(tMultiInput *pMulti)
{
    pMulti->multiStartTime = SYSTEM_TOOL.getSystemTick();
    pMulti->multiRunTime = 0.0;
    pMulti->funcPlayChecker = false;
    pMulti->funcHomeChecker = false;
    pMulti->playHomeChecker = false;
}

bool CButton::isButtonPressed(bool button, tButtonChecker *pChecker)
{
    bool ret = false;

    if (button == true && pChecker->trig == false)
    {
        // 버튼 입력 트리거 발동.
        pChecker->trig = true;
        pChecker->startTirgTime = SYSTEM_TOOL.getSystemTime();
        ret = true;
        //eblog(LOG_LV_NECESSARY, "triger on , time : "<<pChecker->startTirgTime);
    }
    else if (button == true && pChecker->trig == true)
    {
        //버튼 눌리고 있는 상태
        ret = true;
    }
    else if (button == false && pChecker->trig == false)
    {
        // 어떤 경우지?? -> 처음 상태 or 버튼 해제 후 트리거 발동 한 상태-> 의미있나?
    }
    else if (button == false && pChecker->trig == true)
    {
        //버튼 해제 및 트리거 발동.
        //eblog(LOG_LV_NECESSARY, "triger off , time : "<<SYSTEM_TOOL.getSystemTime() - pChecker->startTirgTime );
        pChecker->trig = false;        
        pChecker->startTirgTime = 0.0;
    }
    else
    {
        //에러
        eblog(LOG_LV_NECESSARY, "error : "<<SYSTEM_TOOL.getSystemTime());
    }

    return ret;
}

E_BUTTON_PRESS_TIME CButton::makeButtonState(bool button, tButtonChecker *pChecker)
{
    E_BUTTON_PRESS_TIME ret = E_BUTTON_PRESS_TIME::NOT_WORK;        

    //싱글에서만 할 것인지, 멀티에서 할 것인지

    //버튼이 눌리고 있는 중
    if (isButtonPressed(button, pChecker))
    {
        pChecker->runTime = SYSTEM_TOOL.getSystemTime() - pChecker->startTirgTime;

        //short key 판정.
        if ( pChecker->runTime > BUTTON_SHORT_TICK )
        {
            pChecker->shortCheck = true;
            //eblog(LOG_LV_NECESSARY, "short : "<<pChecker->runTime);

            // 버튼 LED
            if(pChecker->ledLock == false && pChecker->runTime < BUTTON_LONG_TICK)    
            {
                LED_CTR.setBtnLedTick();
                LED_CTR.btnLedBlue();
                pChecker->ledLock = true;
            }
            else if(SYSTEM_TOOL.getSystemTick() - LED_CTR.getBtnLedTick() >= 0.5*SEC_1 && pChecker->runTime < BUTTON_LONG_TICK)
            {
                LED_CTR.btnLedBlue();
                LED_CTR.setBtnLedTick();
            }

#if 0 //버튼 사운드
            if (pChecker->soundLock == false)           
            {
                SOUND_CTR.soundPlay(E_SoundClass::SOUND_DONE_EFFECT);
                pChecker->soundLock = true;
            }    
#endif
        }

        //long key 판정.
        if ( pChecker->runTime > BUTTON_LONG_TICK && pChecker->btnLock == false)
        {
            pChecker->longCheck = true; 
            //eblog(LOG_LV_NECESSARY, "long : "<<pChecker->runTime);

            // // 버튼 LED
            // if(pChecker->ledLock == false)    
            // {
            //     LED_CTR.setBtnLedTick();
            //     LED_CTR.btnLedWhite();
            //     pChecker->ledLock = true;
            // }
            // else if(SYSTEM_TOOL.getSystemTick() - LED_CTR.getBtnLedTick() >= 0.5*SEC_1)
            // {
            //     LED_CTR.btnLedWhite();
            //     LED_CTR.setBtnLedTick();
            // }
            //checkerClear(pChecker);
            LED_CTR.ledAllOff();
            pChecker->btnLock = true;
            ret = E_BUTTON_PRESS_TIME::LONG;
            eblog(LOG_LV_NECESSARY, "long press !!");
        }
    }
    // 버튼을 땠을때->이게 트리거임
    else
    {
        //short key 확정.
        if (pChecker->shortCheck == true && pChecker->longCheck == false)
        {
            ret = E_BUTTON_PRESS_TIME::SHORT;
            eblog(LOG_LV_NECESSARY, "short press !!");

            //LED off
            LED_CTR.ledAllOff();
        }
#if 0
        //long key 확정.
        if (pChecker->shortCheck == true && pChecker->longCheck == true)
        {
            ret = E_BUTTON_PRESS_TIME::LONG;
            eblog(LOG_LV_NECESSARY, "long press !!");

            //LED off
            LED_CTR.ledAllOff();
        }
#endif
        checkerClear(pChecker);
    }

    return ret;
}

//싱글버튼 short 0.2초, long 3초
E_BUTTON_PRESS_SORT CButton::makeButtonMode(tMultiInput *pMulti,tButtonChecker *pFuncChecker, tButtonChecker *pPlayChecker, tButtonChecker *pHomeChecker)
{
    E_BUTTON_PRESS_SORT ret = getBtnMode();

    if(ret == E_BUTTON_PRESS_SORT::SINGLE_BUTTON)
    {
        //다른 버튼 press 확인
        ret = checkOtherButtonPress(pMulti, &funcChecker, &playChecker, &homeChecker);
    }
    else if (ret == E_BUTTON_PRESS_SORT::MULTI_BUTTON)
    {
        //버튼누르면 동작 LED
        if(SYSTEM_TOOL.getSystemTick() - LED_CTR.getBtnLedTick() >= 0.5*SEC_1 && pMulti->ledLock == false)
        {
            LED_CTR.btnLedWhite();
            LED_CTR.setBtnLedTick();
            eblog(LOG_LV_NECESSARY, "multi LED ");
        }
#if 0 //버튼누르면 동작 사운드
        if (pMulti->soundLock == false)           
        {
            SOUND_CTR.soundPlay(E_SoundClass::SOUND_DONE_EFFECT);
            pMulti->soundLock = true;
        }
#endif
        //다른 버튼 unpress확인    
        ret = checkOtherButtonUnpress(pMulti, &funcChecker, &playChecker, &homeChecker);
        
        if (pMulti->funcPlayChecker == false &&
            pMulti->funcHomeChecker == false &&
            pMulti->playHomeChecker == false)
        {
            // eblog(LOG_LV_NECESSARY, "funcplayChecker : "<<pMulti->funcPlayChecker);
            // eblog(LOG_LV_NECESSARY, "funchomeChecker : "<<pMulti->funcHomeChecker);
            // eblog(LOG_LV_NECESSARY, "playhomeChecker : "<<pMulti->playHomeChecker);        
            return  ret;
        }
    }    

    return ret;
}

E_BUTTON_PRESS_SORT CButton::checkOtherButtonPress(tMultiInput *pMulti, tButtonChecker *pFuncChecker, tButtonChecker *pPlayChecker, tButtonChecker *pHomeChecker)
{
    E_BUTTON_PRESS_SORT ret = E_BUTTON_PRESS_SORT::SINGLE_BUTTON; 
    //eblog(LOG_LV_NECESSARY, "checkOtherButtonPress : ");
    
    //1+2 눌려진 상태
    if (pFuncChecker->trig == true &&
        pPlayChecker->trig == true &&
        pHomeChecker->trig == false)
    {
        eblog(LOG_LV_NECESSARY, "funcPlayChecker : true");
        pMulti->funcPlayChecker = true;
        //멀티 초기화
        pMulti->multiStartTime = SYSTEM_TOOL.getSystemTick();
        pMulti->btnLock = false;

        // LED, SOUND 잠금 해제
        pMulti->ledLock = false;
        pMulti->soundLock = false;
        
        ret = E_BUTTON_PRESS_SORT::MULTI_BUTTON;
    }

    //1+3 눌려진 상태
    if (pFuncChecker->trig == true &&
        pPlayChecker->trig == false &&
        pHomeChecker->trig == true)
    {
        eblog(LOG_LV_NECESSARY, "funcHomeChecker : true");
        pMulti->funcHomeChecker = true;
        //멀티 초기화
        pMulti->multiStartTime = SYSTEM_TOOL.getSystemTick();
        pMulti->btnLock = false;

        // LED, SOUND 잠금 해제
        pMulti->ledLock = false;
        pMulti->soundLock = false;

        ret = E_BUTTON_PRESS_SORT::MULTI_BUTTON;
    }

    // 2+3 눌려진 상태  
    if (pFuncChecker->trig == false&&
        pPlayChecker->trig == true&&
        pHomeChecker->trig == true)
    {
        //멀티 초기화
        eblog(LOG_LV_NECESSARY, "playHomeChecker : true");
        pMulti->playHomeChecker = true;
        pMulti->multiStartTime = SYSTEM_TOOL.getSystemTick();
        pMulti->btnLock = false;

        // LED, SOUND 잠금 해제
        pMulti->ledLock = false;
        pMulti->soundLock = false;
        ret = E_BUTTON_PRESS_SORT::MULTI_BUTTON;
    }


    return ret;
}

 E_BUTTON_PRESS_SORT CButton::checkOtherButtonUnpress(tMultiInput *pMulti, tButtonChecker *pFuncChecker, tButtonChecker *pPlayChecker, tButtonChecker *pHomeChecker)
 {
    E_BUTTON_PRESS_SORT ret = E_BUTTON_PRESS_SORT::MULTI_BUTTON;
    
    //1버튼만 눌리는 경우
    //다른 키가 눌리면 버튼을 초기화 하기 때문에 trig가 안됨
    if(pFuncChecker->trig == true&&
        pPlayChecker->trig == false&&
        pHomeChecker->trig == false) 
    {
        eblog(LOG_LV_NECESSARY, "init Button");
        checkerClear(pFuncChecker);
        checkerClear(pPlayChecker);
        checkerClear(pHomeChecker);
        bState.func=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.play=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.home=E_BUTTON_PRESS_TIME::NOT_WORK;
        LED_CTR.ledAllOff();

        ret = E_BUTTON_PRESS_SORT::SINGLE_BUTTON;
    }   
    
    //2버튼만 눌리는 경우
    else if(pFuncChecker->trig == false&&
            pPlayChecker->trig == true&&
            pHomeChecker->trig == false) 
    {
        eblog(LOG_LV_NECESSARY, "init Button");
        checkerClear(pFuncChecker);
        checkerClear(pPlayChecker);
        checkerClear(pHomeChecker);
        bState.func=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.play=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.home=E_BUTTON_PRESS_TIME::NOT_WORK;
        LED_CTR.ledAllOff();

        ret = E_BUTTON_PRESS_SORT::SINGLE_BUTTON;
    }   
    
    //3버튼만 눌리는 경우
    else if(pFuncChecker->trig == false&&
            pPlayChecker->trig == false&&
            pHomeChecker->trig == true)
    {
        eblog(LOG_LV_NECESSARY, "init Button");
        checkerClear(pFuncChecker);
        checkerClear(pPlayChecker);
        checkerClear(pHomeChecker);
        bState.func=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.play=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.home=E_BUTTON_PRESS_TIME::NOT_WORK;
        LED_CTR.ledAllOff();

        ret = E_BUTTON_PRESS_SORT::SINGLE_BUTTON;
    }

    // 모든 btn off
    else if(pFuncChecker->trig == false&&
            pPlayChecker->trig == false&&
            pHomeChecker->trig == false)
    {
        eblog(LOG_LV_NECESSARY, "init Button");
        checkerClear(pFuncChecker);
        checkerClear(pPlayChecker);
        checkerClear(pHomeChecker);
        bState.func=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.play=E_BUTTON_PRESS_TIME::NOT_WORK;
        bState.home=E_BUTTON_PRESS_TIME::NOT_WORK;
        LED_CTR.ledAllOff();

        ret = E_BUTTON_PRESS_SORT::SINGLE_BUTTON;
    }
    // unpress 된 버튼이 없는 경우 = 멀티버튼 상태   
    else
    {
        makeMultiButtonState(pMulti);
        //초기화 위치를 바꿀 건지 멀티 관련 초기화 함수를 만들어야 할지 고민해야함
        ret = E_BUTTON_PRESS_SORT::MULTI_BUTTON;
    }

    return ret;
 }

//멀티버튼 1+2: 3초, 1+3: 10초
void CButton::makeMultiButtonState(tMultiInput *pMulti)
{
    //eblog(LOG_LV_NECESSARY, "make multi btn state");
    E_BUTTON_PRESS_TIME ret = E_BUTTON_PRESS_TIME::NOT_WORK;

    pMulti->multiRunTime = SYSTEM_TOOL.getSystemTick() - pMulti->multiStartTime;

    //1+2 가 눌린 경우
    if (pMulti->funcPlayChecker)
    {   
        //eblog(LOG_LV_NECESSARY, "multiBtnTime:"<< pMulti->multiRunTime);

        // x초 이후 && 잠금 off
        if (pMulti->multiRunTime >=  FUNC_PLAY_BUTTON_TIME*SEC_1 && pMulti->btnLock == false )
        {
            pMulti->multiStartTime = SYSTEM_TOOL.getSystemTick();

            eblog(LOG_LV_NECESSARY, "multi long - func+play");
            // 각 키에 Long 값을 세팅
            bState.func=E_BUTTON_PRESS_TIME::LONG;
            bState.play=E_BUTTON_PRESS_TIME::LONG;
            bState.home=E_BUTTON_PRESS_TIME::NOT_WORK;
            pMulti->btnLock = true;

            //LED off
            pMulti->ledLock = true;
            LED_CTR.ledAllOff();
        }
    }
    //1+3 이 눌린 경우
    if (pMulti->funcHomeChecker)
    {
        //eblog(LOG_LV_NECESSARY, "multiBtnTime:"<< pMulti->multiRunTime);

        if (pMulti->multiRunTime >= FUNC_HOME_BUTTON_TIME*SEC_1 && pMulti->btnLock == false )
        {
            pMulti->multiStartTime = SYSTEM_TOOL.getSystemTick();

            eblog(LOG_LV_NECESSARY, "multi long - home+func");
            // 각 키에 Long 값을 세팅
            bState.func=E_BUTTON_PRESS_TIME::LONG;
            bState.play=E_BUTTON_PRESS_TIME::NOT_WORK;
            bState.home=E_BUTTON_PRESS_TIME::LONG;
            pMulti->btnLock = true;
            
            //LED off
            pMulti->ledLock = true;
            LED_CTR.ledAllOff();
        }
    }
    //2+3 이 눌린 경우
    if (pMulti->playHomeChecker)
    {
        //눌리고 있는 시간에 따라 long, SHORT 결정
        if (pMulti->multiRunTime >= PLAY_HOME_BUTTON_TIME*SEC_1 && pMulti->btnLock == false )
        {
            pMulti->multiStartTime = SYSTEM_TOOL.getSystemTick();

            eblog(LOG_LV_NECESSARY, "multi long - play+home");
            // 각 키에 Long 값을 세팅
            bState.func=E_BUTTON_PRESS_TIME::NOT_WORK;
            bState.play=E_BUTTON_PRESS_TIME::LONG;
            bState.home=E_BUTTON_PRESS_TIME::LONG;
            pMulti->btnLock = true;
            
            //LED off
            pMulti->ledLock = true;
            LED_CTR.ledAllOff();
        }  
    }
}

E_FUNCTION_BUTTON CButton::makeButtonKeyFunc(tButtonState state, tMultiInput *pMulti)
{
    E_FUNCTION_BUTTON funcButton = E_FUNCTION_BUTTON::NONE;

    if(state.sort == E_BUTTON_PRESS_SORT::SINGLE_BUTTON)
    {
        //eblog(LOG_LV_NECESSARY, "make Btn Key single");
        //물 공급량 변경
        if( state.func == E_BUTTON_PRESS_TIME::SHORT     &&
            state.play == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.home == E_BUTTON_PRESS_TIME::NOT_WORK  )
        {
            funcButton = E_FUNCTION_BUTTON::FUNC_SHORT;
        }

        //청소 시작
        if( state.func == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.play == E_BUTTON_PRESS_TIME::SHORT     &&
            state.home == E_BUTTON_PRESS_TIME::NOT_WORK  )
        {
            funcButton = E_FUNCTION_BUTTON::START_SHORT;
        }

        //충전 복귀
        if( state.func == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.play == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.home == E_BUTTON_PRESS_TIME::SHORT  )
        {
            funcButton = E_FUNCTION_BUTTON::HOME_SHORT;
        }

        //전원 off
        if( state.func == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.play == E_BUTTON_PRESS_TIME::LONG      &&
            state.home == E_BUTTON_PRESS_TIME::NOT_WORK  )
        {
            funcButton = E_FUNCTION_BUTTON::START_LONG;
        }

        //잔수 제거
        if( state.func == E_BUTTON_PRESS_TIME::LONG      &&
            state.play == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.home == E_BUTTON_PRESS_TIME::NOT_WORK  )
        {
            funcButton = E_FUNCTION_BUTTON::FUNC_LONG;
        }

        //건조 토글
        if( state.func == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.play == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.home == E_BUTTON_PRESS_TIME::LONG      )
        {
            funcButton = E_FUNCTION_BUTTON::HOME_LONG;
        }

    }
    else if (state.sort == E_BUTTON_PRESS_SORT::MULTI_BUTTON)
    {
        //Wi-Fi(1+2)
        if( state.func == E_BUTTON_PRESS_TIME::LONG  &&
            state.play == E_BUTTON_PRESS_TIME::LONG  &&
            state.home == E_BUTTON_PRESS_TIME::NOT_WORK )
        {
            funcButton = E_FUNCTION_BUTTON::START_FUNC_LONG;
            multiButtonClear(pMulti);
        }

        //초기화(1+3)
        if( state.func == E_BUTTON_PRESS_TIME::LONG  &&
            state.play == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.home == E_BUTTON_PRESS_TIME::LONG )
        {
            funcButton = E_FUNCTION_BUTTON::HOME_FUNC_LONG;
            multiButtonClear(pMulti);
        }

        
        //차일드락(2+3)
        if( state.func == E_BUTTON_PRESS_TIME::NOT_WORK  &&
            state.play == E_BUTTON_PRESS_TIME::LONG  &&
            state.home == E_BUTTON_PRESS_TIME::LONG )
        {
            funcButton = E_FUNCTION_BUTTON::START_HOME_LONG;
            multiButtonClear(pMulti);
        }
    }

    if(funcButton != E_FUNCTION_BUTTON::NONE)
    {            
        eblog(LOG_LV_NECESSARY, "buttonKey : "<<(int)funcButton);
    }
    
    
    return funcButton; 
}
