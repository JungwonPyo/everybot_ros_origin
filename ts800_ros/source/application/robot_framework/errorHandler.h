/**
 * @file errorHandler.h
 * @author jspark
 * @brief 수집된 에러들을 어떻게 처리할지 결정함.
 * 감지 기준의 에러와 처리 기준의 에러가 1대 1이 아닌 경우,
 * 감지 기준의 에러 종류와, 처리 기준의 에러 종류 등의 분류가 필요함.
 * 
 * @date 2023-05-19
 */
#pragma once

#include "errorCollector.h"
#include "coreData/serviceData.h"
#include "rsuContext.h"



class CErrorHandler
{
private:
    E_ERROR_HANDLING_TYPE lastErrorType;
    bool bErrorActive;

public:
    CErrorHandler(/* args */);
    ~CErrorHandler();
    bool procError(E_ERROR_HANDLING_TYPE errCmd, E_KEY_TYPE key, bool extPower);

protected:
    virtual E_ERROR_HANDLING_TYPE makeError(tErrorInfo errInf) = 0;
    virtual void handleContinuousCliffError() = 0;
    virtual void handleTiltError() = 0;
    virtual void handleMissingRosDataError() = 0;
    virtual void handleCommunicationError() = 0;
    virtual void handleDockingFailError() = 0;
    virtual void handleWheelError() = 0;
    virtual void handleTrapError() = 0;

    bool isErrorActive();
    void activateError(E_ERROR_HANDLING_TYPE type);
    void deactivateError();

    bool isContinuousCliffError(tErrorInfo errInf, CRsuContext *pContext);
    bool isCommunicationError(tErrorInfo errInf, CRsuContext *pContext);
    bool isTiltError(tErrorInfo errInf, CRsuContext *pContext);
    bool isDockinFailError(tErrorInfo errInf, CRsuContext *pContext);
    bool isMissingRosDataError(tErrorInfo errInf, CRsuContext *pContext);
    bool isWheelError(tErrorInfo errInf, CRsuContext *pContext);
    bool isTrapError(tErrorInfo errInf, CRsuContext *pContext);

    bool checkKeyForErrorDeactivate(E_KEY_TYPE key, bool extPower);
    void handleExitSlam();

    void setErrorSound(E_ERROR_HANDLING_TYPE type);
    E_ERROR_HANDLING_TYPE getErrorSound();

private: /* debug */
    int __debugPrintCnt;
};