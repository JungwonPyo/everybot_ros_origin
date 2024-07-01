#pragma once

#include <iostream>
#include <thread>
#include <unordered_map>
#include <mutex>
#include "control/motionController.h"
#include "eblog.h"

enum class subTaskState
{
    none,       // 작업중이지 않음.
    running,    // 작업중..
    complete,   // 작업 완료
    fail,       // 작업 실패
};

static std::string enumToString(subTaskState value) {
    static const std::unordered_map<subTaskState, std::string> enumToStringMap = {
        { subTaskState::none,      "Subtask상태[ none ]" },
        { subTaskState::running,   "Subtask상태[ 실행중 ]" },
        { subTaskState::complete,  "Subtask상태[ 완료 ]" },
        { subTaskState::fail,      "Subtask상태[ 실패 ]" },
    };
    auto it = enumToStringMap.find(value);
    return (it != enumToStringMap.end()) ? it->second : "Subtask상태[ unknown ]";
}

template<typename inputT, typename outputT>
class AbstractSubTask
{
private:
    std::mutex threadMutex;
    std::thread backgroundThread;
    subTaskState taskState;
    std::string name;
    int debugCnt;

public:
    AbstractSubTask(std::string taskName) : name{taskName}, taskState{subTaskState::none}, isStart{false} {
        debugCnt = 0;
    }

    virtual ~AbstractSubTask() {}

    std::string getName() {return name;}

    void start(inputT inputData)
    {
        if(backgroundThread.joinable()) stop();

        std::lock_guard<std::mutex> lock(threadMutex);
        ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDWHITE<<name<<BOLDGREEN<<"\t시작.  : "<<++debugCnt);
        this->inputData = inputData;
        this->isStart = true;
        setTaskState(subTaskState::running);        
        backgroundThread = std::thread(&AbstractSubTask::loop, this);
    }

    void stop()
    {
        std::lock_guard<std::mutex> lock(threadMutex);

        if(CMotionController::getInstance().isRunning())  CMotionController::getInstance().stop(); // 제어가 살아 있으면 종료.
        setTaskState(subTaskState::none);
        if (backgroundThread.joinable())
        {
            backgroundThread.join();
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDWHITE<<name<<BOLDYELLOW<<"\t정지. : "<<--debugCnt);
        }
    }

    bool isRunning()    {return (taskState==subTaskState::running) ? true : false;}
    
    bool isComplete()   {return (taskState==subTaskState::complete) ? true : false;}

    subTaskState getTaskState() {return taskState;}


protected:
    inputT inputData;
    bool isStart;
    void setTaskState(subTaskState taskState)
    {
        if( this->taskState != taskState )
        {
            ceblog(LOG_LV_NECESSARY, BOLDBLACK, BOLDWHITE<<name<<BOLDBLACK<<" 상태변경. \t"<<BOLDWHITE<<enumToString(this->taskState)<<BOLDBLACK<<"--> "<<BOLDCYAN<<enumToString(taskState));
        }
        this->taskState = taskState;
    }

    /**
     * @brief start()가 실행되었을 때, 초기화할 것들.
     */
    virtual void init() = 0;

public:
    virtual void loop() = 0;
    virtual void proc() = 0;
    virtual outputT getResult() = 0;
};