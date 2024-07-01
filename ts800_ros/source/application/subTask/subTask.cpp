#include "subTask.h"

CSubTask::CSubTask()
{
}

CSubTask::~CSubTask()
{

}

CSubTask &CSubTask::getInstance()
{
    static CSubTask s;
    return s;
}

void CSubTask::init()
{

}
