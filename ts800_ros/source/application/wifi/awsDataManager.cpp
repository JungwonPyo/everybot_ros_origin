#include"awsDataManager.h"


CAwsDataManager::CAwsDataManager(CRsuContext* _pContext) : pContext(_pContext)
{
    AwsMapSendTick = SYSTEM_TOOL.getSystemTime();
}
CAwsDataManager::~CAwsDataManager(){}


bool CAwsDataManager::checkSendMapTime()
{
    if (SYSTEM_TOOL.getSystemTime() - AwsMapSendTick > MSG_SENDING_INTERVAL)
    {
        ceblog(LOG_LV_AWS, GREEN," now service : " << enumToString(pContext->getServiceId()));
        AwsMapSendTick = SYSTEM_TOOL.getSystemTime();
        return true;
    }
    else {
        return false;
    }
}


//배열로 보내는 형태
/**
 * @brief 맵 데이터, 로봇 coordinate, 탐색 & 청소 trajectory 데이터 전송  
 * 
 */
void CAwsDataManager::sendMapDataToAws2()
{
    bool bPose = false, bExploreTrajectory = false, bCleanTrajectory = false;
    auto isSendMap = false;
    bool bCradle = false;

    bool mapTriger = ServiceData.awsData.mapTrigger;
    auto cmpId = pContext->getServiceId();
    auto cmpStatus = pContext->getServiceStatus(); 

    if((cmpId == E_SERVICE_ID::EXPLORER || cmpId == E_SERVICE_ID::CLEAN || cmpId == E_SERVICE_ID::DOCKING) 
        && (cmpStatus == E_SERVICE_STATUS::completed || cmpStatus == E_SERVICE_STATUS::paused))
    {
        //탐색,청소,도킹의     완료, 일시정지 상태면 지도 데이터 전송 안함
        return;
    }

    if (SUB_TASK.cleanPlan.isSetupNogoZoneDocking())
    {
        bCradle = true;
    }

    if (ServiceData.robotMap.simplifyMap.isInit())
    {
        if((cmpId == E_SERVICE_ID::EXPLORER || cmpId == E_SERVICE_ID::CLEAN 
        || cmpId == E_SERVICE_ID::DOCKING) && mapTriger)
        {
            isSendMap = true;
            ServiceData.awsData.mapTrigger = false;
        }
        else
        {
            isSendMap = false;
            ServiceData.awsData.mapTrigger = true;
        }

        // ceblog(LOG_LV_AWS, YELLOW,"origin_x : "<<ServiceData.robotMap.simplifyMap.getInfo().origin_x
        //                         <<", origin_y : "<<ServiceData.robotMap.simplifyMap.getInfo().origin_y);

#if 0 //map데이터 디버깅
        for(int i = 0; i < mapInfo.width; i++)
        {
            for(int j = 0; j < mapInfo.height; j++ )
            {
                int out = pMapArray[i+j*mapInfo.width]==255 ? 0 : 1; //55보다 작으면 0 == 빈공간  1 == 벽
                //std::cout<<(int)pMapArray[i*j+j];
                std::cout<<out;
            }
            std::cout<<std::endl;
        }

        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
        std::cout<<std::endl;
#endif
    }
    else 
    {
        ceblog(LOG_LV_AWS, YELLOW,"fail copy map OR there is no map");
    }
    

    /* robot pose data */
    if(cmpId == E_SERVICE_ID::EXPLORER || cmpId == E_SERVICE_ID::CLEAN 
    || cmpId == E_SERVICE_ID::DOCKING)
        bPose = true;
    else
        bPose = false;

    /* trajectory data */
    if (cmpId == E_SERVICE_ID::CLEAN)
    {
        //데이터 타입이 리스트 <구조체> 이네 이거 변형해야함 ->double *Data, int size, int seq
        //x,y 좌표가 double 형임
        //send clean path
        if (!ServiceData.robotMap.robotTrajectory.getCleanedTrajectory().empty())//이전 궤적 != 현재 궤적 인경우
            bCleanTrajectory = true;
    }
    else if (cmpId == E_SERVICE_ID::EXPLORER)
    {
        //send explorer path
        if (!ServiceData.robotMap.robotTrajectory.getExploredTrajectory().empty())
            bExploreTrajectory = true;
    } 

    /* send message */
#if 0
    if (isSendMap && bPose && (bExploreTrajectory || bCleanTrajectory))
    {
        ceblog(LOG_LV_AWS, YELLOW,"send map, pose, trajectory data");
        if (bExploreTrajectory)
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
                    tAwsMapData(isSendMap, ServiceData.localiz.getPose(), ServiceData.robotMap.robotTrajectory.getExploredTrajectory())));
        else 
            SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
                    tAwsMapData(isSendMap, ServiceData.localiz.getPose(), ServiceData.robotMap.robotTrajectory.getCleanedTrajectory())));
    }
    else if (isSendMap && bPose && bCradle)
    {
        ceblog(LOG_LV_AWS, YELLOW,"send map, pose data");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
                    tAwsMapData(isSendMap, ServiceData.localiz.getPose(), SUB_TASK.cleanPlan.getNoGoZoneDockingCoord())));
    }
    else if (isSendMap && bPose)
    {
        ceblog(LOG_LV_AWS, YELLOW,"send map, pose data");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
                    tAwsMapData(isSendMap, ServiceData.localiz.getPose())));
    }
    else if (isSendMap && bCradle)
    {
        ceblog(LOG_LV_AWS, YELLOW,"send map data");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
                    tAwsMapData(isSendMap, SUB_TASK.cleanPlan.getNoGoZoneDockingCoord())));
    }
    else if (isSendMap)
    {
        ceblog(LOG_LV_AWS, YELLOW,"send map data");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
                    tAwsMapData(isSendMap)));
    }
    else if (bPose)
    {
        ceblog(LOG_LV_AWS, YELLOW,"send pose data");
        SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
                    tAwsMapData(ServiceData.localiz.getPose())));
    }
    else
        ceblog(LOG_LV_AWS, YELLOW,"there is no any data");
#else
    // if (isSendMap && bPose && (bExploreTrajectory || bCleanTrajectory) && bCradle)
    // {
    //     ceblog(LOG_LV_AWS, YELLOW,"send map, pose, trajectory data");
    //     if (bExploreTrajectory)
    //         SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
    //                 tAwsMapData(serviceData.robotMap.simplifyMap.getSimplifyGridMapPtr(),serviceData.robotMap.simplifyMap.getInfo(), ServiceData.localiz.getPose(),
    //                 ServiceData.robotMap.robotTrajectory.getExploredTrajectory(), SUB_TASK.cleanPlan.getNoGoZoneDockingCoord())));
    //     else 
    //         SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
    //                 tAwsMapData(serviceData.robotMap.simplifyMap.getSimplifyGridMapPtr(),serviceData.robotMap.simplifyMap.getInfo(), ServiceData.localiz.getPose(),
    //                 ServiceData.robotMap.robotTrajectory.getCleanedTrajectory(), SUB_TASK.cleanPlan.getNoGoZoneDockingCoord())));
    // }
    // else if (isSendMap && bPose && (bExploreTrajectory || bCleanTrajectory))
    // {
    //     ceblog(LOG_LV_AWS, YELLOW,"send map, pose, trajectory data");
    //     if (bExploreTrajectory)
    //         SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
    //                 tAwsMapData(serviceData.robotMap.simplifyMap.getSimplifyGridMapPtr(), ServiceData.localiz.getPose(), ServiceData.robotMap.robotTrajectory.getExploredTrajectory())));
    //     else 
    //         SEND_MESSAGE(message_t(E_MESSAGE_TYPE_REPORT_AWS_DATA, 
    //                 tAwsMapData(serviceData.robotMap.simplifyMap.getSimplifyGridMapPtr(), ServiceData.localiz.getPose(), ServiceData.robotMap.robotTrajectory.getCleanedTrajectory())));
    // }

#endif
}