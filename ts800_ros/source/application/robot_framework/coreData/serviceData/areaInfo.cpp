#include "coreData/serviceData/areaInfo.h"
#include "coreData/serviceData.h"
#include "eblog.h"

CAreaInfo::CAreaInfo(){}
CAreaInfo::~CAreaInfo(){}

void CAreaInfo::setAreaInfo()
{
    // if(ServiceData.rsBridge.getAreaInfo().second != 0)
    // {
    //     tAreaInfo* areaInfoPtr = ServiceData.rsBridge.getAreaInfo().first;
    //     areaInfoData.areaNumber = ServiceData.rsBridge.getAreaInfo().second;
    //     if(!areaInfoData.info.empty())
    //     {
    //         areaInfoData.info.clear();
    //     }
    //     for (int i = 0; i < areaInfoData.areaNumber; ++i) {
    //         areaInfoData.info.push_back(areaInfoPtr[i]);
    //         eblog(LOG_LV_AWS," areaInfo.info "<<"[" << i << "] : id = " << areaInfoData.info.back().id 
    //                                 <<"[" << i << "] : x1 = " << areaInfoData.info.back().x1
    //                                 <<"[" << i << "] : y1 = " << areaInfoData.info.back().y1
    //                                 <<"[" << i << "] : x2 = " << areaInfoData.info.back().x2
    //                                 <<"[" << i << "] : y2 = " << areaInfoData.info.back().y2
    //                                 <<"[" << i << "] : roomName = " << areaInfoData.info.back().name
    //                                 <<"[" << i << "] : color = " << areaInfoData.info.back().color);
    //     }
        
    //     ServiceData.awsData.setSendAreaInfo();
    // }
}
tAreaInfoData CAreaInfo::getAreaInfo()
{
    return areaInfoData;
}

void CAreaInfo::setEditAreaData()
{
    if(ServiceData.rsBridge.getDivideAreaInfo().second != 0){
        tDivideArea *pDivide = ServiceData.rsBridge.getDivideAreaInfo().first;
        editArea.divideNum = ServiceData.rsBridge.getDivideAreaInfo().second;
        editArea.divide.id = pDivide->id;
        ceblog(LOG_LV_NECESSARY, CYN," divide Num : " << editArea.divideNum << " divide id : " << editArea.divide.id);
        for(int i = 0; i < editArea.divideNum; i++)
        {
            editArea.divide.point[i] = pDivide->point[i];
           if(i%2  == 0){
                ceblog(LOG_LV_NECESSARY, CYN," polygon X : " << editArea.divide.point[i]);
            }
            else{
                ceblog(LOG_LV_NECESSARY, CYN," polygon Y : " << editArea.divide.point[i]);
            }
           
        }
        ServiceData.awsData.setSendDivideArea(pDivide);
    }
    if(ServiceData.rsBridge.getCombineAreaInfo().second != 0){
        tCombieArea *pCombine = ServiceData.rsBridge.getCombineAreaInfo().first;
        editArea.combineNum = ServiceData.rsBridge.getCombineAreaInfo().second;
        ceblog(LOG_LV_NECESSARY, CYN," combine Num : " << editArea.combineNum);
        int i = 0;
        for(int i = 0; i < editArea.combineNum; i++)
        {
            editArea.combine.point[i] = pCombine->point[i];
            ceblog(LOG_LV_NECESSARY, CYN," room id : " << editArea.combine.point[i]);
        }
        ServiceData.awsData.setSendCombineArea(pCombine); 
    }

}

tDivideArea CAreaInfo::getDivideAreaData()
{
    return editArea.divide;
}

tCombieArea CAreaInfo::getCombineAreaData()
{
    return editArea.combine;
}

tAreaEdit CAreaInfo::getEditAreaData()
{
    return editArea;
}