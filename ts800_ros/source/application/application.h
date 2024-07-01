/**
 * @file application.h
 * @author jspark
 * @brief 
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "ebtypedef.h"
#include "rsaDispatcher.h"
#include "commonStruct.h"

enum class APPLICATION_STATE
{
    START_BOOT,
    RUN_BOOT,
    RUN_DISPATCHER,
};
/**
 * @brief 디스패처 루프로 실행시킬 Application 클래스
 */
class Application
{
private: /* not use! */
    Application(const Application &other);
    Application &operator=(const Application &other);

public:
    explicit Application(ros::NodeHandle nh);
    ~Application();
    void init();
    void execute();
    int executeLidar(void);

private:
    bool isRuntimeElapsed(int runtimeHz);

private:
    APPLICATION_STATE state;
    CRsaDispatcher *pRsaDispatcher;
};