#include "rmcl.h"
#include "eblog.h"

CRmcl::CRmcl(void):  
        
        // 측정 모델 파라미터
        measurementModelType_(0),   // 측정 모델의 종류 선택 : 0은 likelihood field model, 1은 beam model, 2는 class conditional measurement model
                                    //                       class conditional measurement model은 pose tracking에 권장되는 모델로, 위치 추정에 더 나은 성능 예상
        scanStep_(5),               // 측정 값 중 일부를 건너 뛰는 스캔 스텝 값 : 클수록 - 계산 비용 감소, 성능 저하  [  0보다 커야 한다. 10 or 5를 권장  ]
        
        
       

        estimateReliability_(true),             // 신뢰도 추정 수행 여부 , 실패 감지하는데 도움 줌
        useGLPoseSampler_(true),                // 글로벌 로컬리제이션 기반의 포즈 샘플링 사용 여부 중요한 샘플 추가로 위치 추정의 성능 향상
        
        // 파티클 수과 샘플링, 리샘플링 파라미터
        particlesNum_(200),         // 파티클 수 결정 : 클수록 - 위치 정확성 증가, 계산 비용 증가
        useAugmentedMCL_(true),                 // 부가적인 파티클을 리샘플링 시 추가하는지 여부 판단
        alphaSlow_(0.001),                      // 부가적인 파티클을 추가하는 비율
        alphaFast_(0.99),                       // 부가적인 파티클을 추가하는 비율
        addRandomParticlesInResampling_(true),  // 리샘플링시 무작위 파티클을 추가하는지 여부
        randomParticlesRate_(0.1),              // 무작위 파티클 추가 하는 비율
        resampleThresholdESS_(0.5), // 유효한 파티클의 비율이 이 임계값보다 낮으면 리샘플링이 수행됩니다. 유효한 파티클의 비율은 알고리즘의 신뢰도를 나타내는 중요한 지표 ( 0 ~ 1 사이)

        // 측정 모델 가중치 파라미터
        zHit_(0.9),                             // 모델의 가중치 파라미터 :; 각 파라미터는 측정값이 해당 모델과 얼마나 일치하는지에 대한 정보  
        zShort_(0.2),                           // ( z_hit, zRand_, zMax_ 파라미터의 합은 1 이어야함.)
        zMax_(0.05),
        zRand_(0.05),
        varHit_(0.04),                          // 측정값이 일치할 가능성에 대한 가중치
        lambdaShort_(1.0),                      // 측정값이 장애물과 가까운 경우에 대한 가중치 
        lambdaUnknown_(0.01),                   // 측정값이 알 수 없는 영역에 대한 가중치

        // 리샘플링 파라미터
        
        laserFrame_("laser"),
        baseLinkFrame_("base_link"),
        mapFrame_("map"),
        odomFrame_("odom"),
        broadcastTF_(true),
        useOdomTF_(true),

        // 초기 설정 파라미터
        initialPoseX_(0.0),
        initialPoseY_(0.0),
        initialPoseYaw_(0.0),
        initialNoiseX_(0.02),
        initialNoiseY_(0.02),
        initialNoiseYaw_(0.01),

        glParticlesNum_(0),
        mapResolution_(0.0),
        mapWidth_(0), 
        mapHeight_(0),

        pRand_(0.0),
        measurementModelRandom_(0.0),
        measurementModelInvalidScan_(0.0),

        reliability_(0),
        fuseGLPoseSamplerOnlyUnreliable_(false),

        amclRandomParticlesRate_(0.0),
        effectiveSampleSize_(0.0),
        maxLikelihoodParticleIdx_(0),

        totalLikelihood_(0.0),
        averageLikelihood_(0.0),
        maxLikelihood_(0.0),

        randomParticlesNoise_({0.05, 0.05, 0.1}),
        odomNoiseDDM_({1.0, 0.5, 0.5, 1.5}),
        odomNoiseODM_({1.0, 0.5, 0.5, 0.5, 1.0, 0.5, 0.5, 0.5, 1.0}),
        deltaXSum_(0.0),
        deltaYSum_(0.0),
        deltaDistSum_(0.0),
        deltaYawSum_(0.0),
        deltaTimeSum_(0.0),
        resampleThresholds_({0.2, 0.2, 0.2, 0.02, -99999.0}),
        pKnownPrior_(0.5),  // 0 ~ 1 사이여야함.
        pUnknownPrior_(0.5),
        omegaSlow_(0.0),
        omegaFast_(0.0),
        relTransDDM_({0.0, 0.0}),
        relTransODM_({0.0, 0.0, 0.0}),
        classifierType_(0),
        maeClassifierDir_("/home/ebot/catkin_ws/src/ts800/application/ros/ts800_ros/source/application/location/classifiers/"),
        gmmPositionalVariance_(0.01),
        gmmAngularVariance_(0.01),
        predDistUnifRate_(0.05),
        localizationHz_(10.0),
        transformTolerance_(1.0),
        glSampledPoseTimeTH_(1.0),
        prevTime(0.0),
        scanMightInvalid_(false),
        isInitialized_(true),
        canUpdateGLSampledPoses_(true),
        canUseGLSampledPoses_(false),
        isGLSampledPosesUpdated_(false),
        tfListener_(),
        unknownScanProbThreshold_(0.9),
        rad2deg_(180.0 / M_PI),
        finalMAE(-1)
    {
        pUnknownPrior_ = 1.0 - pKnownPrior_;
        
        // degree to radian
        initialPoseYaw_ *= M_PI / 180.0;

        // set initial pose
        mclPose_.setPose(initialPoseX_, initialPoseY_, initialPoseYaw_);
        resetParticlesDistribution();
        systemPose_.setPose(0.0, 0.0, 0.0);
        systemPose_.setPose(0.0, 0.0, 0.0);
        deltaX_ = deltaY_ = deltaDist_ = deltaYaw_ = 0.0;
 
        //( -2.88 rad = -165.0 deg )
        //lidar arg= base_link => m "x,y,z,yaw,roll,pitch"
        //ydlidar - args="0.08 0.0 0.0 -2.20 0.0 0.0 /base_link /laser 40" />  
        //3i-lidar - args="-0.175 0.0 0.0 -2.88 0.0 0.0 /base_link /laser 40" />
#if 1 //3i-lidar
        baseLink2Laser_.setX(-0.175 );
        baseLink2Laser_.setY(0.0);
        baseLink2Laser_.setYaw(-2.88);
#else //ydliar 
        baseLink2Laser_.setX(0.085);
        baseLink2Laser_.setY(0.0);
        baseLink2Laser_.setYaw(0.0);
#endif
        // 신뢰도를 추정하기 위함
        if (estimateReliability_) 
        {
            // reliabilities_ 변수 초기화
            resetReliabilities();

            // 분류 유형 0인경우 평균절대오차(MAE)를 설정
            if (classifierType_ == 0) 
            {
                maeClassifier_.setClassifierDir(maeClassifierDir_);
                maeClassifier_.readClassifierParams();
                maes_.resize(particlesNum_);
            } 
            else // 0이 아닌경우 에러메시지 출력 & 종료 
            {
                ROS_ERROR("Incorrect classifier type was selected."
                    " The expected type is 0, but %d was selected.", classifierType_);
                exit(1);
            }
        }

        // 측정 모델계산에 필요한 변수 설정
        // normConstHit_ = 측정의 정규화 상수
        // varHit_ : 측정의 분산
        // denomHit_ : 측정의 분모
        normConstHit_ = 1.0 / sqrt(2.0 * varHit_ * M_PI);
        denomHit_ = 1.0 / (2.0 * varHit_);
       

        isInitialized_ = true;
    }

/**
 * @brief 스캔 데이터가 업데이트 가능한지 여부를 설정하는 함수
 * 
 * @param canUpdateScan 스캔 데이터 업데이트 가능 여부
 */
void CRmcl::setCanUpdateScan(bool canUpdateScan) 
{
    int invalidScanNum = 0;

    // 범위를 벗어난 스캔 데이터 비율 계산
    for (int i = 0; i < (int)scan_.ranges.size(); ++i) 
    {
        double r = scan_.ranges[i];
        if (r < scan_.range_min || scan_.range_max < r)
            invalidScanNum++;
    }
    double invalidScanRate = (double)invalidScanNum / (int)scan_.ranges.size();

    // 0.95보다 크거나 scan_.ranges.size() = 0이면 에러 메시지 출력
    if (invalidScanRate > 0.95 || scan_.ranges.size() == 0) 
    {
        scanMightInvalid_ = true;
        // ROS_ERROR("MCL scan might invalid.");
    }
    else 
    {
        scanMightInvalid_ = false;
    }
}


CRmcl::~CRmcl()
{

}

inline std::string CRmcl::getMapFrame(void) { return mapFrame_; }
inline sensor_msgs::LaserScan CRmcl::getScan(void) { return scan_; }
inline int CRmcl::getParticlesNum(void) { return particlesNum_; }
inline CRmclPose CRmcl::getParticlePose(int i) { return particles_[i].getPose(); }
inline double CRmcl::getParticleW(int i) { return particles_[i].getW(); }
inline CRmclPose CRmcl::getBaseLink2Laser(void) { return baseLink2Laser_; }
inline int CRmcl::getScanStep(void) { return scanStep_; }
inline int CRmcl::getMaxLikelihoodParticleIdx(void) { return maxLikelihoodParticleIdx_; }
inline double CRmcl::getNormConstHit(void) { return normConstHit_; }
inline double CRmcl::getDenomHit(void) { return denomHit_; }
inline double CRmcl::getZHit(void) { return zHit_; }
inline double CRmcl::getMeasurementModelRandom(void) { return measurementModelRandom_; }

// inline setting functions
inline void CRmcl::setParticleW(int i, double w) { particles_[i].setW(w); }
inline void CRmcl::setTotalLikelihood(double totalLikelihood) { totalLikelihood_ = totalLikelihood; }
inline void CRmcl::setAverageLikelihood(double averageLikelihood) { averageLikelihood_ = averageLikelihood; }
inline void CRmcl::setMaxLikelihood(double maxLikelihood) { maxLikelihood_ = maxLikelihood; }
inline void CRmcl::setMaxLikelihoodParticleIdx(int maxLikelihoodParticleIdx) { maxLikelihoodParticleIdx_ = maxLikelihoodParticleIdx; }

// inline other functions
void CRmcl::clearLikelihoodShiftedSteps(void) { likelihoodShiftedSteps_.clear(); }
void CRmcl::addLikelihoodShiftedSteps(bool flag) { likelihoodShiftedSteps_.push_back(flag); }

double CRmcl::getReliability(void){ return reliability_;}
double CRmcl::getFinalMAE(){return finalMAE;}

/**
 * @brief 파티클 필터 업데이트 함수
 * 
 */
void CRmcl::updateParticlesByMotionModel(void) 
{
    // deltaX, deltaY, deltaDist, deltaYaw 변수 초기화 및 이전 변수값 0으로 초기화
    double deltaX = deltaX_;
    double deltaY = deltaY_;
    double deltaDist = deltaDist_;
    double deltaYaw = deltaYaw_;
    deltaX_ = deltaY_ = deltaDist_ = deltaYaw_ = 0.0;
    
    // 현재 delta 값들의 절대값을 더해줌
    deltaXSum_ += fabs(deltaX);
    deltaYSum_ += fabs(deltaY);
    deltaDistSum_ += fabs(deltaDist);
    deltaYawSum_ += fabs(deltaYaw);

    // 현재 위치를 계산 
    // yaw : 로봇의 현재 방향각
    // t : 로봇의 이동방향과 yaw 각도를 이용하여 새로운 방향각 계산
    double yaw = mclPose_.getYaw();
    double t = yaw + deltaYaw / 2.0;
    double x = mclPose_.getX() + deltaDist * cos(t);
    double y = mclPose_.getY() + deltaDist * sin(t);
    yaw += deltaYaw;

    // 계산된 x, y, yaw 값들을 mclPose에 저장
    if(std::isnan(x)|| std::isnan(y)|| std::isnan(yaw))
    {

    }
    else
    {
        mclPose_.setPose(x, y, yaw);
    }

    // 노이즈 추가
    double dist2 = deltaDist * deltaDist;
    double yaw2 = deltaYaw * deltaYaw;
    double distRandVal = dist2 * odomNoiseDDM_[0] + yaw2 * odomNoiseDDM_[1];
    double yawRandVal = dist2 * odomNoiseDDM_[2] + yaw2 * odomNoiseDDM_[3];

    for (int i = 0; i < particlesNum_; ++i) 
    {
        double ddist = deltaDist + nrand(distRandVal);
        double dyaw = deltaYaw + nrand(yawRandVal);
        double yaw = particles_[i].getYaw();
        double t = yaw + dyaw / 2.0;
        double x = particles_[i].getX() + ddist * cos(t);
        double y = particles_[i].getY() + ddist * sin(t);
        yaw += dyaw;
        if(std::isnan(x)|| std::isnan(y) || std::isnan(yaw))
        {

        }
        else
        {
            particles_[i].setPose(x, y, yaw);
        }

        // 신뢰도를 이용시 : 이동 거리와 회전각도로 입자 신뢰도 업데이트
        if (estimateReliability_) 
        {
            double decayRate = 1.0 - (relTransDDM_[0] * ddist * ddist + relTransDDM_[1] * dyaw * dyaw);
            if (decayRate <= 0.0)
                decayRate = 10.0e-6;
            reliabilities_[i] *= decayRate;
        }
    } 
}

/**
 * @brief 로봇의 위치 추정에 사용되는 가능성(likelihood)을 계산하는 함수
 * 
 */
void CRmcl::calculateLikelihoodsByMeasurementModel(void) 
{
    // scan 데이터 유효성 체크
    if (scanMightInvalid_)
        return;

    // 로봇의 기준점에서 lidar의 위치까지의 거리 및 방향 
    double xo = baseLink2Laser_.getX();
    double yo = baseLink2Laser_.getY();
    double yawo = baseLink2Laser_.getYaw();
    std::vector<CRmclPose> sensorPoses(particlesNum_);
    
    double yaw = 0.0;
    double sensorX = 0.0;
    double sensorY = 0.0;
    double sensorYaw = 0.0;

    double range = 0.0;
    double rangeAngle = 0.0;
    double max = 0.0;
    double p = 0.0;
    double w =0.0;
    int maxIdx = 0;
    double sum = 0.0;
     
    // particle로 lidar의 측정값을 예측
    for (int i = 0; i < particlesNum_; ++i) 
    {
        yaw = particles_[i].getYaw();
        sensorX = xo * cos(yaw) - yo * sin(yaw) + particles_[i].getX();
        sensorY = xo * sin(yaw) + yo * cos(yaw) + particles_[i].getY();
        sensorYaw = yawo + yaw;

        CRmclPose sensorPose(sensorX, sensorY, sensorYaw);
        sensorPoses[i] = sensorPose;
        particles_[i].setW(0.0);
    }

    likelihoodShiftedSteps_.clear();

    // 예측된 값(sensorPoses)을 이용하여 lidar의 값과 로봇의 위치에 대한 가능성(likelihood)을 계산
    for (int i = 0; i < (int)scan_.ranges.size(); i += scanStep_) 
    {
        range = scan_.ranges[i];
        rangeAngle = (double)i * scan_.angle_increment + scan_.angle_min;
    
        for (int j = 0; j < particlesNum_; ++j) 
        {
            if (measurementModelType_ == 0)
            {
                p = calculateLikelihoodFieldModel(sensorPoses[j], range, rangeAngle);
            }
            w = particles_[j].getW();

            // 계산된 가능성을 로그로 누적 
            //  -> 로그 사용 이유 : 곱셈 연산을 누적 수행할 때 곱셈의 결과가 매우 작아지는 경우에도, 로그에서는 덧셈으로 해결할수있기 때문
            w += log(p);
           
            particles_[j].setW(w);
            if (j == 0) 
            {
                max = w;
            }
            else 
            {
                if (max < w)
                    max = w;
            }
        }

        // 값이 매우 작아진 경우 모든 입자의 가중치를 일정한 값(300)만큼 올림.
        // -> 계산이 불가능한 경우를 방지 ( 예외 처리 )
        if (max < -300.0) 
        {
            for (int j = 0; j < particlesNum_; ++j) 
            {
                w = particles_[j].getW() + 300.0;
                particles_[j].setW(w);
            }
            likelihoodShiftedSteps_.push_back(true);
        } 
        else 
        {
            likelihoodShiftedSteps_.push_back(false);
        }
    }  


    // 가중치를 확률치로 변환 => 가중치를 합하여 가능성(likelihood)을 계산
    for (int i = 0; i < particlesNum_; ++i) 
    {
        w = exp(particles_[i].getW());
        particles_[i].setW(w);
        sum += w;
        if (i == 0) 
        {
            max = w;
            maxIdx = i;
        } 
        else if (max < w) 
        {
            max = w;
            maxIdx = i;
        }
    }

    // totalLikelihood_ : 가중치 합
    // averageLikelihood_ : 가중치 평균값
    // maxLikelihood_ : 가장 큰 가능성(likelihood) 저장
    // maxLikelihoodParticleIdx_ : 가장 큰 가능성(likelihood)의 인덱스 저장
    totalLikelihood_ = sum;
    averageLikelihood_ = sum / (double)particlesNum_;
    maxLikelihood_ = max;
    maxLikelihoodParticleIdx_ = maxIdx;
}

/**
 * @brief 입자의 가능도를 계산.
 * 
 */
void CRmcl::calculateLikelihoodsByDecisionModel(void) 
{
    // scan 데이터 유효성 체크
    if (scanMightInvalid_)
        return;

    // 신뢰도 추정의 사용 여부 체크 
    if (!estimateReliability_)
        return;

    // 모델 유형이 2 이고, 분류 유형이 0 인 경우
    if (measurementModelType_ == 2 && classifierType_ == 0) 
    {
        // 최대 가능성을 가진 점의 pose값으로 알려지지 않은 스캔 데이터 측정
        CRmclPose mlPose = particles_[maxLikelihoodParticleIdx_].getPose();
        estimateUnknownScanWithClassConditionalMeasurementModel(mlPose);
    }

    double sum = 0.0;
    double max = 0.0;
    int maxIdx = 0;
    double mae =0.0;
    double w = 0.0;

    double measurementLikelihood = 0.0;
    std::vector<double> residualErrors;
    double decisionLikelihood = 0.0;     

    // 모든 particle에 대한 측정 가능도와 결정 가능도를 계산
    for (int i = 0; i < particlesNum_; ++i) 
    {
        CRmclPose particlePose = particles_[i].getPose();
        measurementLikelihood = particles_[i].getW();
        residualErrors = getResidualErrors<double>(particlePose);  

        // 분류 유형 == 0인 경우의 계산    
        if (classifierType_ == 0) 
        {
            mae = maeClassifier_.getMAE(residualErrors);
            decisionLikelihood = maeClassifier_.calculateDecisionModel(mae, &reliabilities_[i]);
            // 잔차 오차의 평균 절대 오차를 계산
            maes_[i] = mae;
        }


        w = measurementLikelihood * decisionLikelihood;
        particles_[i].setW(w);
        sum += w;
        if (i == 0) {
            max = w;
            maxIdx = 0;
        } else if (max < w) {
            max = w;
            maxIdx = i;
        }
    }

    // 입제에 대한 가능도, 총가능도, 평균 가능도, 최대 가능도 결정
    totalLikelihood_ = sum;
    averageLikelihood_ = sum / (double)particlesNum_;
    maxLikelihood_ = max;
    maxLikelihoodParticleIdx_ = maxIdx;
    reliability_ = reliabilities_[maxIdx];
}

/**
 * @brief GL(Global Localization)단계에서 가능도(likelihood)를 계산
 * 
 */
void CRmcl::calculateGLSampledPosesLikelihood(void) 
{
    // GL smapling 기능 사용 여부
    if (!useGLPoseSampler_)
        return;
    
    // scan 데이터 유효성 체크
    if (scanMightInvalid_)
        return;
    
    // reliability_가 높은 pose만 이용하기로 한 경우 : 신뢰도가 0.9보다 높은 경우 함수 종료
    // estimateReliability_ : reliability_ 값 추정 여부
    // fuseGLPoseSamplerOnlyUnreliable_ : reliability_가 낮은 pose만 사용 여부
    if (estimateReliability_ && fuseGLPoseSamplerOnlyUnreliable_) 
    {
        if (reliability_ >= 0.9)
            return;
    }

    // GL sampling 실행 : robot의 위치 추정이 실패한 경우 이 기능을 사용
    // - 여러개의 particle들을 무작위로 생성하여 적합한 particle를 선택하여 로봇의 위치를 추정하는 기술
    glParticlesNum_ = (int)glSampledPoses_.poses.size();
    double dt = fabs(mclPoseStamp_.toSec() - glSampledPosesStamp_.toSec());
    if (dt > glSampledPoseTimeTH_ || glParticlesNum_ == 0 || !isGLSampledPosesUpdated_)
        return;

    canUpdateGLSampledPoses_ = isGLSampledPosesUpdated_ = false;
    double xo = baseLink2Laser_.getX();
    double yo = baseLink2Laser_.getY();
    double yawo = baseLink2Laser_.getYaw();
    std::vector<CRmclPose> sensorPoses(glParticlesNum_);
    glParticles_.resize(glParticlesNum_);
    if (estimateReliability_) 
    {
        glSampledPosesReliabilities_.resize(glParticlesNum_);
        if (classifierType_ == 0)
            glSampledPosesMAEs_.resize(glParticlesNum_);
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double sensorX =  0.0;
    double sensorY = 0.0;
    double sensorYaw = 0.0;

    for (int i = 0; i < glParticlesNum_; ++i) 
    {
        tf::Quaternion q(glSampledPoses_.poses[i].orientation.x, 
            glSampledPoses_.poses[i].orientation.y, 
            glSampledPoses_.poses[i].orientation.z,
            glSampledPoses_.poses[i].orientation.w);
        
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        glParticles_[i].setPose(glSampledPoses_.poses[i].position.x, glSampledPoses_.poses[i].position.y, yaw);

        sensorX = xo * cos(yaw) - yo * sin(yaw) + glParticles_[i].getX();
        sensorY = xo * sin(yaw) + yo * cos(yaw) + glParticles_[i].getY();
        sensorYaw = yawo + yaw;
        CRmclPose sensorPose(sensorX, sensorY, sensorYaw);
        sensorPoses[i] = sensorPose;
        glParticles_[i].setW(0.0);
    }
    canUpdateGLSampledPoses_ = true;

    int idx = 0;
    double range = 0.0;
    double rangeAngle = 0.0;
    double max = 0.0;
    double p =0.0;
    double w =0.0;
    for (int i = 0; i < (int)scan_.ranges.size(); i += scanStep_) 
    {
        range = scan_.ranges[i];
        rangeAngle = (double)i * scan_.angle_increment + scan_.angle_min;
        for (int j = 0; j < glParticlesNum_; ++j) 
        {
           
            if (measurementModelType_ == 0)
                p = calculateLikelihoodFieldModel(sensorPoses[j], range, rangeAngle);
            w = glParticles_[j].getW();
            w += log(p);
            glParticles_[j].setW(w);
        }

        if (likelihoodShiftedSteps_[idx]) 
        {
            for (int j = 0; j < glParticlesNum_; ++j) 
            {
                w = glParticles_[j].getW() + 300.0;
                glParticles_[j].setW(w);
            }
        }
        idx++;
    }

    double normConst = 1.0 / sqrt(2.0 * M_PI * (gmmPositionalVariance_ + gmmPositionalVariance_ + gmmAngularVariance_));
    double angleResolution = 1.0 * M_PI / 180.0;
    double sum = totalLikelihood_;
    max = maxLikelihood_;
    double gmmRate = 1.0 - predDistUnifRate_;
    int maxIdx = -1;
    std::vector<double> residualErrors;
    double decisionLikelihood = 0.0;
    double mae = 0.0;

    double gmmVal = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    double dyaw = 0.0;
    double pGMM = 0.0;
    double predLikelihood = 0.0;
    for (int i = 0; i < glParticlesNum_; ++i) 
    {
        // measurement likelihood
        w = exp(glParticles_[i].getW());

        // decision likelihood
        if (estimateReliability_) 
        {
            residualErrors = getResidualErrors<double>(glParticles_[i].getPose());
            glSampledPosesReliabilities_[i] = 0.5;
            
            if (classifierType_ == 0) 
            {
                mae = maeClassifier_.getMAE(residualErrors);
                decisionLikelihood = maeClassifier_.calculateDecisionModel(mae, &glSampledPosesReliabilities_[i]);
                glSampledPosesMAEs_[i] = mae;
            }

            w *= decisionLikelihood;
        }

        // predictive distribution likelihood

        for (int j = 0; j < particlesNum_; ++j) 
        {
            dx = glParticles_[i].getX() - particles_[j].getX();
            dy = glParticles_[i].getY() - particles_[j].getY();
            dyaw = glParticles_[i].getYaw() - particles_[j].getYaw();
            while (dyaw < -M_PI)
                dyaw += 2.0 * M_PI;
            while (dyaw > M_PI)
                dyaw -= 2.0 * M_PI;
            gmmVal += normConst * exp(-((dx * dx) / (2.0 * gmmPositionalVariance_) + (dy * dy) / (2.0 * gmmPositionalVariance_) + (dyaw * dyaw) / (2.0 * gmmAngularVariance_)));
        }
        pGMM = (double)glParticlesNum_ * gmmVal * mapResolution_ * mapResolution_ * angleResolution / (double)particlesNum_;
        predLikelihood = gmmRate * pGMM + predDistUnifRate_ * 10e-9;
        w *= predLikelihood;
        if (w > 1.0)
            w = 1.0;

        glParticles_[i].setW(w);
        sum += w;
        if (max < w) 
        {
            max = w;
            maxIdx = i;
        }
    }

    if (std::isnan(sum)) 
    {
        for (int i = 0; i < glParticlesNum_; ++i)
            glParticles_[i].setW(0.0);
        canUseGLSampledPoses_ = false;
        return;
    }

    totalLikelihood_ = sum;
    averageLikelihood_ = sum / (double)(particlesNum_ + glParticlesNum_);

    if (maxIdx >= 0) 
    {
        maxLikelihood_ = max;
        maxLikelihoodParticleIdx_ = particlesNum_ + maxIdx;
        if (estimateReliability_)
            reliability_ = glSampledPosesReliabilities_[maxIdx];
    }

    // glParticles_ 와 sensorPoses를 이용하여 가능성 계산 => 로봇의 위치를 업데이트
    canUseGLSampledPoses_ = true;
}

/**
 * @brief 랜덤 파티클 비율을 계산하는 함수
 * 
 */
void CRmcl::calculateAMCLRandomParticlesRate(void)
{
    // 확장된 AMCL을 사용 여부 체크
    if (!useAugmentedMCL_)
        return;
    
    // scan 데이터의 유효성 체크
    if (scanMightInvalid_)
        return;

    // omegaSlow_, omegaFast_ 변수를 업데이트 
    // averageLikelihood_ : 파티클들이 현재 위치에서 센서 측정값과 일치하는 정도
    // omegaSlow_, omegaFast_, alphaSlow_, alphaFast_ : 업데이트 속도 결정 파라미터
    omegaSlow_ += alphaSlow_ * (averageLikelihood_ - omegaSlow_);
    omegaFast_ += alphaFast_ * (averageLikelihood_ - omegaFast_);
    amclRandomParticlesRate_ = 1.0 - omegaFast_ / omegaSlow_;
    if (amclRandomParticlesRate_ < 0.0)
        amclRandomParticlesRate_ = 0.0;
}

/**
 * @brief 유효 샘플 수를 계산하는 함수
 * 
 */
void CRmcl::calculateEffectiveSampleSize(void) 
{
    // scan 데이터 유효성 체크
    if (scanMightInvalid_)
        return;


    double sum = 0.0;

   
    if (!useGLPoseSampler_ || !canUseGLSampledPoses_) 
    {
        double wo = 1.0 / (double)particlesNum_;
        for (int i = 0; i < particlesNum_; ++i) 
        {
            double w = particles_[i].getW() / totalLikelihood_;
            particles_[i].setW(w);
            sum += w * w;
        }
    } 
    else  // GL(global localization)를 사용하는 경우 
    {
        double wo = 1.0 / (double)(particlesNum_ + glParticlesNum_);
        for (int i = 0; i < particlesNum_; ++i) 
        {
            double w = particles_[i].getW() / totalLikelihood_;
            particles_[i].setW(w);
            sum += w * w;
        }
        for (int i = 0; i < glParticlesNum_; ++i) 
        {
            double w = glParticles_[i].getW() / totalLikelihood_;
            glParticles_[i].setW(w);
            sum += w * w;
        }
    }
    // sum : 가중치를 제곱한 것들의 합
    effectiveSampleSize_ = 1.0 / sum;
}

/**
 * @brief 로봇의 위치를 추정하는 함수
 * 
 */
void CRmcl::estimatePose(void) 
{
    // scan 데이터 유효성 체크
    if (scanMightInvalid_)
        return;

    double tmpYaw = mclPose_.getYaw();
    double x = 0.0, y = 0.0, yaw = 0.0;
    double sum = 0.0;

    // 계산된 particle를 이용하여 로봇의 위치와 방향을 결정
    for (int i = 0; i < particlesNum_; ++i) 
    {
        double w = particles_[i].getW();
        x += particles_[i].getX() * w;
        y += particles_[i].getY() * w;
        double dyaw = tmpYaw - particles_[i].getYaw();
        while (dyaw < -M_PI)
            dyaw += 2.0 * M_PI;
        while (dyaw > M_PI)
            dyaw -= 2.0 * M_PI;
        yaw += dyaw * w;
        sum += w;
    }

    // GL(global localization)를 사용하는 경우 
    if (useGLPoseSampler_ && canUseGLSampledPoses_) 
    {
        double x2 = x, y2 = y, yaw2 = yaw;
        for (int i = 0; i < glParticlesNum_; ++i) 
        {
            double w = glParticles_[i].getW();
            x += glParticles_[i].getX() * w;
            y += glParticles_[i].getY() * w;
            double dyaw = tmpYaw - glParticles_[i].getYaw();
            while (dyaw < -M_PI)
                dyaw += 2.0 * M_PI;
            while (dyaw > M_PI)
                dyaw -= 2.0 * M_PI;
            yaw += dyaw * w;
            sum += w;
        }
        if (sum > 1.0)
            x = x2, y = y2, yaw = yaw2;
    }

    yaw = tmpYaw - yaw;
    mclPose_.setPose(x, y, yaw);
}

/**
 * @brief 파티클 필터의 샘플링을 다시 하는 함수
 * 
 */
void CRmcl::resampleParticles(void) 
{
    // scan 데이터의 유형성 체크
    if (scanMightInvalid_)
        return;

    double threshold = (double)particlesNum_ * resampleThresholdESS_;

    // GL(global localization)를 사용하는 경우 
    if (useGLPoseSampler_ && canUseGLSampledPoses_)
        threshold = (double)(particlesNum_ + glParticlesNum_) * resampleThresholdESS_;

    //effectiveSampleSize_ 이 threshold보다 큰 경우 함수 종료
    if (effectiveSampleSize_ > threshold)
        return;

    // deltaXSum_, deltaYSum_, deltaDistSum_, deltaYawSum_, deltaTimeSum_ 이 모두 resampleThresholds_에 저장된 값보다 작으면 종료
    // = 기준보다 이동량이 적다면 리샘플링을 할 필요가 없다.
    if (deltaXSum_ < resampleThresholds_[0] && deltaYSum_ < resampleThresholds_[1] &&
        deltaDistSum_ < resampleThresholds_[2] && deltaYawSum_ < resampleThresholds_[3] &&
        deltaTimeSum_ < resampleThresholds_[4])
        return;

    deltaXSum_ = deltaYSum_ = deltaDistSum_ = deltaYSum_ = deltaTimeSum_ = 0.0;
    std::vector<double> wBuffer;


    if (useGLPoseSampler_ && canUseGLSampledPoses_) 
    {
        wBuffer.resize(particlesNum_ + glParticlesNum_);
        wBuffer[0] = particles_[0].getW();
        for (int i = 1; i < particlesNum_; ++i)
            wBuffer[i] = particles_[i].getW() + wBuffer[i - 1];
        for (int i = 0; i < glParticlesNum_; ++i)
            wBuffer[particlesNum_ + i] = glParticles_[i].getW() + wBuffer[particlesNum_ + i - 1];
    } 
    else 
    {
        wBuffer.resize(particlesNum_);
        wBuffer[0] = particles_[0].getW();
        for (int i = 1; i < particlesNum_; ++i)
            wBuffer[i] = particles_[i].getW() + wBuffer[i - 1];
    }

    std::vector<Particle> tmpParticles = particles_;
    std::vector<double>  tmpReliabilities;
    if (estimateReliability_)
        tmpReliabilities = reliabilities_;
    double wo = 1.0 / (double)particlesNum_;

    if (!addRandomParticlesInResampling_ && !useAugmentedMCL_) 
    {
        // normal resampling
        for (int i = 0; i < particlesNum_; ++i) 
        {
            double darts = (double)rand() / ((double)RAND_MAX + 1.0);
            bool isResampled = false;
            for (int j = 0; j < particlesNum_; ++j) 
            {
                if (darts < wBuffer[j]) 
                {
                    particles_[i].setPose(tmpParticles[j].getPose());
                    if (estimateReliability_)
                        reliabilities_[i] = tmpReliabilities[j];
                    particles_[i].setW(wo);
                    isResampled = true;
                    break;
                }
            }
            if (!isResampled && useGLPoseSampler_ && canUseGLSampledPoses_) 
            {
                for (int j = 0; j < glParticlesNum_; ++j) 
                {
                    if (darts < wBuffer[particlesNum_ + j]) 
                    {
                        particles_[i].setPose(glParticles_[j].getPose());
                        if (estimateReliability_)
                            reliabilities_[i] = glSampledPosesReliabilities_[j];
                        particles_[i].setW(wo);
                        break;
                    }
                }
            }
        }
    } 
    else 
    {
        // resampling and add random particles
        double randomParticlesRate = randomParticlesRate_;
        if (useAugmentedMCL_ && amclRandomParticlesRate_ > 0.0) 
        {
            omegaSlow_ = omegaFast_ = 0.0;
            randomParticlesRate = amclRandomParticlesRate_;
        } 
        else if (!addRandomParticlesInResampling_) 
        {
            randomParticlesRate = 0.0;
        }
        int resampledParticlesNum = (int)((1.0 - randomParticlesRate) * (double)particlesNum_);
        int randomParticlesNum = particlesNum_ - resampledParticlesNum;
        for (int i = 0; i < resampledParticlesNum; ++i) 
        {
            double darts = (double)rand() / ((double)RAND_MAX + 1.0);
            bool isResampled = false;
            for (int j = 0; j < particlesNum_; ++j) 
            {
                if (darts < wBuffer[j]) 
                {
                    particles_[i].setPose(tmpParticles[j].getPose());
                    if (estimateReliability_)
                        reliabilities_[i] = tmpReliabilities[j];
                    particles_[i].setW(wo);
                    isResampled = true;
                    break;
                }
            }
            if (!isResampled && useGLPoseSampler_ &&  canUseGLSampledPoses_) 
            {
                for (int j = 0; j < glParticlesNum_; ++j) 
                {
                    if (darts < wBuffer[particlesNum_ + j]) 
                    {
                        particles_[i].setPose(glParticles_[j].getPose());
                        if (estimateReliability_)
                            reliabilities_[i] = glSampledPosesReliabilities_[j];
                        particles_[i].setW(wo);
                        break;
                    }
                }
            }
        }

        double xo = mclPose_.getX();
        double yo = mclPose_.getY();
        double yawo = mclPose_.getYaw();
        for (int i = resampledParticlesNum; i < resampledParticlesNum + randomParticlesNum; ++i)
        {
            double x = xo + nrand(randomParticlesNoise_[0]);
            double y = yo + nrand(randomParticlesNoise_[1]);
            double yaw = yawo + nrand(randomParticlesNoise_[2]);
            particles_[i].setPose(x, y, yaw);
            particles_[i].setW(wo);
            if (estimateReliability_)
                reliabilities_[i] = reliability_;
        }
    }
    canUseGLSampledPoses_ = false;
}

/**
 * @brief  pose를 기반으로 라이다 scan 데이터를 이용해 residual errors를 계산하는 함수
 * 
 * @tparam T 
 * @param pose 
 * @return std::vector<T> 
 */
template <typename T> std::vector<T> CRmcl::getResidualErrors(CRmclPose pose) 
{
    // residualErrors 초기화
    double yaw = pose.getYaw();
    double sensorX = baseLink2Laser_.getX() * cos(yaw) - baseLink2Laser_.getY() * sin(yaw) + pose.getX();
    double sensorY = baseLink2Laser_.getX() * sin(yaw) + baseLink2Laser_.getY() * cos(yaw) + pose.getY();
    double sensorYaw = baseLink2Laser_.getYaw() + yaw;
    int size = (int)scan_.ranges.size();
    std::vector<T> residualErrors(size);

    for (int i = 0; i < size; ++i) 
    {
        double r = scan_.ranges[i];
        // 거리값이 범위 밖일 때
        if (r <= scan_.range_min || scan_.range_max <= r) 
        {
            residualErrors[i] = -1.0;
            continue;
        }

        // angle 값을 이용해 x,y 좌표계산 => u,v 좌표 계산
        double t = (double)i * scan_.angle_increment + scan_.angle_min + sensorYaw;
        double x = r * cos(t) + sensorX;
        double y = r * sin(t) + sensorY;
        int u, v;
        xy2uv(x, y, &u, &v);

        // (u,v) 맵 안에 있을 경우
        if (onMap(u, v)) 
        {
            T dist = (T)distMap_.at<float>(v, u);
            residualErrors[i] = dist;
        }
        else 
        {
            residualErrors[i] = -1.0;
        }
    }
    return residualErrors;
}

/**
 * @brief 결과 출력 함수
 * 
 */
void CRmcl::printResult(void) 
{
    eblog(LOG_LV, "RMCL: x = " << mclPose_.getX() << " [m], y = " << mclPose_.getY() << " [m], yaw = " << mclPose_.getYaw() * rad2deg_ << " [deg]" );
    eblog(LOG_LV, "Odom: x = " << systemPose_.getX() << " [m], y = " << systemPose_.getY() << " [m], yaw = " << systemPose_.getYaw() * rad2deg_ << " [deg]" );
    eblog(LOG_LV, "total likelihood = " << totalLikelihood_);
    eblog(LOG_LV, "particlesNum_ = " << particlesNum_);
    eblog(LOG_LV, "average likelihood = " << averageLikelihood_ );
    eblog(LOG_LV, "max likelihood = " << maxLikelihood_ );
    eblog(LOG_LV, "effective sample size = " << effectiveSampleSize_ );

    
    if (useAugmentedMCL_)
        eblog(LOG_LV, "amcl random particles rate = " << amclRandomParticlesRate_);

    if (estimateReliability_ && classifierType_ == 0)
    {
        finalMAE= maeClassifier_.getMAE(getResidualErrors<double>(particles_[maxLikelihoodParticleIdx_].getPose()));

        eblog(LOG_LV, "reliability = " << reliability_ << " ( mae = " << finalMAE << " ) ");
    }
}

/**
 * @brief likelihood 맵을 작성하고 시각화하는 함수
 * 
 */
void CRmcl::plotLikelihoodMap(void) 
{
    double yaw = mclPose_.getYaw();
    double sensorX = baseLink2Laser_.getX() * cos(yaw) - baseLink2Laser_.getY() * sin(yaw) + mclPose_.getX();
    double sensorY = baseLink2Laser_.getX() * sin(yaw) + baseLink2Laser_.getY() * cos(yaw) + mclPose_.getY();
    double sensorYaw = baseLink2Laser_.getYaw() + yaw;
    double range = 0.5;

    std::vector<CRmclPose> sensorPoses;
    for (double x = -range - mapResolution_; x <= range + mapResolution_; x += mapResolution_) 
    {
        for (double y = -range - mapResolution_; y <= range + mapResolution_; y += mapResolution_) 
        {
            CRmclPose sensorPose(sensorX + x, sensorY + y, sensorYaw);
            sensorPoses.push_back(sensorPose);
        }
    }

    std::vector<double> likelihoods((int)sensorPoses.size(), 0.0);
    double rangeAngle =0.0;
    double max =0.0;
    double p = 0.0;
    double w = 0.0;
    for (int i = 0; i < (int)scan_.ranges.size(); i += scanStep_) 
    {
        range = scan_.ranges[i];
        rangeAngle = (double)i * scan_.angle_increment + scan_.angle_min;
        
        for (int j = 0; j < (int)sensorPoses.size(); ++j) 
        {
            if (measurementModelType_ == 0)
                p = calculateLikelihoodFieldModel(sensorPoses[j], range, rangeAngle);
            else if (measurementModelType_ == 1)
                p = calculateBeamModel(sensorPoses[j], range, rangeAngle);
            else
                p = calculateClassConditionalMeasurementModel(sensorPoses[j], range, rangeAngle);
            w = likelihoods[j];
            w += log(p);
            likelihoods[j] = w;
            if (j == 0) 
            {
                max = w;
            } 
            else 
            {
                if (max < w)
                    max = w;
            }
        }
        if (max < -300.0) 
        {
            for (int j = 0; j < (int)sensorPoses.size(); ++j)
                likelihoods[j] += 300.0;
        }
    }

    double sum = 0.0;
    int maxIdx = 0.0;
    double reliability = 0.5;
    std::vector<double> residualErrors;
    double mae =0.0;
    for (int i = 0; i < (int)sensorPoses.size(); ++i) 
    {
        w = exp(likelihoods[i]);
        if (estimateReliability_) 
        {
            
            
            if (classifierType_ == 0) 
            {
                residualErrors = getResidualErrors<double>(sensorPoses[i]);
                mae = maeClassifier_.getMAE(residualErrors);
                w *= maeClassifier_.calculateDecisionModel(mae, &reliability);
            }

        }
        likelihoods[i]  = w;
        sum += w;
    }

    FILE *fp;
    fp = fopen("/tmp/als_ros_likelihood_map.txt", "w");
    int cnt = 0;
    for (double x = -range - mapResolution_; x <= range + mapResolution_; x += mapResolution_) 
    {
        for (double y = -range - mapResolution_; y <= range + mapResolution_; y += mapResolution_) 
        {
            fprintf(fp, "%lf %lf %lf\n", x, y, likelihoods[cnt] / sum);
            cnt++;
        }
        fprintf(fp, "\n");
    }
    fclose(fp);

    fp = fopen("/tmp/als_ros_scan_points.txt", "w");
    double r = 0.0;
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
    for (int i = 0; i < (int)scan_.ranges.size(); ++i) 
    {
        r = scan_.ranges[i];
        if (r < scan_.range_min || scan_.range_max < r)
            continue;
        t = (double)i * scan_.angle_increment + scan_.angle_min + sensorYaw;
        x = r * cos(t) + sensorX;
        y = r * sin(t) + sensorY;
        fprintf(fp, "%lf %lf\n", x, y);
    }
    fclose(fp);

    static FILE *gp;
    if (gp == NULL) 
    {
        gp = popen("gnuplot -persist", "w");
        fprintf(gp, "set colors classic\n");
        fprintf(gp, "set grid\n");
        fprintf(gp, "set size ratio 1 1\n");
        fprintf(gp, "set xlabel \"%s\"\n", "{/Symbol D}x [m]");
        fprintf(gp, "set ylabel \"%s\"\n", "{/Symbol D}y [m]");
        fprintf(gp, "set tics font \"Arial, 14\"\n");
        fprintf(gp, "set xlabel font \"Arial, 14\"\n");
        fprintf(gp, "set ylabel font \"Arial, 14\"\n");
        fprintf(gp, "set cblabel font \"Arial, 14\"\n");
        fprintf(gp, "set xrange [ %lf : %lf ]\n", -range, range);
        fprintf(gp, "set yrange [ %lf : %lf ]\n", -range, range);
        fprintf(gp, "set pm3d map interpolate 2, 2\n");
        fprintf(gp, "unset key\n");
        fprintf(gp, "unset cbtics\n");
    }
    fprintf(gp, "splot \"/tmp/als_ros_likelihood_map.txt\" with pm3d\n");
    fflush(gp);
}

/**
 * @brief 정규 분포를 따르는 난수를 생성
 * 
 * @param n 
 * @return double 
 */
inline double CRmcl::nrand(double n) 
{ 
    return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX)); 
}

/**
 * @brief 지도 범위 내인지 확인하는 함수
 * 
 * @param u 
 * @param v 
 * @return true 
 * @return false 
 */
inline bool CRmcl::onMap(int u, int v) 
{
    if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_)
        return true;
    else
        return false;
}

/**
 * @brief  x,y 좌표를 가지고 u,v좌표로 변환
 * 
 * @param x 
 * @param y 
 * @param u 
 * @param v 
 */
inline void CRmcl::xy2uv(double x, double y, int *u, int *v) 
{
    double dx = x - mapOrigin_.getX();
    double dy = y - mapOrigin_.getY();
    double yaw = -mapOrigin_.getYaw();
    double xx = dx * cos(yaw) - dy * sin(yaw);
    double yy = dx * sin(yaw) + dy * cos(yaw);
    *u = (int)(xx / mapResolution_);
    *v = (int)(yy / mapResolution_);
}

/**
 * @brief Laser 스캔 업데이트 
 * 
 * @param lidarData 
 */
void CRmcl::updateLaserData(sensor_msgs::LaserScan lidarData) 
{   
    scan_ = lidarData;
    // 무작위 노이즈에 대한 가중치 계산
    pRand_ = 1.0 / (scan_.range_max / mapResolution_);
    measurementModelRandom_ = zRand_ * pRand_; // 무작위 노이즈에 대한 가중치 
    measurementModelInvalidScan_ = zMax_ + zRand_ * pRand_; // 레이저 스캔 데이터의 유효성에 대한 가중치
}

/**
 * @brief 로봇의 위치와 방향 정보 업데이트
 * 
 * @param robotPose 
 */
void CRmcl::updateOdomData(tPose robotPose) 
{
    double currTime = get_system_time();

    // prevTime 초기화
    if (isInitialized_) 
    {
        prevTime = currTime;
        isInitialized_ = false;
        return;
    }

    // 이전 업데이트 시간과 현재 시간 사이에 경과 시간을 계산
    double deltaTime = get_system_time(prevTime);
    if (deltaTime == 0.0)
        return;

    // 로봇의 x, y, 및 이동거리, 각도 변화량을 누적
    deltaX_ += robotPose.x - systemPose_.getX();;
    deltaY_ += robotPose.y - systemPose_.getY();;
    deltaDist_ += robotPose.x - systemPose_.getX();;
    deltaYaw_ += DEG2RAD(robotPose.angle);

    // deltaYaw_의 정상 범위 ( -pi ~ pi) 체크 
    while (deltaYaw_ < -M_PI)
        deltaYaw_ += 2.0 * M_PI;
    while (deltaYaw_ > M_PI)
        deltaYaw_ -= 2.0 * M_PI;
    deltaTimeSum_ += deltaTime;

    if(std::isnan(robotPose.x)|| std::isnan( robotPose.y)|| std::isnan(robotPose.angle))
    {
       eblog(LOG_LV, "||____________ robotPose is nan __________________||" );
    }
    else
    {
        systemPose_.setPose(robotPose.x, robotPose.y, DEG2RAD(robotPose.angle));
    }
    prevTime = currTime;
}

/**
 * @brief 맵 업데이트 ( gridmap -> uchar Mat -> float Mat )
 * 
 * @param map 
 * @param info 
 */
void CRmcl::updateMapData(u8 *map, tGridmapInfo info) 
{

    mapWidth_ = info.width;
    mapHeight_ = info.height;
    mapResolution_ = info.resolution;
    
    // gridmap -> uchar 맵 변환
    cv::Mat binMap(mapHeight_, mapWidth_, CV_8UC1);
    for (int v = 0; v < mapHeight_; v++) 
    {
        for (int u = 0; u < mapWidth_; u++) 
        {
            int node = v * mapWidth_ + u;
            int val = (unsigned char)map[node];
            //if (90 <= val && val <= 100) // 70 <= var <= 100 일때 장애물로 지도 업데이트
            if (val == GRAY_LV_KNOWN_WALL) //기존에는 dist  값이 100 일 때만  장애물로 지도 업데이트
                binMap.at<uchar>(v, u) = 0;
            else
                binMap.at<uchar>(v, u) = 1;
        }
    }
    cv::Mat distMap(mapHeight_, mapWidth_, CV_32FC1);

    // (이진화된 binMap을 distMap으로 변환)
    cv::distanceTransform(binMap, distMap, cv::DIST_L2, 5);
    
    // uchar Mat -> float Mat 변환 
    for (int v = 0; v < mapHeight_; v++) 
    {
        for (int u = 0; u < mapWidth_; u++) 
        {
            float d = distMap.at<float>(v, u) * (float)mapResolution_;
            distMap.at<float>(v, u) = d;
        }
    }

    distMap_ = distMap;
    tf::Quaternion q(0,0,0,1);
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    mapOrigin_.setX(info.origin_x);
    mapOrigin_.setY(info.origin_y);
    mapOrigin_.setYaw(yaw);

}

void CRmcl::setInitPose(tPose initPose) 
{
    mclPose_.setPose(initPose.x, initPose.y,initPose.angle);
    resetParticlesDistribution();
    if (estimateReliability_)
    {
        resetReliabilities();
    }
    isInitialized_ = true;
}

/**
 * @brief 파티클의 분포를 재설정하는 함수
 * 
 */
void CRmcl::resetParticlesDistribution(void) 
{
    particles_.resize(particlesNum_);
    double xo = mclPose_.getX();
    double yo = mclPose_.getY();
    double yawo = mclPose_.getYaw();
    double wo = 1.0 / (double)particlesNum_;
    for (int i = 0; i < particlesNum_; ++i) 
    {
        double x = xo + nrand(initialNoiseX_);
        double y = yo + nrand(initialNoiseY_);
        double yaw = yawo + nrand(initialNoiseYaw_);
        particles_[i].setPose(x, y, yaw);
        particles_[i].setW(wo);
    }
}

/**
 * @brief 로봇의 추정된 위치 반환 함수
 * 
 * @return tPose 
 */
tPose CRmcl::getExtimatedPose()
{
    tPose tmp = tPose();
    tmp.x = mclPose_.getX();
    tmp.y = mclPose_.getY();
    tmp.angle = mclPose_.getYaw();
    return tmp;
}

/**
 * @brief 파티클을 반환한다.
 * 
 * @return geometry_msgs::PoseArray 
 */
geometry_msgs::PoseArray CRmcl::getParticles()
{
        // particles
    geometry_msgs::PoseArray particlesPoses;
    particlesPoses.header.frame_id = mapFrame_;
    particlesPoses.header.stamp = mclPoseStamp_;
    particlesPoses.poses.resize(particlesNum_);
    for (int i = 0; i < particlesNum_; ++i) 
    {
        geometry_msgs::Pose pose;
        pose.position.x = particles_[i].getX();
        pose.position.y = particles_[i].getY();
        pose.orientation = tf::createQuaternionMsgFromYaw(particles_[i].getYaw());
        particlesPoses.poses[i] = pose;
    }
    
    return particlesPoses;

}

void CRmcl::resetReliabilities(void) 
{
    reliabilities_.resize(particlesNum_, 0.5);
}


/**
 * @brief 가능도(likelihood) 계산 ( fiedl 모델 )
 * 
 * @param pose 
 * @param range 
 * @param rangeAngle 
 * @return double 
 */
double CRmcl::calculateLikelihoodFieldModel(CRmclPose pose, double range, double rangeAngle) 
{
    // scan의 거리값이 범위 내인지 확인
    if (range <= scan_.range_min || scan_.range_max <= range)
        return measurementModelInvalidScan_;

    double t = pose.getYaw() + rangeAngle;
    double x = range * cos(t) + pose.getX();
    double y = range * sin(t) + pose.getY();
    int u, v;
    xy2uv(x, y, &u, &v);
    double p;
    if (onMap(u, v)) 
    {
        double dist = (double)distMap_.at<float>(v, u);
        double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
        p = zHit_ * pHit + measurementModelRandom_;
    } 
    else 
    {
        p = measurementModelRandom_;
    }
    if (p > 1.0)
    {
        p = 1.0;
    }
   
    return p;
}

double CRmcl::calculateBeamModel(CRmclPose pose, double range, double rangeAngle) 
{
    if (range <= scan_.range_min || scan_.range_max <= range)
        return measurementModelInvalidScan_;

    double t = pose.getYaw() + rangeAngle;
    double x = pose.getX();
    double y = pose.getY();
    double dx = mapResolution_ * cos(t);
    double dy = mapResolution_ * sin(t);
    int u, v;
    double expectedRange = -1.0;
    double hitThreshold = 0.5 * mapResolution_;
    for (double r = 0.0; r < scan_.range_max; r += mapResolution_) 
    {
        xy2uv(x, y, &u, &v);
        if (onMap(u, v)) 
        {
            double dist = (double)distMap_.at<float>(v, u);
            if (dist < hitThreshold) 
            {
                expectedRange = r;
                break;
            }
        } 
        else 
        {
            break;
        }
        x += dx;
        y += dy;
    }

    double p;
    if (range <= expectedRange) 
    {
        double error = expectedRange - range;
        double pHit = normConstHit_ * exp(-(error * error) * denomHit_) * mapResolution_;
        double pShort = lambdaShort_ * exp(-lambdaShort_ * range) / (1.0 - exp(-lambdaShort_ * scan_.range_max)) * mapResolution_;
        p = zHit_ * pHit + zShort_ * pShort + measurementModelRandom_;
    } 
    else 
    {
        p = measurementModelRandom_;
    }
    if (p > 1.0)
        p = 1.0;
    return p;
}

double CRmcl::calculateClassConditionalMeasurementModel(CRmclPose pose, double range, double rangeAngle) 
{
    if (range <= scan_.range_min || scan_.range_max <= range)
        return measurementModelInvalidScan_;

    double t = pose.getYaw() + rangeAngle;
    double x = range * cos(t) + pose.getX();
    double y = range * sin(t) + pose.getY();
    double pUnknown = lambdaUnknown_ * exp(-lambdaUnknown_ * range) / (1.0 - exp(-lambdaUnknown_ * scan_.range_max)) * mapResolution_ * pUnknownPrior_;
    int u, v;
    xy2uv(x, y, &u, &v);
    double p = pUnknown;
    if (onMap(u, v)) 
    {
        double dist = (double)distMap_.at<float>(v, u);
        double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
        p += (zHit_ * pHit + measurementModelRandom_) * pKnownPrior_;
    } 
    else 
    {
        p += measurementModelRandom_ * pKnownPrior_;
    }
    if (p > 1.0)
        p = 1.0;
    return p;
}

/**
 * @brief ClassConditionalMeasurementModel에서 알려지지 않은 스캔을 추정
 * 
 * @param pose 
 */
void CRmcl::estimateUnknownScanWithClassConditionalMeasurementModel(CRmclPose pose) 
{
    // 라이다 측정값의 추정 확률에 기반하여  unknownScan_를 업데이트
    unknownScan_ = scan_;
    double yaw = pose.getYaw();
    double sensorX = baseLink2Laser_.getX() * cos(yaw) - baseLink2Laser_.getY() * sin(yaw) + pose.getX();
    double sensorY = baseLink2Laser_.getX() * sin(yaw) + baseLink2Laser_.getY() * cos(yaw) + pose.getY();
    double sensorYaw = baseLink2Laser_.getYaw() + yaw;
    
    // 레이저 측정값에 대해 측정값을 아는지 여부 결정 및 확률 계산
    for (int i = 0; i < (int)unknownScan_.ranges.size(); ++i) 
    {
        double r = unknownScan_.ranges[i];
        if (r <= unknownScan_.range_min || unknownScan_.range_max <= r) 
        {
            unknownScan_.ranges[i] = 0.0;
            continue;
        }
        double t = sensorYaw + (double)i * unknownScan_.angle_increment + unknownScan_.angle_min;
        double x = r * cos(t) + sensorX;
        double y = r * sin(t) + sensorY;
        int u, v;
        xy2uv(x, y, &u, &v);
        double pKnown;
        double pUnknown = lambdaUnknown_ * exp(-lambdaUnknown_ * r) / (1.0 - exp(-lambdaUnknown_ * unknownScan_.range_max)) * mapResolution_ * pUnknownPrior_;
        if (onMap(u, v)) 
        {
            double dist = (double)distMap_.at<float>(v, u);
            double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
            pKnown = (zHit_ * pHit + measurementModelRandom_) * pKnownPrior_;
        } 
        else 
        {
            pKnown = measurementModelRandom_ * pKnownPrior_;
        }

        // 두 확률을 합산하여 총 합으로 나눈 후 확률 계산
        double sum = pKnown + pUnknown;
        pUnknown /= sum;

        // unknownScanProbThreshold_ 보다 작으면 레이저 측정값을 0으로 하여 알려지지 않은 측정값으로 처리
        if (pUnknown < unknownScanProbThreshold_)
            unknownScan_.ranges[i] = 0.0;
    }
}