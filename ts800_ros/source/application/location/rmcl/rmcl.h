#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include "rmclPose.h"
#include "particle.h"
#include "maeClassifier.h"

#include "coreData/serviceData.h"

class CRmcl 
{
public:

CRmcl();
~CRmcl();

    int particlesNum_;
private:
    // tf frames
    std::string laserFrame_, baseLinkFrame_, mapFrame_, odomFrame_;
    bool broadcastTF_, useOdomTF_;

    // poses
    double initialPoseX_, initialPoseY_, initialPoseYaw_;
    CRmclPose mclPose_, baseLink2Laser_, systemPose_;
    ros::Time mclPoseStamp_, odomPoseStamp_, glSampledPosesStamp_;

    // particles
    std::vector<Particle> particles_;
    double initialNoiseX_, initialNoiseY_, initialNoiseYaw_;
    bool useAugmentedMCL_, addRandomParticlesInResampling_;
    double randomParticlesRate_;
    std::vector<double> randomParticlesNoise_;
    int glParticlesNum_;
    std::vector<Particle> glParticles_;
    geometry_msgs::PoseArray glSampledPoses_;
    bool canUpdateGLSampledPoses_, canUseGLSampledPoses_, isGLSampledPosesUpdated_;
    double glSampledPoseTimeTH_, gmmPositionalVariance_, gmmAngularVariance_;
    double predDistUnifRate_;

    // map
    cv::Mat distMap_;
    double mapResolution_;
    CRmclPose mapOrigin_;
    int mapWidth_, mapHeight_;

    // motion
    double deltaX_, deltaY_, deltaDist_, deltaYaw_;
    double deltaXSum_, deltaYSum_, deltaDistSum_, deltaYawSum_, deltaTimeSum_;
    std::vector<double> resampleThresholds_;
    std::vector<double> odomNoiseDDM_, odomNoiseODM_;

    // measurements
    sensor_msgs::LaserScan scan_, unknownScan_;
    std::vector<bool> likelihoodShiftedSteps_;

    // measurement model
    // 0: likelihood field model, 1: beam model, 2: class conditional measurement model
    int measurementModelType_;
    double zHit_, zShort_, zMax_, zRand_;
    double varHit_, lambdaShort_, lambdaUnknown_;
    double normConstHit_, denomHit_, pRand_;
    double measurementModelRandom_, measurementModelInvalidScan_;
    double pKnownPrior_, pUnknownPrior_, unknownScanProbThreshold_;
    double alphaSlow_, alphaFast_, omegaSlow_, omegaFast_;
    int scanStep_;
    bool scanMightInvalid_;
    double resampleThresholdESS_;

    // localization result
    double totalLikelihood_, averageLikelihood_, maxLikelihood_;
    double amclRandomParticlesRate_, effectiveSampleSize_;
    int maxLikelihoodParticleIdx_;

    // other parameters
    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener tfListener_;
    bool isInitialized_;
    double localizationHz_;
    double transformTolerance_;
    double prevTime;

    // reliability estimation
    bool estimateReliability_;
    int classifierType_;
    double reliability_;
    std::vector<double> reliabilities_, glSampledPosesReliabilities_;
    std::vector<double> relTransDDM_, relTransODM_;

    // mean absolute error (MAE)-based failure detector
    MAEClassifier maeClassifier_;
    std::string maeClassifierDir_;
    std::vector<double> maes_, glSampledPosesMAEs_;

    // global-localization-based pose sampling
    bool useGLPoseSampler_, fuseGLPoseSamplerOnlyUnreliable_;

    double finalMAE;

    // constant parameters
    const double rad2deg_;

public:
    // inline setting functions
    void setCanUpdateScan(bool canUpdateScan);

    // inline getting functions
    inline std::string getMapFrame(void); 
    inline sensor_msgs::LaserScan getScan(void); 
    inline int getParticlesNum(void);
    inline CRmclPose getParticlePose(int i);
    inline double getParticleW(int i);
    inline CRmclPose getBaseLink2Laser(void);
    inline int getScanStep(void);
    inline int getMaxLikelihoodParticleIdx(void);
    inline double getNormConstHit(void);
    inline double getDenomHit(void);
    inline double getZHit(void);
    inline double getMeasurementModelRandom(void);
    double getReliability(void);
    double getFinalMAE();
    
    // inline setting functions
    inline void setMCLPoseStamp(ros::Time stamp);
    inline void setParticleW(int i, double w);
    inline void setTotalLikelihood(double totalLikelihood);
    inline void setAverageLikelihood(double averageLikelihood);
    inline void setMaxLikelihood(double maxLikelihood);
    inline void setMaxLikelihoodParticleIdx(int maxLikelihoodParticleIdx);

    // inline other functions
    void clearLikelihoodShiftedSteps(void);
    void addLikelihoodShiftedSteps(bool flag);

    void updateParticlesByMotionModel(void);
    void calculateLikelihoodsByMeasurementModel(void);
    void calculateLikelihoodsByDecisionModel(void);
    void calculateGLSampledPosesLikelihood(void);
    void calculateAMCLRandomParticlesRate(void);
    void calculateEffectiveSampleSize(void);
    void resampleParticles(void);
    template <typename T> std::vector<T> getResidualErrors(CRmclPose pose);
    void plotLikelihoodMap(void);
    
    void estimatePose(void);
    void printResult(void);
    void updateMapData(u8 *map,tGridmapInfo info);
    void updateLaserData(sensor_msgs::LaserScan msg);
    void updateOdomData(tPose robotPose);
    void setInitPose(tPose initPose);
    tPose getExtimatedPose();
    geometry_msgs::PoseArray getParticles();
  
private:
    inline double nrand(double n);
    inline bool onMap(int u, int v);
    inline void xy2uv(double x, double y, int *u, int *v);
    void resetParticlesDistribution(void);
    void resetReliabilities(void);
    double calculateLikelihoodFieldModel(CRmclPose pose, double range, double rangeAngle);
    double calculateBeamModel(CRmclPose pose, double range, double rangeAngle);
    double calculateClassConditionalMeasurementModel(CRmclPose pose, double range, double rangeAngle);
    void estimateUnknownScanWithClassConditionalMeasurementModel(CRmclPose pose);


};