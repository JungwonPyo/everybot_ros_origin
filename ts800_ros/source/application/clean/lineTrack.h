#pragma once

#include "coreData/serviceData.h"
#include "control/control.h"
#include "avoidingWallTrack.h"

enum class E_LINE_TRACK_STEP
{
	CHECK_ROBOT_LOCATION,
	GO_STRAIGHT,
	GOAL_CHECK_VALID,
	GO_CORNER,
	SET_LINE,
	LINE_HEADING,
	LINE_TRACK,
	CHECK_LINE_TRACK,
	WALL_TRACK,
	ERROR,
	COMPLETE,
};


class CLineTrack
{
public:
    CLineTrack();
    ~CLineTrack();
	
	void initLineTrack(std::list<tPoint> boundary);
	bool runLineTrack();
private:
    /* data */
	CAvoidingWallTrack avoiding;

	std::list<tPoint> boundaryPoints;
	tPose			  startRobotPose;
	tPose			  comparePose;
	E_LINE_TRACK_STEP lineTrackStep;
	tPoint 			  targetCornerPoint;
	tPoint 			  startCornerPoint;

	E_LINE_TRACK_STEP procCheckRobotLocation(tPose robotPose);
	E_LINE_TRACK_STEP procGoStraight(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
	E_LINE_TRACK_STEP procGoCorner(tPose robotPose);
	E_LINE_TRACK_STEP procGoalCheckValid(tPose robotPose);
	E_LINE_TRACK_STEP procSetLine(tPose robotPose);
	E_LINE_TRACK_STEP procLineHeading(tPose robotPose);
	E_LINE_TRACK_STEP procLineTrack(tPose robotPose);
	E_LINE_TRACK_STEP procCheckLineTrack(tPose robotPose, RSU_OBSTACLE_DATA *pObstacle);
	E_LINE_TRACK_STEP procWallTrack(tPose robotPose);

	double distanceFromRobotToLine(tPoint point1, tPoint point2, tPose robotPose);
	bool   checkRobotInBoundary(std::list<tPoint> boundary, tPose robotPose);
	bool   checkRobotNearBoundary(std::list<tPoint> boundary, tPose robotPose);
	bool   checkEndLineTrack(tPose robotPose);

	void setLineTrackState(E_LINE_TRACK_STEP ret);
	E_LINE_TRACK_STEP getLineTrackState();
};