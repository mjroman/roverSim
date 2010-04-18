#ifndef AUTOCODE_H
#define AUTOCODE_H

#include "sr2rover.h"

typedef	enum _RoverState{
	RSStopped = 0, //Rover is stopped (may or may not be remaining WP). State for resetting position, etc
	RSMovingTowardsWaypoint = 1, // Normal state when moving
	RSNoRemainingWaypoints = 2, // No plan or plan is finished
	RSGoingToSleep = 3, //Power monitor or user has told rover to sleep
	RSWakingFromSleep = 4, // after mac has woke up but while senors, etc are booting
	RSDoingScience = 5, // taking pictures, etc.  Rover is not moving
	RSCallingForHelp = 6, // progress alarm, long stalls, or teleport problems
	RSInTeleopMode = 7, // when being teleoperated
	RSAvoidingNearbyObstacles = 8, // when path is being modified to avoid obstacles sensed by panel or profile laser
	RSAvoidingDistantObstacles = 9, // when path is being modified to avoid obstacles sensed by body laser
	RSReachedWaypoint = 10, // a momentary state when the point is reached but science or next point not started
	RSGoingPastObstacles = 11  // the rover is ignoring the current waypoint direction in order to drive clear of an obstacle
} RoverState;

typedef	enum _RoverError{
	RENone = 0,
	REPosition = 1,
	REStall = 2,
	REProgress = 3,
	REPitch = 4,
	RERoll = 5
} RoverError;

class autoCode : public QObject
{
	Q_OBJECT
	private:
		SR2rover 	*sr2;
		RoverState 	state;
		RoverError	error;
		int			wpCompleteCount;
		WayPoint 	currentWaypoint;
		
		void callForHelp(RoverError errCode);
		float compassToCartRadians(float rad);
		float avgAngle(float rad1, float rad2);
		float compassWaypointHeading();
		//float roverWaypointHeading();
		float distanceToWaypoint();
		void turnToward(float relHead);
		
	protected:
		float	POINTTURNSPEED;
		float	POINTTURNANGLE;
		float	TURNMULTIPLIER;	
		float	CLOSEENOUGH;
		float 	TURNACCURACYLIMIT;
		int		ROVERCRUISESPEED;		
		
	public:
		int getCurrentWaypoint() { return wpCompleteCount; }
		float roverWaypointHeading();
		autoCode(SR2rover *bot);
		~autoCode();
	
	public slots:
		void moveToWaypoint();
};

#endif //AUTOCODE_H