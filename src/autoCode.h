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
	private:
		SR2rover 	*sr2;
		int 		state;
		int 		error;
		
		void callForHelp(int errCode);
		float compassToCartDegrees(float deg);
		float avgAngle(float deg1, float deg2);
		float compassWaypointHeading(int wp);
		float roverWaypointHeading(int wp);
		float distanceToWaypoint(int wp);
		void turnToHeading(float relHead);
		void moveToWaypoint();
		
	protected:
		float	POINTTURNSPEED;
		float	POINTTURNANGLE;
		float	TURNMULTIPLIER;			
		
	public:
		autoCode(SR2rover *bot);
		~autoCode();
};

#endif //AUTOCODE_H