#ifndef STRUCTURES_H
#define STRUCTURES_H

#include <QVariant>
#include <LinearMath/btVector3.h>
class btRigidBody;
class btCollisionObject;

/////////////////////////////////////////
// Path planning structures
/////////////
typedef enum _PathState
{
	PS_SEARCHING = 0,
	PS_COMPLETE,
	PS_GOALOCCLUDED,
	PS_PATHNOTFOUND,
	PS_SWITCHBACKLEFT,
	PS_SWITCHBACKRIGHT,
	PS_NOPROGRESS
} PathState;

typedef struct _minimaPoint
{
	btVector3			point;
	float				progress;
	float				threshold;
	int					spin;
}minimaPoint;
	
typedef struct _rankPoint
{
	btCollisionObject* 	object;
	btVector3			point;
	int					corner;
	float				gScore;
	float				hScore;
	float				fScore;
	int					parentIndex;	
}rankPoint;

typedef struct _goalPath
{
	QList<rankPoint>	points;
	float				length;		// holds the path length to the goal that is shortest
	int					time;		// holds the number of milliseconds for calculating the path
	float				efficiency;
}goalPath;

/////////////////////////////////////////
// object mouse picking structure
/////////////
typedef struct _pickValue
{
	btRigidBody*	rigidbody;
	btVector3		hitPoint;
	btVector3		rotAxis;
	float			dropHeight;
}pickValue;

/////////////////////////////////////////
// Settings parameter structure
/////////////
typedef struct _settingParam {
	QString name;
	QVariant stuff;
}settingParam;

/////////////////////////////////////////
// Rover Waypoint Structures
/////////////
typedef	enum _WPstate{
	WPstateNew = 0, // to be visited
	WPstateOld = 1, // visited
	WPstateCurrent = 2,
	WPstateSkipped = 3
} WPstate;

typedef enum _WPscience {
    WPscienceNone = 0,
    WPsciencePanorama = 1,
    WPscienceSpectra = 2,
    WPsciencePanoramaAndSpectra = 3
} WPscience;

typedef struct _WayPoint{
	int				uuid; //just a number so we can tell one from another
	btVector3		position;
	WPscience		science;
	WPstate			state; // to be visited, visitied, current, etc
}WayPoint;

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

typedef struct _coordAngle {
        int deg;
        double min;
        double dd;
} coordAngle;

typedef	struct _LatLonCoord {
        coordAngle lat;
        coordAngle lon;
} LatLonCoord;

#endif // STRUCTURES_H