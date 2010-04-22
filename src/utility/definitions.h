#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define FARCLIPPING     1001.0
#define NEARCLIPPING    0.05

#define	PI				3.14159265358979323846
#define TWOPI			2*PI
#define HALFPI 			1.57079632679489661923
#define SQRTTWO			1.41421356
#define DEGTORAD(deg)           (((deg) * PI) / 180.0)
#define RADTODEG(rad)           (((rad) * 180.0) / PI)
#define MINIMUM(x,y)            (((x) < (y)) ? (x) : (y))
#define MAXIMUM(x,y)            (((x) > (y)) ? (x) : (y))
#define SQ(x)			((x)*(x))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _coordAngle {
        int deg;
        double min;
        double dd;
} coordAngle;

typedef	struct _LatLonCoord {
        coordAngle lat;
        coordAngle lon;
} LatLonCoord;

typedef struct _Vertex {
        float x;
        float y;
        float z;
} Vertex;

typedef struct _Point {
    float u;
    float v;
} Point;

typedef struct _Triangle {
        int v1;
        int v2;
        int v3;
} Triangle;

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
	Vertex			position;
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

Vertex diff(Vertex v1,Vertex v2);
Vertex mult(Vertex v1,Vertex v2);
Vertex normalize(Vertex v1);
Vertex normalCross(Vertex v1,Vertex v2);
Vertex vertToVec(Vertex a,Vertex b);
Vertex sumNormals(int v0,int v1,int v2,int v3,int v4,Vertex *verts);
float distBtwVerts(Vertex p1,Vertex p2);

#ifdef __cplusplus
}
#endif

#endif // DEFINITIONS_H
