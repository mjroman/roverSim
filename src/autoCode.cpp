#include "autoCode.h"
#include "utility/definitions.h"
/*
*	Edited and ported by Matt Roman 4/17/10
*	Created by David P. Miller on 7/22/08.
*  	Copyright 2008 University of Oklahoma. All rights reserved.
*/

autoCode::autoCode(SR2rover *bot)
:
sr2(bot)
{
	POINTTURNSPEED=750;
		POINTTURNANGLE = 30;
	TURNMULTIPLIER = 6;
}

autoCode::~autoCode()
{
}

void autoCode::callForHelp(int errCode)
{
	sr2->stopRobot();
	state = RSCallingForHelp;
	error = errCode;
}
// Convert from compass angles where N=0 and E=90 to Cartesian where N=90 and E=0
float autoCode::compassToCartDegrees(float deg)
{
	deg = 360 - (deg-90);
	if(deg > 360) deg -= 360;
	return deg;
}
// finds the acute half angle between two angles
float autoCode::avgAngle(float deg1, float deg2)
{
	float avgAngle;
	avgAngle = (deg1 + deg2)/2;
	if((MAXIMUM(deg1,deg2)-MINIMUM(deg1,deg2)) > 180){ // two heading are obtuse
		avgAngle += 180;	// compute the acute angle
		if(avgAngle >= 360) avgAngle -= 360;
	}
	return avgAngle;
}
// Returns the heading angle to the waypoint in compass degrees
float autoCode::compassWaypointHeading(int wp)
{
	float xDiff = sr2->waypointList[wp].position.x - sr2->position.x();
	float yDiff = sr2->waypointList[wp].position.x - sr2->position.y();
	if(xDiff == 0 && yDiff == 0) return sr2->heading;	// if the rover is on the waypoint return rovers current heading
	return compassToCartDegrees(RADTODEG(atan2(yDiff,xDiff)));
}
// Retruns the angle to the waypoint relative to the rover
float autoCode::roverWaypointHeading(int wp)
{
	float absHeading = compassWaypointHeading(wp);
	float relHeading = absHeading - sr2->heading;
	if(relHeading < -180) return relHeading+360;
	if(relHeading > 180) return relHeading-360;
	return relHeading;
}

float autoCode::distanceToWaypoint(int wp)
{
	Vertex rover;
	rover.x = sr2->position.x();
	rover.y = sr2->position.y();
	rover.z = sr2->position.z();
	return distBtwVerts(sr2->waypointList[wp].position,rover);
}
// // gets the rover turning the requested amount (large turns are done as point turns. Small turns done gradually).
// this function just sets the speeds and returns. relHeading = turn angle relative to the rover.
void autoCode::turnToHeading(float relHeading)
{ 
	if(relHeading > 0.0){	// Right Turn
		sr2->leftSpeed = POINTTURNSPEED;
		if(relHeading > POINTTURNANGLE) // make a sharp right turn
			sr2->rightSpeed = -POINTTURNSPEED;
		else
			sr2->rightSpeed = POINTTURNSPEED - relHeading * TURNMULTIPLIER;
	}
	else{					// Left Turn
		sr2->rightSpeed = POINTTURNSPEED;
		if(relHeading < -POINTTURNANGLE) // make a sharp left turn
			sr2->leftSpeed = -POINTTURNSPEED;
		else
			sr2->leftSpeed = POINTTURNSPEED + relHeading * TURNMULTIPLIER;
	}
}

void autoCode::moveToWaypoint()
{
	
}