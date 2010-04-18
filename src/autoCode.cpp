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
	connect(sr2, SIGNAL(updated()),this, SLOT(moveToWaypoint()));
	POINTTURNSPEED = 6;
	POINTTURNANGLE = DEGTORAD(30);
	TURNMULTIPLIER = 4;
	CLOSEENOUGH = 1;
	TURNACCURACYLIMIT = DEGTORAD(4);
	ROVERCRUISESPEED = 18;
	
	wpCompleteCount = 0;
	currentWaypoint = sr2->waypointList[wpCompleteCount];
}

autoCode::~autoCode()
{
}

void autoCode::callForHelp(RoverError errCode)
{
	sr2->stopRobot();
	state = RSCallingForHelp;
	error = errCode;
}
// Convert from compass angles where N=0 and E=90 to Cartesian where N=90 and E=0
float autoCode::compassToCartRadians(float rad)
{
	rad = TWOPI - (rad-HALFPI);
	if(rad > TWOPI) rad -= TWOPI;
	return rad;
}
// finds the acute half angle between two angles IN RADIANS
float autoCode::avgAngle(float rad1, float rad2)
{
	float avgAngle;
	avgAngle = (rad1 + rad2)/2;
	if((MAXIMUM(rad1,rad2)-MINIMUM(rad1,rad2)) > PI){ // two heading are obtuse
		avgAngle += PI;	// compute the acute angle
		if(avgAngle >= TWOPI) avgAngle -= TWOPI;
	}
	return avgAngle;
}
// Returns the heading angle to the waypoint in compass RADIANS
float autoCode::compassWaypointHeading()
{
	float xDiff = currentWaypoint.position.x - sr2->position.x();
	float yDiff = currentWaypoint.position.y - sr2->position.y();
	if(xDiff == 0 && yDiff == 0) return sr2->heading;	// if the rover is on the waypoint return rovers current heading
	return compassToCartRadians(atan2(yDiff,xDiff));
}
// Retruns the angle in RADIANS to the waypoint relative to the rover
float autoCode::roverWaypointHeading()
{
	float absHeading = compassWaypointHeading();
	float relHeading = absHeading - sr2->heading;
	if(relHeading < -PI) return relHeading+TWOPI;
	if(relHeading > PI) return relHeading-TWOPI;
	return relHeading;
}

float autoCode::distanceToWaypoint()
{
	Vertex rover;
	rover.x = sr2->position.x();
	rover.y = sr2->position.y();
	rover.z = sr2->position.z();
	return distBtwVerts(currentWaypoint.position,rover);
}
// // gets the rover turning the requested amount (large turns are done as point turns. Small turns done gradually).
// this function just sets the speeds and returns. relHeading = turn angle relative to the rover.
void autoCode::turnToward(float relHeading)
{ 
	if(relHeading > 0.0){	// Right Turn
		sr2->setLeftSpeed(POINTTURNSPEED);
		if(relHeading > POINTTURNANGLE) // make a sharp right turn
			sr2->setRightSpeed(-POINTTURNSPEED);
		else
			sr2->setRightSpeed(POINTTURNSPEED - relHeading * TURNMULTIPLIER);
	}
	else{					// Left Turn
		sr2->setRightSpeed(POINTTURNSPEED);
		if(relHeading < -POINTTURNANGLE) // make a sharp left turn
			sr2->setLeftSpeed(-POINTTURNSPEED);
		else
			sr2->setLeftSpeed(POINTTURNSPEED - relHeading * TURNMULTIPLIER);
	}
}

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//***********************************************************************************
// Moves robot towards waypoint.  When WP is reached it stops rover and returns -1, otherwise
// the distance to the waypoint is returned. Note if WP is almost close enough and rover is 
// avoiding obstacles, that WP will be skipped
void autoCode::moveToWaypoint()
{
	// if the rover is close enought to the waypoint consider it complete
	// set the waypoint as old and move to the next one
	if(distanceToWaypoint() < CLOSEENOUGH){ 	
		sr2->waypointList[wpCompleteCount].state = WPstateOld;	
		wpCompleteCount++;
		if(wpCompleteCount < sr2->waypointList.size()) { // get the next waypoint in the list
			currentWaypoint = sr2->waypointList[wpCompleteCount];
			state = RSMovingTowardsWaypoint;
		}
		else{ // waypoint list is complete should be at goal or no plan
			sr2->stopRobot();
			// disconnect from sr2 update signal
			disconnect(sr2,0,this,0);
			state = RSNoRemainingWaypoints;
			return;
		}
	}

	float relTargetHeading = roverWaypointHeading();	
	// check for obstacles
	
	// if not avoiding drive to waypoint
	if(fabs(relTargetHeading) < TURNACCURACYLIMIT){ // cruise straight
		sr2->setLeftSpeed(ROVERCRUISESPEED);
		sr2->setRightSpeed(ROVERCRUISESPEED);
	}
	else { // turn toward the waypoint
		turnToward(relTargetHeading);
	}
}
