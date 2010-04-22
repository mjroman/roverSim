#include "autoCode.h"
/*
*	Edited and ported by Matt Roman 4/17/10
*	Created by David P. Miller on 7/22/08.
*  	Copyright 2008 University of Oklahoma. All rights reserved.
*/

autoCode::autoCode(SR2rover *bot, QWidget *parent)
:
QWidget(parent),
sr2(bot)
{
	setupUi(this);
	move(20,100);
	setWindowFlags(Qt::WindowStaysOnTopHint);
	
	POINTTURNSPEED = 6;
	POINTTURNANGLE = DEGTORAD(30);
	TURNMULTIPLIER = 6;
	CLOSEENOUGH = 1;
	TURNACCURACYLIMIT = DEGTORAD(4);
	ROVERCRUISESPEED = 10;
	BODYDIST = 1.5;
	PROFILEOBSTACLEMAX = 0.15;
	MAXPITCH = DEGTORAD(15.0);
	MAXROLL = DEGTORAD(20.0);
	PITCHDOWNIGNOREDISTOBSTACLES = DEGTORAD(10);
	BODYTOOCLOSE = 0.5;
	PANELOBSTACLEMAX = 0.15;
	TURNFACTOR = 0.5;
	GOPASTDISTANCE = 2;
	PATHEFFICIENCY = 0.2; 
	
	wpIndex = 0;
	sr2->waypointList[wpIndex].state = WPstateCurrent;
	currentWaypoint = sr2->waypointList[wpIndex];
	currentWaypointDisplay = true;
	
	roverStateKeyMapping();
	roverErrorKeyMapping();
	waypointStateKeyMapping();
	waypointScienceKeyMapping();
	stopAutonomous(RSInTeleopMode);
	error = RENone;
	setComboWaypointList();
	updateGUI();
	
	show();
}

autoCode::~autoCode()
{
}

void autoCode::callForHelp(RoverError errCode)
{
	error = errCode;
	stopAutonomous(RSCallingForHelp);
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

float autoCode::distanceToWaypoint(int i)
{
	Vertex rover;
	rover.x = sr2->position.x();
	rover.y = sr2->position.y();
	rover.z = sr2->position.z();
	return distBtwVerts(sr2->waypointList[i].position,rover);
}

void autoCode::goAutonomous()
{
	autoRunning = true;
	connect(sr2, SIGNAL(updated()),this, SLOT(moveToWaypoint()));
	state = RSMovingTowardsWaypoint; // reset the rover state
	error = RENone;	// reset the rover error
	lastBlockedDirection = 0;
	expectedDistance = sr2->odometer + (1+PATHEFFICIENCY) * distanceToWaypoint(wpIndex); // reset the expected distance
	label_running->setText("Auto running");
	updateGUI();
}

void autoCode::stopAutonomous(RoverState rs)
{
	autoRunning = false;
	sr2->stopRobot();
	// disconnect from sr2 update signal
	disconnect(sr2,0,this,0);
	state = rs;
	label_running->setText("Auto Stopped");
	updateGUI();
}

void autoCode::toggleAutonomous()
{
	if(autoRunning) stopAutonomous(RSInTeleopMode);
	else goAutonomous();
}

/////////////////////////////////////////
// Gets the rover turning the requested amount (large turns are done as point turns. Small turns done gradually).
// this function just sets the speeds and returns. relHeading = turn angle relative to the rover.
/////////////
void autoCode::driveToward(float relHeading,float distTo)
{ 
	float leftSpeed,rightSpeed;
	if(fabs(relHeading) < TURNACCURACYLIMIT){ // high speed forward
		distTo -= CLOSEENOUGH;
		if(distTo < 1) leftSpeed = rightSpeed = (ROVERCRUISESPEED - POINTTURNSPEED)*distTo + POINTTURNSPEED;
		else leftSpeed = rightSpeed = ROVERCRUISESPEED;
		
		if(relHeading > 0.0) rightSpeed = rightSpeed - (relHeading/TURNACCURACYLIMIT)*5;
		else leftSpeed = leftSpeed + (relHeading/TURNACCURACYLIMIT)*5;
	}
	else if(fabs(relHeading) < POINTTURNANGLE){ // low speed turn
		leftSpeed = rightSpeed = POINTTURNSPEED;
		if(relHeading > 0.0) rightSpeed = rightSpeed - (relHeading/POINTTURNANGLE)*TURNMULTIPLIER;
		else leftSpeed = leftSpeed + (relHeading/POINTTURNANGLE)*TURNMULTIPLIER;
	}
	else{	// point turn
		leftSpeed = rightSpeed = POINTTURNSPEED;
		if(relHeading > 0.0) rightSpeed = -POINTTURNSPEED;// right turn
		else leftSpeed = -POINTTURNSPEED;// left turn
	}	
	sr2->setRightSpeed(rightSpeed);
	sr2->setLeftSpeed(leftSpeed);	
}

/////////////////////////////////////////
// THIS IS THE HIGHEST LEVEL FUNCTION CALL !!!!
// Moves robot towards waypoint.  When WP is reached it stops rover.
// Note: if WP is almost close enough and rover is avoiding obstacles, that WP will be skipped
/////////////
void autoCode::moveToWaypoint()
{
	float wpRange = distanceToWaypoint(wpIndex);
	
	// if the rover is close enought to the waypoint consider it complete
	// set the waypoint as old and move to the next one
	if(wpRange < CLOSEENOUGH){ 	
		sr2->waypointList[wpIndex].state = WPstateOld;	
		wpIndex++;
		// get the next waypoint in the list
		if(wpIndex < sr2->waypointList.size()) { 
			sr2->waypointList[wpIndex].state = WPstateCurrent;
			currentWaypoint = sr2->waypointList[wpIndex];
			// set the expected drive distance to the new waypoint
			expectedDistance = sr2->odometer + (1.0+PATHEFFICIENCY) * distanceToWaypoint(wpIndex);
			state = RSReachedWaypoint;
			updateGUI();
			return;
		}
		else{ // waypoint list is complete should be at goal or no plan
			wpIndex = sr2->waypointList.size() - 1;
			stopAutonomous(RSNoRemainingWaypoints);
			return;
		}
	}
	
// if the rover has driven farther than expected to the next waypoint
	if(expectedDistance < sr2->odometer){
		// stop and call for help
		callForHelp(REProgress);
		return;
	}

	float relTargetHeading = roverWaypointHeading();
		
// check for obstacles, returns false if there was an error
	if(!checkForObstacles(wpRange)) return;
	
	// drive to waypoint
	if(state == RSMovingTowardsWaypoint){
		lastBlockedDirection = 0;
		driveToward(relTargetHeading,wpRange);	
	}
	// drive past obstacle
	else if(state == RSGoingPastObstacles){
		// continue driving to the waypoint if past the obstacle
		if(lastBlockedPosition.distance2(sr2->position) > GOPASTDISTANCE){
			lastBlockedDirection = 0;
			state = RSMovingTowardsWaypoint;
			driveToward(relTargetHeading,wpRange);
		}
		else{
			// no obstacle in view so drive past it
			if(state == RSGoingPastObstacles){
				sr2->setRightSpeed(ROVERCRUISESPEED);
				sr2->setLeftSpeed(ROVERCRUISESPEED);
			}
		}
	}
	// avoid obstacles
	else{
		lastBlockedPosition = sr2->position;  // mark this spot as latest position where obstacle was seen
		avoidingTurn();
	}
	
	if(blockedDirection < 0.0) lastBlockedDirection = -1;
	else lastBlockedDirection = 1;
	
	updateGUI();
}

/////////////////////////////////////////
// Checks for obstacles and sets the direction the rover is blocked , + is right, - is left
// sets the rover state if avoiding near or far, driving past, or driving to waypoint
/////////////
bool autoCode::checkForObstacles(float distTo)
{
	float criticalDist = distTo;
	int size = sr2->getLaserScanner(PANELLASER)->getDataSize();
	float* heights = sr2->getPanelLaserHeights();
	float* ranges = sr2->getLaserScanner(BODYLASER)->getData();
	int i,aheadBlockedLong=0, aheadBlockedShort=0, instShort=0,instLong=0;
	static int prevBlockedShort=0, prevBlockedLong=0;
	int slopeObstacle = 0;
	
	blockedDirection = 0;
	
	//set to minimum of critDistance and BODYDIST
	criticalDist = (criticalDist < BODYDIST) ? criticalDist : BODYDIST;

// check rover POSE SLOPE, roll and pitch to so it doesn't flip over	
	// roll maxed out by itself is an emergency
	if(fabs(sr2->roll) > MAXROLL){
		blockedDirection = (sr2->roll > 0.0) ? 5 /*rolling LEFT*/ : -5 /*rolling RIGHT*/;
		callForHelp(RERoll);
		return false; // Roll is maxed out!!!!!!
	}
	else if(sr2->pitch > MAXPITCH){
		aheadBlockedLong=1;
		blockedDirection = (sr2->roll > 0.0) ? 5 /*rolling LEFT*/ : -5 /*rolling RIGHT*/;
		slopeObstacle = 1;
	}
	
// look for any BODY blocking obstacles directly in front of the rover
	if(!slopeObstacle || (slopeObstacle && sr2->pitch > PITCHDOWNIGNOREDISTOBSTACLES)){
		for(i=0;i<size;i++){
			// check for near obstacles
			if(ranges[i] < BODYTOOCLOSE){
				if(instShort) aheadBlockedShort = 1;
				else instShort = 1;
				// + is blocked on right, - is blocked on left
				blockedDirection = blockedDirection + 2/(i - 7.5);
			}
			// check for far obstacles
			else if(ranges[i] < criticalDist 
				&& ranges[i+1] < criticalDist
				&& i > 3
				&& i < 13){
				if(instLong) aheadBlockedLong = 1;
				else instLong = 1;
				// + is blocked on right, - is blocked on left
				blockedDirection = blockedDirection + 1/(i - 7.5);
			}
		}
	}
	//something directly ahead turn right
	if(blockedDirection == 0.0 && (aheadBlockedShort || aheadBlockedLong)) blockedDirection = -2.0;
		
// look through PANEL laser points in front of right and left wheels for obstacles
	for(i=1;i<5; i++){	// right wheel
	//is there a rock or hole?
		if((fabs(heights[i]) > PANELOBSTACLEMAX) 
			&& (fabs(heights[i+1]) > PANELOBSTACLEMAX) 
			&& (fabs(heights[i+2]) > PANELOBSTACLEMAX)){
				if(instShort) aheadBlockedShort = 1;
				else instShort = 1;
				// + is blocked on right, - is blocked on left
				blockedDirection = blockedDirection + 1/(7.5 - i);
		}
	}
	for(i=14;i>10; i--){  // left wheel
//is there a rock or hole?
		if((fabs(heights[i]) > PANELOBSTACLEMAX) 
			&& (fabs(heights[i-1]) > PANELOBSTACLEMAX)
			&& (fabs(heights[i-2]) > PANELOBSTACLEMAX)){
				if(instShort)aheadBlockedShort=1;
				else instShort = 1;
				// + is blocked on right, - is blocked on left
				blockedDirection = blockedDirection + 2/(7.5 - i);
		}
	}
	
	// mark that you are avoiding
	if(aheadBlockedLong) state = RSAvoidingDistantObstacles;
	else if(aheadBlockedShort) state = RSAvoidingNearbyObstacles;
	else if(prevBlockedShort || prevBlockedLong) state = RSGoingPastObstacles;
	else if(state != RSGoingPastObstacles) state = RSMovingTowardsWaypoint;
	
	prevBlockedShort = aheadBlockedShort;
	prevBlockedLong = aheadBlockedLong;
	return true;
}

// checks for obstacles without driving to waypoint, used for debugging
void autoCode::quickObstacleCheck()
{
	error = RENone;
	// run the obstacle check function where the waypoint is 3.0 meters away
	checkForObstacles(3.0);
	updateGUI();
}

/////////////////////////////////////////
// Drives around obstacles if any were detected
// This function turns wide or sharp or extreme as needed
/////////////
void autoCode::avoidingTurn()
{
	float closeTurnFactor = (TURNFACTOR > 0.0)? 0.0 : TURNFACTOR;

	// last block was to the left
	if(lastBlockedDirection < 0){
			// obstacle to the left or far off to the right
		if(blockedDirection < 0.0 || (blockedDirection > 0.0 && state == RSAvoidingDistantObstacles)){
			sr2->setLeftSpeed(ROVERCRUISESPEED);
			sr2->setRightSpeed(ROVERCRUISESPEED*((state == RSAvoidingNearbyObstacles) ? closeTurnFactor : TURNFACTOR));
			return;
		}
			// obstacle close to the right
		else if(blockedDirection > 0.0){
			sr2->setLeftSpeed(0);
			sr2->setRightSpeed(-ROVERCRUISESPEED);
			return;
		}
	}
	else{//last block was on right
			// obstacle to the right or far off to the left
		if(blockedDirection > 0.0 || (blockedDirection < 0.0 && state == RSAvoidingDistantObstacles)){
			sr2->setRightSpeed(ROVERCRUISESPEED);
			sr2->setLeftSpeed(ROVERCRUISESPEED*((state == RSAvoidingNearbyObstacles) ? closeTurnFactor : TURNFACTOR));
			return;
		}
			// obstacle close to the left
		else if(blockedDirection < 0.0){
			sr2->setRightSpeed(0);
			sr2->setLeftSpeed(-ROVERCRUISESPEED);
			return;
		}
	}
}

/////////////////////////////////////////
// GUI window update functions and setup
/////////////
void autoCode::roverStateKeyMapping()
{
	RSmap[RSStopped] = "Rover Stopped";
	RSmap[RSMovingTowardsWaypoint] = "Moving toward Waypoint";
	RSmap[RSNoRemainingWaypoints] = "No remaining Waypoints";
	RSmap[RSGoingToSleep] = "Going to Sleep";
	RSmap[RSWakingFromSleep] = "Waking from Sleep";
	RSmap[RSDoingScience] = "Doing Science";
	RSmap[RSCallingForHelp] = "Calling for Help";
	RSmap[RSInTeleopMode] = "Teleoperation Mode";
	RSmap[RSAvoidingNearbyObstacles] = "Avoiding Near Obstacles";
	RSmap[RSAvoidingDistantObstacles] = "Avoiding Far Obstacles";
	RSmap[RSReachedWaypoint] = "Waypoint Reached";
	RSmap[RSGoingPastObstacles] = "Driving Past Obstacles";
}

void autoCode::roverErrorKeyMapping()
{
	REmap[RENone] = "---";
	REmap[REPosition] = "Position Error";
	REmap[REStall] = "Wheel Stall";
	REmap[REProgress] = "No Progress";
	REmap[REPitch] = "Pitch Max";
	REmap[RERoll] = "Roll Max";
}

void autoCode::waypointStateKeyMapping()
{
	WPmap[WPstateNew] = "To be visited";
	WPmap[WPstateOld] = "Visited";
	WPmap[WPstateCurrent] = "Driving to now";
	WPmap[WPstateSkipped] = "Skipped";
}

void autoCode::waypointScienceKeyMapping()
{
	WSmap[WPscienceNone] = "No Science";
	WSmap[WPsciencePanorama] = "Panorama";
	WSmap[WPscienceSpectra] = "Spectra";
	WSmap[WPsciencePanoramaAndSpectra] = "Pan and Spectra";
}

void autoCode::setComboWaypointList()
{
	combo_wpSelect->clear();
	for(int i = 0; i < sr2->waypointList.size(); ++i)
	{
		combo_wpSelect->addItem(QString::number(sr2->waypointList[i].uuid),QVariant(i));
	}
}

void autoCode::on_combo_wpSelect_activated(int index)
{
	currentWaypointDisplay = false;
	updateGUI();
	QTimer::singleShot(5000,this, SLOT(displayCurrentWaypoint()));
}

void autoCode::updateGUI()
{
	label_state->setText(RSmap[state]);
	label_error->setText(REmap[error]);
	label_wpCount->setText(QString::number(wpIndex));
	label_obstDirection->setText(QString::number(blockedDirection,'f',4));
	
	int i;
	if(currentWaypointDisplay) {
		i = wpIndex;
		combo_wpSelect->setCurrentIndex(i);
	}
	else i = combo_wpSelect->currentIndex();
	
	WayPoint w = sr2->waypointList[i];
	label_wpState->setText(WPmap[w.state]);
	label_wpScience->setText(WSmap[w.science]);
	label_wpPosition->setText(QString("(%1 ,%2 ,%3)")
								.arg(w.position.x,0,'f',2)
								.arg(w.position.y,0,'f',2)
								.arg(w.position.z,0,'f',2));
	label_wpDistance->setText(QString::number(distanceToWaypoint(i),'f',3));
	label_debug->setText(QString::number(expectedDistance));
}

void autoCode::displayCurrentWaypoint()
{
	currentWaypointDisplay = true;
	updateGUI();
}