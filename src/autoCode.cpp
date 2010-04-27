#include "autoCode.h"
/*
*	Edited and ported by Matt Roman 4/17/10
*	Created by David P. Miller on 7/22/08.
*  	Copyright 2008 University of Oklahoma. All rights reserved.
*/

#define OBSTNAVGROUP	"ObstacleNav" // settings group key value


autoCode::autoCode(SR2rover *bot,QList<WayPoint> *list, QWidget* parent)
:
QWidget(parent),
sr2(bot),
running(false),
state(RSInTeleopMode),
error(RENone),
WPlist(list),
wpIndex(0),
wpRange(0),
expectedDistance(0),
blockedDirection(0),
lastBlockedDirection(0),
// location of the settings file ~/.config/OUengineering/Rover_Sim.ini
simSettings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim")
{
	setupUi(this);
	move(20,55);
	setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Navigation Info.");
	
	initSettingsNames();
	
	if(!QFile::exists(simSettings.fileName())) initSettings();
	
	parameterListInit();
	tableSetup();
	
	roverStateKeyMapping();
	roverErrorKeyMapping();
	
	stopAutonomous(RSInTeleopMode);
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
// Returns the distacne from the rover position to i'th waypoint in list
float autoCode::distanceToWaypoint(int i)
{
	Vertex rover;
	rover.x = sr2->position.x();
	rover.y = sr2->position.y();
	rover.z = sr2->position.z();
	return distBtwVerts(WPlist->at(i).position,rover);
}

/////////////////////////////////////////
// Gets the rover turning the requested amount (large turns are done as point turns. Small turns done gradually).
// this function just sets the speeds and returns. relHeading = turn angle relative to the rover.
/////////////
void autoCode::driveToward(float relHeading,float distTo)
{ 
	float leftSpeed,rightSpeed;
	if(fabs(relHeading) < TURNACCURACYLIMIT.stuff.toFloat()){ // high speed forward
		distTo -= CLOSEENOUGH.stuff.toFloat();
		if(distTo < 1) leftSpeed = rightSpeed = (ROVERCRUISESPEED.stuff.toFloat() - POINTTURNSPEED.stuff.toFloat())*distTo + POINTTURNSPEED.stuff.toFloat();
		else leftSpeed = rightSpeed = ROVERCRUISESPEED.stuff.toFloat();
		
		if(relHeading > 0.0) rightSpeed = rightSpeed - (relHeading/TURNACCURACYLIMIT.stuff.toFloat())*5;
		else leftSpeed = leftSpeed + (relHeading/TURNACCURACYLIMIT.stuff.toFloat())*5;
	}
	else if(fabs(relHeading) < POINTTURNANGLE.stuff.toFloat()){ // low speed turn
		leftSpeed = rightSpeed = POINTTURNSPEED.stuff.toFloat();
		if(relHeading > 0.0) rightSpeed = rightSpeed - (relHeading/POINTTURNANGLE.stuff.toFloat())*TURNMULTIPLIER.stuff.toFloat();
		else leftSpeed = leftSpeed + (relHeading/POINTTURNANGLE.stuff.toFloat())*TURNMULTIPLIER.stuff.toFloat();
	}
	else{	// point turn
		leftSpeed = rightSpeed = POINTTURNSPEED.stuff.toFloat();
		if(relHeading > 0.0) rightSpeed = -POINTTURNSPEED.stuff.toFloat();// right turn
		else leftSpeed = -POINTTURNSPEED.stuff.toFloat();// left turn
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
	wpRange = distanceToWaypoint(wpIndex);
	
	// if the rover is close enought to the waypoint consider it complete
	// set the waypoint as old and move to the next one
	if(wpRange < CLOSEENOUGH.stuff.toFloat()){ 	
		currentWaypoint.state = WPstateOld;
		WPlist->replace(wpIndex, currentWaypoint);
		wpIndex++;
		// get the next waypoint in the list
		if(wpIndex < WPlist->size()) { 
			currentWaypoint = WPlist->at(wpIndex);
			currentWaypoint.state = WPstateCurrent;
			WPlist->replace(wpIndex, currentWaypoint);
			// set the expected drive distance to the new waypoint
			expectedDistance = sr2->odometer + (1.0+PATHEFFICIENCY.stuff.toFloat()) * distanceToWaypoint(wpIndex);
			state = RSReachedWaypoint;
			updateTool();
			return;
		}
		else{ // waypoint list is complete should be at goal or no plan
			wpIndex = WPlist->size() - 1;
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
		if(lastBlockedPosition.distance2(sr2->position) > GOPASTDISTANCE.stuff.toFloat()){
			lastBlockedDirection = 0;
			state = RSMovingTowardsWaypoint;
			driveToward(relTargetHeading,wpRange);
		}
		else{
			// no obstacle in view so drive past it
			if(state == RSGoingPastObstacles){
				sr2->setRightSpeed(ROVERCRUISESPEED.stuff.toFloat());
				sr2->setLeftSpeed(ROVERCRUISESPEED.stuff.toFloat());
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
	
	updateTool();
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
	criticalDist = (criticalDist < BODYDIST.stuff.toFloat()) ? criticalDist : BODYDIST.stuff.toFloat();

// check rover POSE SLOPE, roll and pitch to so it doesn't flip over	
	// roll maxed out by itself is an emergency
	if(fabs(sr2->roll) > MAXROLL.stuff.toFloat()){
		blockedDirection = (sr2->roll > 0.0) ? 5 /*rolling LEFT*/ : -5 /*rolling RIGHT*/;
		callForHelp(RERoll);
		return false; // Roll is maxed out!!!!!!
	}
	else if(sr2->pitch > MAXPITCH.stuff.toFloat()){
		aheadBlockedLong=1;
		blockedDirection = (sr2->roll > 0.0) ? 5 /*rolling LEFT*/ : -5 /*rolling RIGHT*/;
		slopeObstacle = 1;
	}
	
// look for any BODY blocking obstacles directly in front of the rover
	if(!slopeObstacle || (slopeObstacle && sr2->pitch > PITCHDOWNIGNOREDISTOBSTACLES.stuff.toFloat())){
		for(i=0;i<size;i++){
			// check for near obstacles
			if(ranges[i] < BODYTOOCLOSE.stuff.toFloat()){
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
		if((fabs(heights[i]) > PANELOBSTACLEMAX.stuff.toFloat()) 
			&& (fabs(heights[i+1]) > PANELOBSTACLEMAX.stuff.toFloat()) 
			&& (fabs(heights[i+2]) > PANELOBSTACLEMAX.stuff.toFloat())){
				if(instShort) aheadBlockedShort = 1;
				else instShort = 1;
				// + is blocked on right, - is blocked on left
				blockedDirection = blockedDirection + 1/(7.5 - i);
		}
	}
	for(i=14;i>10; i--){  // left wheel
//is there a rock or hole?
		if((fabs(heights[i]) > PANELOBSTACLEMAX.stuff.toFloat()) 
			&& (fabs(heights[i-1]) > PANELOBSTACLEMAX.stuff.toFloat())
			&& (fabs(heights[i-2]) > PANELOBSTACLEMAX.stuff.toFloat())){
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

// Used for debugging, checks for obstacles without driving to waypoint
void autoCode::quickObstacleCheck()
{
	error = RENone;
	// run the obstacle check function where the waypoint is 3.0 meters away
	checkForObstacles(3.0);
	updateTool();
}

/////////////////////////////////////////
// Drives around obstacles if any were detected
// This function turns wide or sharp or extreme as needed
/////////////
void autoCode::avoidingTurn()
{
	float closeTurnFactor = (TURNFACTOR.stuff.toFloat() > 0.0)? 0.0 : TURNFACTOR.stuff.toFloat();

	// last block was to the left
	if(lastBlockedDirection < 0){
			// obstacle to the left or far off to the right
		if(blockedDirection < 0.0 || (blockedDirection > 0.0 && state == RSAvoidingDistantObstacles)){
			sr2->setLeftSpeed(ROVERCRUISESPEED.stuff.toFloat());
			sr2->setRightSpeed(ROVERCRUISESPEED.stuff.toFloat()*((state == RSAvoidingNearbyObstacles) ? closeTurnFactor : TURNFACTOR.stuff.toFloat()));
			return;
		}
			// obstacle close to the right
		else if(blockedDirection > 0.0){
			sr2->setLeftSpeed(0);
			sr2->setRightSpeed(-ROVERCRUISESPEED.stuff.toFloat());
			return;
		}
	}
	else{//last block was on right
			// obstacle to the right or far off to the left
		if(blockedDirection > 0.0 || (blockedDirection < 0.0 && state == RSAvoidingDistantObstacles)){
			sr2->setRightSpeed(ROVERCRUISESPEED.stuff.toFloat());
			sr2->setLeftSpeed(ROVERCRUISESPEED.stuff.toFloat()*((state == RSAvoidingNearbyObstacles) ? closeTurnFactor : TURNFACTOR.stuff.toFloat()));
			return;
		}
			// obstacle close to the left
		else if(blockedDirection < 0.0){
			sr2->setRightSpeed(0);
			sr2->setLeftSpeed(-ROVERCRUISESPEED.stuff.toFloat());
			return;
		}
	}
}

void autoCode::goAutonomous()
{
	if(WPlist->isEmpty()) return;
	if(wpIndex > WPlist->size()) wpIndex = WPlist->size()-1;
	
	currentWaypoint = WPlist->at(wpIndex);
	currentWaypoint.state = WPstateCurrent;
	WPlist->replace(wpIndex, currentWaypoint);	
	
	running = true;
	state = RSMovingTowardsWaypoint; // reset the rover state
	error = RENone;	// reset the rover error
	expectedDistance = sr2->odometer + (1+PATHEFFICIENCY.stuff.toFloat()) * distanceToWaypoint(wpIndex); // reset the expected distance
	lastBlockedDirection = 0;
	
	connect(sr2, SIGNAL(updated()),this, SLOT(moveToWaypoint()));
	updateTool();
	buttonRunning->setText("Stop Auto");
}

void autoCode::stopAutonomous(RoverState rs)
{
	sr2->stopRobot();
	running = false;
	state = rs;	
	
	disconnect(sr2, SIGNAL(updated()),this,SLOT(moveToWaypoint()));
	updateTool();
	buttonRunning->setText("Go Auto");
	buttonRunning->setChecked(false);
}

/////////////////////////////////////////
// GUI tool access functions
/////////////
void autoCode::updateTool()
{	
	labelState->setText(RSmap[state]);
	labelError->setText(REmap[error]);
	labelWpID->setText(QString::number(currentWaypoint.uuid));
	labelWpDistance->setText(QString::number(wpRange,'f',3));
	if(blockedDirection == 0.0) labelObstDirection->setText("No Obstacles");
	else labelObstDirection->setText(QString::number(blockedDirection));
}
void autoCode::on_buttonRunning_clicked(bool checked)
{
	if(checked)
		goAutonomous();
	else
		stopAutonomous(RSInTeleopMode);
}

/////////////////////////////////////////
// string key mappings
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

/////////////////////////////////////////
// Parameter table widget setup
/////////////
void autoCode::parameterListInit()
{
	 // open a new group in the settings file
	simSettings.beginGroup(OBSTNAVGROUP);
	// set the parameter value from the settings file
	MAXPITCH.stuff.setValue(simSettings.value(MAXPITCH.name,DEGTORAD(15.0)).toFloat());
	// push the parameter onto the parameter list to be displayed in the table view
	Plist << &MAXPITCH;
	
	MAXROLL.stuff.setValue(simSettings.value(MAXROLL.name,DEGTORAD(20.0)).toFloat());
	Plist << &MAXROLL;
	
	ROVERCRUISESPEED.stuff.setValue(simSettings.value(ROVERCRUISESPEED.name,10.0).toFloat());
	Plist << &ROVERCRUISESPEED;
	
	POINTTURNSPEED.stuff.setValue(simSettings.value(POINTTURNSPEED.name,6.0).toFloat()); 
	Plist << &POINTTURNSPEED;
	
	POINTTURNANGLE.stuff.setValue(simSettings.value(POINTTURNANGLE.name,DEGTORAD(30)).toFloat());
	Plist << &POINTTURNANGLE;
	
	TURNACCURACYLIMIT.stuff.setValue(simSettings.value(TURNACCURACYLIMIT.name,DEGTORAD(4)).toFloat());
	Plist << &TURNACCURACYLIMIT;
	
	TURNMULTIPLIER.stuff.setValue(simSettings.value(TURNMULTIPLIER.name,6.0).toFloat());
	Plist << &TURNMULTIPLIER;
	
	TURNFACTOR.stuff.setValue(simSettings.value(TURNFACTOR.name,0.5).toFloat());
	Plist << &TURNFACTOR;
	
	GOPASTDISTANCE.stuff.setValue(simSettings.value(GOPASTDISTANCE.name,3.0).toFloat());
	Plist << &GOPASTDISTANCE;
	
	CLOSEENOUGH.stuff.setValue(simSettings.value(CLOSEENOUGH.name,1.0).toFloat());
	Plist << &CLOSEENOUGH;
	
	BODYDIST.stuff.setValue(simSettings.value(BODYDIST.name,1.5).toFloat());
	Plist << &BODYDIST;
	
	BODYTOOCLOSE.stuff.setValue(simSettings.value(BODYTOOCLOSE.name,0.5).toFloat());
	Plist << &BODYTOOCLOSE;
	
	PANELOBSTACLEMAX.stuff.setValue(simSettings.value(PANELOBSTACLEMAX.name,0.15).toFloat());
	Plist << &PANELOBSTACLEMAX;
	
	PROFILEOBSTACLEMAX.stuff.setValue(simSettings.value(PROFILEOBSTACLEMAX.name,0.15).toFloat());
	Plist << &PROFILEOBSTACLEMAX;
	
	PITCHDOWNIGNOREDISTOBSTACLES.stuff.setValue(simSettings.value(PITCHDOWNIGNOREDISTOBSTACLES.name,DEGTORAD(10)).toFloat());
	Plist << &PITCHDOWNIGNOREDISTOBSTACLES;
	
	PATHEFFICIENCY.stuff.setValue(simSettings.value(PATHEFFICIENCY.name,0.3).toFloat());
	Plist << &PATHEFFICIENCY;
	simSettings.endGroup();
	
	qDebug() << simSettings.fileName() << simSettings.status() << simSettings.allKeys();
}
void autoCode::tableSetup()
{
	paramTableWidget->setColumnCount(2);
	paramTableWidget->setColumnWidth(0,90);
	paramTableWidget->setColumnWidth(1,130);
	paramTableWidget->setRowCount(Plist.size()-1);
	paramTableWidget->showGrid();
	paramTableWidget->setAlternatingRowColors(true);
	paramTableWidget->verticalHeader()->setResizeMode(QHeaderView::Fixed);
	paramTableWidget->horizontalHeader()->setResizeMode(QHeaderView::Fixed);

	QTableWidgetItem* item = new QTableWidgetItem("Parameter");
	paramTableWidget->setHorizontalHeaderItem(0,item);
	item = new QTableWidgetItem("Value");
	paramTableWidget->setHorizontalHeaderItem(1,item);
	
	for(int i = 0; i < Plist.size(); ++i)
	{
		navParam* np = Plist[i];
		item = new QTableWidgetItem((*np).name);
		QFont ft = item->font();
		ft.setPointSize(10);
		item->setFont(ft);
		item->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
		item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
		paramTableWidget->setItem(i,0,item);
		QTableWidgetItem* data = new QTableWidgetItem();
		data->setData(Qt::EditRole,(*np).stuff);
		data->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
		paramTableWidget->setItem(i,1,data);
	}
	connect(paramTableWidget, SIGNAL(cellChanged(int,int)),this,SLOT(tableDataChange(int,int)));
}
void autoCode::tableDataChange(int row, int column)
{
	QTableWidgetItem* item = paramTableWidget->item(row,column);
	navParam* np = Plist[row];
	(*np).stuff = item->data(Qt::EditRole);
	qDebug() << (*np).name;
	simSettings.setValue(QString(OBSTNAVGROUP) + "/" + (*np).name,(*np).stuff);
	simSettings.sync();
}

void autoCode::initSettings()
{
	simSettings.beginGroup(OBSTNAVGROUP); // open a new group in the settings file
	simSettings.setValue(MAXPITCH.name,15.0);
	simSettings.setValue(MAXROLL.name,20.0);
	simSettings.setValue(ROVERCRUISESPEED.name,10);
	simSettings.setValue(POINTTURNSPEED.name,6.0);
	simSettings.setValue(POINTTURNANGLE.name,DEGTORAD(30));
	simSettings.setValue(TURNACCURACYLIMIT.name,DEGTORAD(4));
	simSettings.setValue(TURNMULTIPLIER.name,6.0);	
	simSettings.setValue(TURNFACTOR.name,0.5);
	simSettings.setValue(GOPASTDISTANCE.name,3.0);
	simSettings.setValue(CLOSEENOUGH.name,1.0);
	simSettings.setValue(BODYDIST.name,1.5);
	simSettings.setValue(BODYTOOCLOSE.name,0.5);
	simSettings.setValue(PANELOBSTACLEMAX.name,0.15);
	simSettings.setValue(PROFILEOBSTACLEMAX.name,0.15);	
	simSettings.setValue(PITCHDOWNIGNOREDISTOBSTACLES.name,DEGTORAD(10));
	simSettings.setValue(PATHEFFICIENCY.name,0.3);
	simSettings.endGroup();
	simSettings.sync();
}

// set the stirng value of the parameters for settings and printing to GUI
void autoCode::initSettingsNames()
{
	MAXPITCH.name = "MAX PITCH";
	MAXROLL.name = "MAX ROLL";
	ROVERCRUISESPEED.name = "CRUISE SPEED";
	POINTTURNSPEED.name = "POINT TURN SPEED";
	POINTTURNANGLE.name = "POINT TURN ANGLE";
	TURNACCURACYLIMIT.name = "TURN ACCURACY";
	TURNMULTIPLIER.name = "TURN MULTIPLIER";
	TURNFACTOR.name = "TURN FACTOR";
	GOPASTDISTANCE.name = "GO PAST OBST DISTANCE";
	CLOSEENOUGH.name = "CLOSE ENOUGH";
	BODYDIST.name = "BODY DISTANCE";
	BODYTOOCLOSE.name = "BODY TOO CLOSE";
	PANELOBSTACLEMAX.name = "PANEL OBST MAX";
	PROFILEOBSTACLEMAX.name = "PROFILE OBST MAX";
	PITCHDOWNIGNOREDISTOBSTACLES.name = "PITCH DOWN IGNORE";
	PATHEFFICIENCY.name = "PATH EFFICIENCY";
}