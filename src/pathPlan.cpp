#include "pathPlan.h"
#include "cSpace.h"
#include "obstacles.h"
#include "utility/definitions.h"
#include "utility/glshapes.h"
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

#define SPACEMARGIN			0.65

pathPlan::pathPlan(obstacles *obs,simGLView* glView)
:
simGLObject(glView),
m_blocks(obs),
m_CS(NULL),
m_range(0),
m_margin(SPACEMARGIN),
m_step(0.25),
m_breadth(0),
m_saveOn(false),
m_goalOccluded(NULL),
m_efficiencyLimit(0.3),
m_spinProgress(6),
m_spinProgressBase(0),
m_state(PS_SEARCHING),
m_linkViewIndex(0)
{
	// create callbacks to all the drawing methods, these are added to/removed from the display list
	// make sure these remain in order to match with the enumeration
	ACallback<pathPlan> drawCB(this, &pathPlan::drawDebugPath);	m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawCrowFlyLine); 		m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawSavedPaths);			m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawCurrentSearchPath);	m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawRangeFan);			m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawPathBaseLine);		m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawLightTrail);			m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawPathBuildLine);		m_drawingList << drawCB;
	
	m_GP.length = 0;
	m_GP.time = 0;
	m_GP.efficiency = 0;
	
	displayBuildPath(false);
	displayPath(false);
	displayLightTrail(false);
	displayCrowFly(false);
	displaySavedPaths(false);
	displayDebug(false);
	displayCspace(false);
}

pathPlan::~pathPlan()
{
	m_pointPath.clear();
	contactPoints.clear();
	hitPoints.clear();
	for(int i = 0; i < m_pathList.size(); ++i)
	{
		m_pathList[i].points.clear();
	}
	m_pathList.clear();
	m_GP.points.clear();
	if(m_CS) delete m_CS;
	m_CS = 0;
}

void pathPlan::reset()
{
	togglePathReset();
	m_pathList.clear();
	m_GP.points.clear();
	m_GP.length = 0;
	m_GP.time = 0;
	m_GP.efficiency = 0;
}

void pathPlan::goForGoal(btVector3 start, btVector3 end)
{
	memset(&m_startPoint,0,sizeof(rankPoint));
	memset(&m_midPoint,0,sizeof(rankPoint));
	memset(&m_goalPoint,0,sizeof(rankPoint));
	
	m_state = PS_SEARCHING;
	m_startPoint.point = start;
	m_goalPoint.point = end;
	m_straightDistance = start.distance(end);
	m_progressLimit = m_straightDistance / m_efficiencyLimit;
	m_spinDirection = 0;
	
	displayCurrentSearch(true);								// turn on drawing the yellow search lines
	displayBuildPath(true);
	
	this->generateCspace();									// create the C-Space to compute the path in
	
	m_view->overlayString(QString("Searching Range %1").arg(m_range));
	m_firstPath = true;
	
	m_time.start();											// start time of path calculation
	
	if(m_range == 0) {
		this->searchForPath();								// if infinite range find the shortest path
		
		if(m_GP.length == 0) m_state = PS_PATHNOTFOUND;
		else m_state = PS_COMPLETE;
	}
	else {
		m_state = this->cycleToGoal();						// step forward on path until the goal is in range
	
		m_GP.points = m_trailPath + m_GP.points;			// prepend the step trail point list
		m_trailPath.clear();
		m_GP.length = 0;
		for(int i=0; i<m_GP.points.size()-1; i++) 
			m_GP.length += m_GP.points[i].point.distance(m_GP.points[i+1].point);	// calculate the total length
		m_startPoint = m_GP.points[0];						// set the starting point for drawing
	}
	
	m_GP.time = m_time.elapsed();							// get the elapsed time for the path generation
	
	if(m_CS) delete m_CS;									// delete the C Space to free up memory since it is not needed
	m_CS = 0;
	m_pointPath.clear();									// clear out the construction point path
	m_nodeList.clear();										// clear out the node construction list
	
	displayCurrentSearch(false);							// turn off search path drawing
	displayBuildPath(false);
	displayPath(true);
	
	if(m_saveOn) m_view->printText(QString("%1 paths found").arg(m_pathList.size()));
			
	if(m_GP.length != 0) m_GP.efficiency = m_straightDistance/m_GP.length;
}


/////////////////////////////////////////
// Generate a new Configuration Space to plan paths around
/////////////
void pathPlan::generateCspace()
{
	if(m_CS) delete m_CS;
	m_CS = new cSpace(m_startPoint.point,m_range,m_margin,m_blocks,m_view);		// create a new Configuration Space based on the start point
	m_CS->drawCspace(m_displayCS);
	
	if(isGoalInRange()){
		QList<btCollisionObject*>* ghostList = m_CS->getGhostList();
		for(int i=0; i < ghostList->size(); i++)							// check if goal point is inside of a cspace object
		{
			if(m_CS->isPointInsideObject(m_goalPoint.point,ghostList->at(i)))
				m_goalOccluded = ghostList->at(i);
		}
	}
	
	m_midPoint = m_startPoint;												// reset all parameters due to new Cspace
	m_pointPath.clear();
	m_pointPath << m_startPoint;
	m_nodeList.clear();
}

// returns true if the goal is within sensor range of the robot
bool pathPlan::isGoalInRange()
{
	m_goalDistance = m_startPoint.point.distance(m_goalPoint.point);		// calculate the distance to the goal from the start point
	if(m_range == 0) return true;
	if(m_range <= m_goalDistance) return false;
	else return true;
}

/////////////////////////////////////////
// Path creation
/////////////
PathState pathPlan::cycleToGoal()
{
	int i;
	float progress = 0;
	m_trailPath.clear();
	m_trailPath << m_startPoint;
	m_minimaList.clear();
	
	while( progress <= m_progressLimit )										// looping condition based on path efficiency
	{	
		this->searchForPath();													// calculate the path to the goal
		
		if(m_GP.points.size() <= 1){ 											// if only 1 point is in the path list or it is empty then no path is found
			if(m_spinDirection == 1)
				return PS_SWITCHBACKRIGHT;
			else if(m_spinDirection == -1)
				return PS_SWITCHBACKLEFT;
			else
				return PS_PATHNOTFOUND;
		}
		else if( m_range > m_goalDistance ) 
			return PS_COMPLETE;													// a complete path has been found and the goal is in range
		
		i = 1;
		float step = m_step;
		btVector3 strideVect = m_GP.points[i].point - m_trailPath.last().point;	// get the vector to the next step
		
		while(step > strideVect.length()){										// while the step distance is longer than the distance to the next point
			m_trailPath << m_GP.points[i];										// add the current position to the trail path
			step -= strideVect.length();
			i++;
			strideVect = m_GP.points[i].point - m_trailPath.last().point;		// find the vector to the next point on the path
		}
	
		memset(&m_startPoint,0,sizeof(rankPoint));								// create a new start point
		m_startPoint.point = step * strideVect.normalized() + m_trailPath.last().point;	// compute the new point from the end of the trail
		
		btCollisionObject* object = 0;
		if(m_trailPath.last().object) object = m_trailPath.last().object; 		// check if the end of the trail is near a C-Space object
		else if(m_GP.points[i].object) object = m_GP.points[i].object;			// or the next point on the path in the direction headed
		
		if(object){
			btVector3 offset = m_startPoint.point - object->getWorldTransform().getOrigin();
			offset = (0.01 * offset.normalized()) * btVector3(1,1,0);			// move over the start point 1cm from the edge of the C-Space
			m_startPoint.point += offset;
		}
		
		m_trailPath << m_startPoint;
		
		progress += m_step;
		m_GP.length = 0;
		m_GP.points.clear();
		
		this->generateCspace();													// create new C-Space and calculate new goal distance
		
		this->localMinimaCheck(m_trailPath, m_trailPath.size()-1);				// local minima work around
	}
	return PS_NOPROGRESS;														// current path length is not making progress
}


// what is the difference in finding all the open paths and choosing the one with the lowest angle = miller way
// and building a path from the initial intersected object an back calculating from the extremes? = my way
/*
void pathPlan::constructRoadMap()
{
	int i;
	if(this->isRayBlocked(m_midPoint,m_goalPoint) == NULL){
		rankLink l;
		l.first = m_midPoint;
		l.second = m_goalPoint;
		l.length = m_midPoint.point.distance(m_goalPoint.point);
		m_linkList << l;
		return;
	}

	QList<rankPoint> prospectPoints = getVisablePointsFrom(m_midPoint);

	i=0;
	// if(m_linkList.size()){
	// 	while(i < prospectPoints.size()){
	// 	// remove the point if it has already been visited
	// 		if(!isNewPoint(prospectPoints[i]))
	// 			prospectPoints.removeAt(i);
	// 		else i++;	
	// 	}
	// }
	
	
// compute the ranks for the remaining points
	prospectPoints = angleBasedRank(prospectPoints,m_midPoint);
// sort all the points with the smallest rank first, 	
	prospectPoints = quickSortRankLessthan(prospectPoints);

	for(i = 0; i < prospectPoints.size(); ++i)
	{
		rankLink l;
		l.first = m_midPoint;
		l.second = prospectPoints[i];
		if(isNewLink(l)){
			l.length = m_midPoint.point.distance(prospectPoints[i].point);
			m_linkList << l;
		}
		else{
			prospectPoints.removeAt(i);
			i--;
		}
	}
	
	for(i = 0; i < prospectPoints.size() && i < 5; ++i)
	{
		m_midPoint = prospectPoints[i];
		m_view->updateGL();
		constructRoadMap();
	}
}*/


/////////////////////////////////////////
// first path planner algorithm DFSearch, this is where it all happens
/////////////
bool pathPlan::searchForPath(float length)
{
	if(m_GP.length == 0 && m_time.elapsed() > 300000) return false; // 5 minute limit and no path to the goal found exit
	
	float goalDist = m_midPoint.point.distance(m_goalPoint.point);	// find the distance from the current midpoint to the Goal
	
 	if(m_GP.length != 0 && length + goalDist > m_GP.length) 		// incase the new search path is farther than the shortest path
		return false;
	
	btCollisionObject *objBlock = this->isRayBlocked(m_midPoint,m_goalPoint);	// is the ray blocked?
	
	if(objBlock == NULL || objBlock == m_goalOccluded){							// check that the goal point is not occluded by the blocking object
		m_pointPath << m_goalPoint;												// add the goal point to the path
		length += goalDist;						

		if(length < m_GP.length || m_GP.length == 0)							// check if a new shortest path has been found
		{
			if(m_firstPath && m_range == 0) {
				m_view->overlayString("First Path Found");
				m_firstPath = false;
			}
			m_GP.length = length;
			m_GP.points = m_pointPath;											// save the new short path
			if(m_saveOn) m_pathList.push_front(m_GP);
		}
		else if(m_saveOn)
		{
			goalPath newPath;
			newPath.length = length;
			newPath.points = m_pointPath;
			m_pathList.push_back(newPath);
		}
		return true;															// no intersection all done
	}
	
	QList<rankPoint> prospectPoints = getVisablePointsFrom(m_midPoint,length); 	// get all the visable points from the current location
	if(m_range == 0) prospectPoints = this->progressAngleBasedRank(prospectPoints, m_midPoint);	// compute the ranks based on progress angle from start-goal vector
	else prospectPoints = this->angleBasedRank(prospectPoints, m_midPoint); 	// compute the ranks based on angle to goal from midPoint
	prospectPoints = this->prunePointsFrom(prospectPoints);						// remove points that are already in the point path
	
	int i=0;
	while(i < prospectPoints.size() && (m_breadth == 0 || i < m_breadth))
	{
		m_midPoint = prospectPoints[i];											// change the midPoint to the lowest rank point
		m_pointPath << m_midPoint;												// add the potential point to the global path list

		m_view->updateGL();
		
		if(this->searchForPath(prospectPoints[i].length)){						// recursive check for a path to the goal
			m_pointPath.removeLast();											// remove the goal point from the global list
			m_pointPath.removeLast();
			break;																// don't need to do anymore looping since the goal is visible
		}
		else m_pointPath.removeLast();											// else remove the last midPoint and try the next
			
		i++;
	}
	prospectPoints.clear();														// clear all the prospecive points
	return false;
}

/////////////////////////////////////////
// Path Debugging
/////////////
void pathPlan::togglePathReset()
{
	displayPath(true);
	displayBuildPath(false);
	displayDebug(false);
	if(m_CS) delete m_CS;
	m_CS = 0;
	hitPoints.clear();
	contactPoints.clear();
}
// this is a debugging function used to see what paths are available from a given point on the path
// calling this function repeatedly cycles throught the shortest path
void pathPlan::togglePathPoint(int dir)
{
	if(m_GP.points.isEmpty()) return;

	if(m_CS) delete m_CS;
	m_CS = 0;
	hitPoints.clear();
	contactPoints.clear();
	m_spinDirection = 0;
	
	m_linkViewIndex += dir;													// increment the index
	if(m_linkViewIndex >= m_GP.points.size()) m_linkViewIndex = 0;			// make sure the index is in bounds
	else if(m_linkViewIndex < 0) m_linkViewIndex = m_GP.points.size()-1;

	if(!m_displayDebug) displayDebug(true);									// turn on debug drawing

	rankPoint here = m_GP.points[m_linkViewIndex];							// get the point to view from using the index
	here.object = 0;

	m_view->getCamera()->cameraSetDirection(here.point); 					// set the camera view to the path point
	m_CS = new cSpace(here.point,m_range,m_margin,m_blocks,m_view);			// create a new Configuration Space based on the start point
	m_CS->drawCspace(true);
	
	contactPoints = getVisablePointsFrom(here,0);							// gather all objects extreme vertices
	if(m_range == 0) contactPoints = progressAngleBasedRank(contactPoints, m_GP.points[m_linkViewIndex]);
	else contactPoints = angleBasedRank(contactPoints, m_GP.points[m_linkViewIndex]);
	
	if(!contactPoints.isEmpty()) hitPoints << contactPoints[0].point;		// show the most likely path choice
	contactPoints.prepend(m_GP.points[m_linkViewIndex]);					// push the point the current view is from for drawing
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Path generation utility functions
/////////////
void pathPlan::smoothPath()
{
	int i = 0;
	int j = m_GP.points.size()-1;

	while(j > i+1){
		m_view->overlayString(QString("end %1").arg(j));
		while(i < j-1){
			m_view->overlayString(QString("begin %1").arg(i));
			if(isRayBlocked(m_GP.points[j],m_GP.points[i]) == NULL)
			{
				m_view->overlayString("Smooth remove");
				i++;								// get the first point to be erased
				for(int k=0;k<j-i;k++){				
					m_GP.points.removeAt(i);		// do not increment i because the list gets smaller each removal
				}
				j = m_GP.points.size()-1;
				break;
			}
			else
				i++;
		}
		i = 0;
		j--;
	}
	
	m_GP.length = 0;
	for(int k=0; k<m_GP.points.size()-1; k++) 
		m_GP.length += m_GP.points[k].point.distance(m_GP.points[k+1].point);	// calculate the total length
		
}

// checks local minima under limited range sensor paths for switch back condition
// sets the global spin direction and keeps track of spin progress distance
void pathPlan::localMinimaCheck(QList<rankPoint> list, int index)
{
	static float spinDist = 0;
	
	if(list.size() < 3 || index >= list.size() || index < 2) return;	// make sure everything is inbounds
	
	if(m_spinDirection == 0){
		if(spinDist > 0){ spinDist = 0; return; }						// skip the next local minima check after spin progress to avoid switchback condition
		btVector3 preVect = list[index-1].point - list[index-2].point;	// use the previous position to compare
		btVector3 newVect = list[index].point - list[index-1].point;	// to the new position

		if(preVect.dot(newVect) < 0){									// A switchback is detected, dot is negative if the angle is greater than 90
			minimaPoint mp;
			mp.point = list[index-1].point;
			mp.threshold = 2*m_step;
			if(!m_minimaList.isEmpty() && mp.point.distance2(m_minimaList.last().point) < SQ(m_minimaList.last().threshold)){ 
				mp.progress = m_minimaList.last().progress * 2;				// increase the progress limit by twice the previous
				mp.spin = m_minimaList.last().spin * -1;					// flip the direction of spin if near the last local minima
			}
			else {
				if(m_spinProgressBase == 0) mp.progress = m_spinProgress;				// distance based progress
				else if(m_spinProgressBase == 1) mp.progress = m_spinProgress * m_step;	// step based progress
				mp.spin = (newVect.cross(preVect).z() > 0)? -1 : 1;						// 1 = right side -1 = left side
			}
			m_minimaList << mp;
			m_spinDirection = m_minimaList.last().spin;
			spinDist = 0;
		}
	}
	else{
		if(spinDist > m_minimaList.last().progress) m_spinDirection = 0;				// once progress past the local minima has been made reset spin direction
		else spinDist += m_step;
	}
}

// checks the ray between from and to, if it is clear NULL is returned else returns the object blocking
btCollisionObject* pathPlan::isRayBlocked(rankPoint from,rankPoint to, btVector3* point)
{
	QList<rankPoint> hitList;
	QList<btCollisionObject*>* ghostList = m_CS->getGhostList();
	
	for(int i=0;i<ghostList->size();i++){
		int k;
		rankPoint rp;
		rp.object = ghostList->at(i);
		
		if(rp.object == to.object && rp.object == from.object) continue; // checking the edge of an object should be ignored
		
		QList<btVector3> shapePoints = m_CS->getTopShapePoints(rp.object);
		for(int j=0;j<shapePoints.size();j++){
			if(j == shapePoints.size()-1) k=0;
			else k=j+1;
			if(m_CS->segmentIntersection(from.point,to.point,shapePoints[j],shapePoints[k],&rp.point) > 0.){
				rp.rank = from.point.distance2(rp.point);
				hitList << rp;
			}
		}
	}
	
	if(hitList.isEmpty()) return NULL; // no hits all clear
	
	hitList = this->quickSortRankLessthan(hitList); // sort all hit points from closest to farthest
	
	for(int i=0;i<hitList.size();i++){
		if(hitList[i].rank > 0.0){
			if(point) *point = hitList[i].point;
			return hitList[i].object;
		}
	}
	return NULL;
}

// gets the extreme vertecies of the collision object with reference to the midPoint 
void pathPlan::getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right)
{
	float leftMax = 0;
	float rightMax = 0;
	bool setFirst = true;
	btVector3 firstPoint;
	QList<btVector3> ptList;
	
	btTransform objTrans = obj->getWorldTransform();		// the transform of the object being intersected
	ptList = m_CS->getTopShapePoints(obj);					// get all the top points of the obj

	for(int i = 0; i < ptList.size(); ++i)					// cycle through all top points
	{
		if(pivotPoint.object == obj && pivotPoint.corner == i) continue;	// ignore the point if it is equal to the pivot point
		
		btVector3 vert = ptList[i]; 
		//vert.setZ(objTrans.getOrigin().z());								// flatten vectors to 2D
		vert.setZ(m_startPoint.point.z());									// set all extremes to start height
		
		if( setFirst ){ 
			firstPoint = vert; 
			(*left).point = vert;
			(*left).corner = i;
			(*left).object = obj;
			(*right).point = vert;
			(*right).corner = i;
			(*right).object = obj;
			setFirst = false;
			continue; 
		}
		
			// get the cross product to find the direction, left or right side of first point
		btVector3 xc = (firstPoint - pivotPoint.point).cross(vert - pivotPoint.point);
			// get the angle to find the extremes
		float angle = (firstPoint - pivotPoint.point).angle(vert - pivotPoint.point);

			// if cross product between obj center and vertex is + or UP then left point
		if(xc.z() > 0){
			if(angle > leftMax){
				leftMax = angle;
				(*left).point = vert;
				(*left).corner = i;
				(*left).object = obj;
			}
		}
			// else if - or DOWN then right point
		else{
			if(angle > rightMax){
				rightMax = angle;
				(*right).point = vert;
				(*right).corner = i;
				(*right).object = obj;
			}
		}
	}
}

// returns a list of all the visable extremes of C Space obstacles
QList<rankPoint> pathPlan::getVisablePointsFrom(rankPoint here, float dist)
{
	int i;
	rankPoint leftMost;
	rankPoint rightMost;
	QList<rankPoint> list;
	QList<btCollisionObject*>* ghostList = m_CS->getGhostList();

// gather all objects extreme vertices
	for(i = 0; i < ghostList->size(); ++i)										// get all points from individual objects
	{		
		bool lState,rState;
		lState = true;
		rState = true;
		this->getExtremes(ghostList->at(i),here,&leftMost,&rightMost);			// get the far left and right points around the obstacle
		
		if(ghostList->at(i)->getUserPointer())									// check both extremes to make sure they are not inside of a grouped object
		{
			cSpace::overlapGroup* gp = static_cast<cSpace::overlapGroup*>(ghostList->at(i)->getUserPointer());
			for(int j=0;j<gp->list.size();j++)
			{
				if( gp->list[j] == ghostList->at(i) ) continue;					// skip the object the extremes are from
				if( lState && m_CS->isPointInsideObject(leftMost.point,gp->list[j]) )	lState = false;
				if( rState && m_CS->isPointInsideObject(rightMost.point,gp->list[j]) )	rState = false;
				if( !lState && !rState ) break;
			}
		}
		
		// check for spin condition past obstacles and eliminate paths to the left or right depending on spin	
		if(lState && m_spinDirection >= 0) list << leftMost;					// if spin direction is to the right (1) keep all left extremes
		if(rState && m_spinDirection <= 0) list << rightMost;					// if spin is to the left (-1) keep all the right extremes
	}

	i=0;
	while(i < list.size()){
		if(this->isRayBlocked(here,list[i])) 							// remove all remaining points that are blocked
			list.removeAt(i);
		else
			i++;	
	}
	
	for(i=0; i<list.size(); i++)
		list[i].length = here.point.distance(list[i].point) + dist;		// calculate the distance to each point from the midpoint

	return list;
}

// returns a list of all the visable vertices of C Space obstacles
QList<rankPoint> pathPlan::getAllVisablePointsFrom(rankPoint here, float dist)
{
	int i,j;
	QList<rankPoint> list;
	QList<btCollisionObject*>* ghostList = m_CS->getGhostList();

// gather all objects vertices
	for(i = 0; i < ghostList->size(); ++i)										// get all points from individual objects
	{
		QList<btVector3> objPts = m_CS->getTopShapePoints(ghostList->at(i));	// get all the top points that make up the obstacle
		QList<rankPoint> tempList;
		
		for(j = 0; j < objPts.size(); j++){										// convert points into rank points 
			rankPoint rp;
			rp.object = ghostList->at(i);
			rp.point = objPts[j];
			rp.point.setZ(m_startPoint.point.z());
			rp.corner = j;
			tempList << rp;
		}
		
		list += tempList;														// add all remaining rank points from obstacle to the visible list
		objPts.clear();
		tempList.clear();
	}

	i=0;
	while(i < list.size()){
		if(this->isRayBlocked(here,list[i])) 									// remove all remaining points that are blocked
			list.removeAt(i);
		else
			i++;	
	}
	
	if(here.object){															// remove all points between edges of object
		QList<btVector3> objPts = m_CS->getTopShapePoints(here.object);
		if(here.corner < objPts.size()) {
			i = here.corner;													// get the corner
			int nexti,prei;												
			if(i == 0) prei = objPts.size()-1;									// get the adjacent corners
			else prei = i-1;
			if(i == objPts.size()-1) nexti = 0;
			else nexti = i+1;

			btVector3 right = objPts[nexti] - objPts[i];						// get the vectors to the left and right points
			btVector3 left = objPts[prei] - objPts[i];
			j=0;
			while(j < list.size()){
				btVector3 tv = list[j].point - objPts[i];						// test if the cross products are different between the list point
				btVector3 xr = right.cross(tv);
				btVector3 xl = left.cross(tv);
				if(xr.z() > 0 && xl.z() < 0)
					list.removeAt(j);
				else
					j++;
			}
		}
	}
	
	for(i=0; i<list.size(); i++)
		list[i].length = here.point.distance(list[i].point) + dist;				// calculate the distance to each point from the midpoint

	return list;
}

// removes visible points not wanted
QList<rankPoint> pathPlan::prunePointsFrom(QList<rankPoint> list)
{
	int i=0,j;

// check each visible point if it is already in the node list, remove it if it is longer, replace the node if it is shorter, add it if it doesn't exist
	while(i < list.size())
	{
		bool newNode = true;
		for( j = 0; j < m_nodeList.size(); j++)
		{
			if(list[i].object == m_nodeList[j].object && list[i].corner == m_nodeList[j].corner)
			{
				if(list[i].length >= m_nodeList[j].length){			// new visible point is farther so remove it
					list.removeAt(i);
					newNode = false;
					break;
				}
				else{
					m_nodeList.replace(j,list[i]);					// new visible point is shorter so replace it
					newNode = false;
					i++;
					break;
				}
			}
		}
		if(newNode){ 												// new visible point is not in the node list add it
			m_nodeList << list[i];
			i++;
		}
	}
	
	return list;
}

// computes the rank of each point based on the angle between the goal and the pivot point
QList<rankPoint> pathPlan::angleBasedRank(QList<rankPoint> list, rankPoint pivotPoint)
{
	// ranking can not use just the cross product or length of cross product, not enough info
	// have to use the angle between the goal vector and the extreme point
	for(int i = 0; i < list.size(); ++i)
	{
		list[i].rank = (m_goalPoint.point - pivotPoint.point).angle(list[i].point - pivotPoint.point);
	}
	list = this->quickSortRankLessthan(list);	// sort the list with the smallest rank first to the goal
	return list;
}

// computes the rank of each point based on the angle between the goal and the start point
QList<rankPoint> pathPlan::progressAngleBasedRank(QList<rankPoint> list, rankPoint pivotPoint)
{
	int i;
	// ranking can not use just the cross product or length of cross product, not enough info
	// have to use the angle between the goal vector and the extreme point
	for(i = 0; i < list.size(); ++i)
	{
		list[i].rank = (m_goalPoint.point - m_startPoint.point).angle(list[i].point - pivotPoint.point);
		// remove any points where the angle from the midpoint is greater than 90deg to the start-goal vector
	}
	list = this->quickSortRankLessthan(list);	// sort the list with the smallest rank first to the goal
	
	QList<rankPoint> keepers;
	for(i = 0; i < list.size(); ++i)			// to reduce unnecessary searching
	{											// check for paths that search backwards or angles greater than 90 deg
		if(list[i].rank > HALFPI) break;
		else keepers << list[i];				// add all positive progress points
	}
	if(i == 0) keepers << list[0];				// if all paths are behind then only search the one behind
	
	list.clear();
	return keepers;
}

// computes the rank of each point based on the distance from the pivot point
QList<rankPoint> pathPlan::rangeBasedRank(QList<rankPoint> list, rankPoint pivotPoint)
{
	for(int i = 0; i < list.size(); ++i)
	{
		list[i].rank = list[i].point.distance(pivotPoint.point);
	}
	list = this->quickSortRankLessthan(list); 	// sort the list with the smallest rank first to the goal
	return list;
}

// does a quick sort on the list, orders from lowest to highest rank
QList<rankPoint> pathPlan::quickSortRankLessthan(QList<rankPoint> list)
{
	QList<rankPoint> less;
	QList<rankPoint> greater;
	
	if(list.size() <= 1) return list;
	rankPoint pivot = list.takeLast();
	for(int i = 0; i < list.size(); ++i)
	{
		if(list[i].rank < pivot.rank) less << list[i];
		else greater << list[i];
	}
	return (quickSortRankLessthan(less) << pivot << quickSortRankLessthan(greater));
}

// returns true if the link is NEW and not in the global link list yet
/*
bool pathPlan::isNewPoint(rankPoint pt)
{
	rankPoint a,b;
	for(int i = 0; i < m_linkList.size(); ++i)
	{
		a = m_linkList[i].first;
		b = m_linkList[i].second;
		if(a.object == pt.object || b.object == pt.object){
			if(a.corner == pt.corner || b.corner == pt.corner) return false;
		}
	}
	return true;
}

bool pathPlan::isNewLink(rankLink link)
{
	rankPoint a,b;
	for(int i = 0; i < m_linkList.size(); ++i)
	{
		a = m_linkList[i].first;
		b = m_linkList[i].second;
		if(a.object == link.first.object && a.corner == link.first.corner && 
			b.object == link.second.object && b.corner == link.second.corner) return false;
		if(a.object == link.second.object && a.corner == link.second.corner && 
			b.object == link.first.object && b.corner == link.first.corner) return false;
	}
	return true;
}*/


/////////////////////////////////////////
// path display access functions
/////////////
void pathPlan::displayDebug(bool x)
{
	m_displayDebug = x;
	if(x) m_displayList.push_front( &m_drawingList[PP_DEBUG] );
	else m_displayList.removeAll( &m_drawingList[PP_DEBUG] );
	if(m_range) displayRangeFan(x);	
}
void pathPlan::displayCrowFly(bool x)
{
	m_displayCrowFly = x;
	if(x) m_displayList.push_front(&m_drawingList[PP_CROWFLY] );	// insert after the baseline of the path has been drawn
	else m_displayList.removeAll(&m_drawingList[PP_CROWFLY]);
}
void pathPlan::displaySavedPaths(bool x)
{
	m_displaySavedPaths = x;
	if(x) m_displayList.push_front(&m_drawingList[PP_SAVEDPATHS] );	// insert after the baseline of the path has been drawn
	else m_displayList.removeAll(&m_drawingList[PP_SAVEDPATHS]);
}
void pathPlan::displayCurrentSearch(bool x)
{
	if(x) m_displayList.push_back(&m_drawingList[PP_CURRENTSEARCH]);
	else m_displayList.removeAll(&m_drawingList[PP_CURRENTSEARCH]);
}
void pathPlan::displayRangeFan(bool x)
{
	if(x) m_displayList.push_back(&m_drawingList[PP_RANGEFAN]);
	else m_displayList.removeAll(&m_drawingList[PP_RANGEFAN]);
}
void pathPlan::displayPath(bool x)
{
	m_displayPath = x;
	if(x) m_displayList.push_front( &m_drawingList[PP_BASELINE] );	// make sure the base line of the path is drawn first
	else m_displayList.removeAll(&m_drawingList[PP_BASELINE]);
}
void pathPlan::displayLightTrail(bool x)
{
	m_displayLightTrail = x;
	if(x) m_displayList.push_back( &m_drawingList[PP_LIGHTTRAIL] );	// insert after the baseline of the path has been drawn
	else m_displayList.removeAll(&m_drawingList[PP_LIGHTTRAIL]);
}
void pathPlan::displayCspace(bool x)
{
	m_displayCS = x;
	if(m_CS) m_CS->drawCspace(m_displayCS);
}
void pathPlan::displayBuildPath(bool x)
{
	if(x) m_displayList.push_front( &m_drawingList[PP_BUILDPATH] );
	else m_displayList.removeAll( &m_drawingList[PP_BUILDPATH] );
}

/////////////////////////////////////////
// Draw the path's
/////////////
void pathPlan::drawDebugPath()				// draws test points and lines
{
	int i;
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glNormal3f(1,1,1);
	glColor3f(0,1,0);
	for(i = 1; i < contactPoints.size(); ++i)
	{
		glVertex3fv(contactPoints[0].point.m_floats);
		glVertex3fv(contactPoints[i].point.m_floats);
	}
	glEnd();

	glPointSize(6.0);
	glBegin(GL_POINTS);
	glColor3f(0,0,1);
	for(i=0;i<hitPoints.size();i++)
		glVertex3fv(hitPoints[i].m_floats);
	glEnd();
}

void pathPlan::drawCrowFlyLine()			// draws a dark blue line from the start of the path to the goal, used for visual cues and user evaluation
{
	glBegin(GL_LINES);
	glLineWidth(1.0);
	glColor3f(0,0,1);
	glNormal3f(0,0,1);
	glVertex3fv(m_startPoint.point.m_floats);
	glVertex3fv(m_goalPoint.point.m_floats);
	glEnd();
}

void pathPlan::drawSavedPaths()				// if all the paths are being saved draw them in purple
{
	for(int j=1; j < m_pathList.size(); j++){
		glColor3f(0,1,1);
		QList<rankPoint> pp = m_pathList[j].points;
		glBegin(GL_LINE_STRIP);
		for(int i = 0; i < pp.size(); ++i)
		{
			glVertex3fv(pp[i].point.m_floats);
		}
		glEnd();
	}
}

void pathPlan::drawCurrentSearchPath()		// draws a yellow line indicating the current search path while looping
{
	// draws the yellow line on the base of the current search path
	glLineWidth(4.0);
	glBegin(GL_LINE_STRIP);
	glColor3f(1,1,0);
	glNormal3f(0,0,1);
	for(int i = 0; i < m_pointPath.size(); ++i)
	{
		glVertex3fv(m_pointPath[i].point.m_floats);
	}
	glEnd();
}

void pathPlan::drawRangeFan()
{
	btVector3 cc = m_CS->getCenterPoint() + btVector3(0,0,0.1);
	radarFan(cc.m_floats,m_range);
}

void pathPlan::drawPathBuildLine()
{
	int i;
	static bool x = true;
	if(x)glLineStipple(1,0xFF);
	else glLineStipple(1,0xFF00);
	x = !x;
	glLineWidth(6.0);
	glNormal3f(0,0,1);
	glColor3f(m_color.redF(),m_color.greenF(),m_color.blueF());
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINE_STRIP);										// draw the dotted line path
	for(i = 0; i < m_trailPath.size(); i++)
		glVertex3fv(m_trailPath[i].point.m_floats);
	for(i = 0; i < m_GP.points.size(); i++)
		glVertex3fv(m_GP.points[i].point.m_floats);
	glEnd();
	glDisable(GL_LINE_STIPPLE);
	
	glPointSize(8.0);
	glNormal3f(0,0,1);
	if(x)glColor3f(0,0,0);
	else glColor3f(1,1,1);
	glBegin(GL_POINTS);
	for(i = 0; i < m_minimaList.size(); i++)
		glVertex3fv(m_minimaList[i].point.m_floats);
	glEnd();
}

void pathPlan::drawPathBaseLine()			// draws the line on the base of the shortest path in m_color color
{
	glLineWidth(1.0);
	glNormal3f(0,0,1);
	glColor3f(m_color.redF(),m_color.greenF(),m_color.blueF());
	glBegin(GL_LINE_STRIP);
	for(int i = 0; i < m_GP.points.size(); i++)
	{
		glVertex3fv(m_GP.points[i].point.m_floats);
	}
	glEnd();
}

void pathPlan::drawLightTrail()				// draws the path with a verticle haze
{
 	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUAD_STRIP);
	glNormal3f(0,0,1);
	for(int i = 0; i < m_GP.points.size(); ++i)
	{	
		btVector3 t = m_GP.points[i].point;
		glColor4f(m_color.redF(),m_color.greenF(),m_color.blueF(),m_color.alphaF());
		glVertex3f(t.x(),t.y(),t.z());
		glColor4f(0,0,0,0);
		glVertex3f(t.x(),t.y(),t.z()+1.5);
	}
	glEnd();
	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);
}

void pathPlan::renderGLObject()
{
	for(int i = 0; i < m_displayList.size(); ++i)
	{
		m_displayList[i]->Execute();
	}
}