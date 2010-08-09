#include "pathPlan.h"
#include "cSpace.h"
#include "utility/definitions.h"
#include "utility/glshapes.h"
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

pathPlan::pathPlan(btVector3 start, btVector3 end, goalPath gp, simGLView* glView)
:
simGLObject(glView),
m_CS(NULL),
m_displayCS(false),
m_GP(gp),
m_goalOccluded(NULL),
m_linkViewIndex(0)
{
	memset(&m_startPoint,0,sizeof(rankPoint));
	memset(&m_midPoint,0,sizeof(rankPoint));
	memset(&m_goalPoint,0,sizeof(rankPoint));
	
	m_startPoint.point = start;
	m_goalPoint.point = end;
	
	// create callbacks to all the drawing methods, these are added to/removed from the display list
	// make sure these remain in order to match with the enumeration
	ACallback<pathPlan> drawCB(this, &pathPlan::drawDebugPath);	m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawCrowFlyLine); 		m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawSavedPaths);			m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawCurrentSearchPath);	m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawRangeFan);			m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawPathBaseLine);		m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawLightTrail);			m_drawingList << drawCB;

	if(m_GP.saveOn) displaySavedPaths(true);
	
	displayPath(true);										// turn on display of the path
	displayCurrentSearch(true);								// turn on display of current search path
	
	this->generateCspace();
	
	QTime t;
	t.start();												// start time of path calculation
	
	if(m_GP.range == 0) this->findPathA();					// if Gods eye find the shortest path
	else this->cycleToGoal();								// step forward on path until the goal is in range
	
	m_GP.time = t.elapsed();								// get the elapsed time for the path generation
	
	displayCurrentSearch(false);							// turn off search path drawing
	
	m_view->printText("Mapping Complete");
	if(m_GP.saveOn) m_view->printText(QString("%1 paths found").arg(m_pathList.size()));
	if(m_GP.length == 0)
		m_view->printText("No paths to goal found");
	else{
		m_view->printText(QString("Crow Flies:%1   Shortest Length:%2   Efficiency:%3").arg(m_goalDistance).arg(m_GP.length).arg(m_goalDistance/m_GP.length));
		displayCrowFly(true);
		displayLightTrail(true); 							// insert after the baseline of the path has been drawn
	}
	m_view->printText(QString("Search Time: %1 s").arg((float)m_GP.time/1000.0));
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

/////////////////////////////////////////
// Generate a new Configuration Space to plan paths around
/////////////
void pathPlan::generateCspace()
{
	if(m_CS) delete m_CS;
	m_CS = new cSpace(m_startPoint.point,m_GP.range,m_view);					// create a new Configuration Space based on the start point
	
	for(int i=0; i < m_CS->m_ghostObjects.size(); i++)						// check if goal point is inside of a cspace object
	{
		if(m_CS->isPointInsideObject(m_goalPoint.point,m_CS->m_ghostObjects[i]))
			m_goalOccluded = m_CS->m_ghostObjects[i];
	}
	
	m_goalDistance = m_startPoint.point.distance(m_goalPoint.point);		// calculate the distance to the goal from the start point
	m_midPoint = m_startPoint;												// reset all parameters due to new Cspace
	m_pointPath.clear();
	m_pointPath << m_startPoint;
}

// returns true if the goal is within sensor range of the robot
bool pathPlan::isGoalInRange()
{
	if(m_GP.range == 0) return true;
	if(m_GP.range <= m_goalDistance) return false;
	else return true;
}

/////////////////////////////////////////
// Path creation
/////////////
void pathPlan::cycleToGoal()
{
	int i;
	QList<rankPoint> deltList;
	
	while( !this->isGoalInRange() )	// while the goal is not in range
	{
		this->findPathA();			// calculate the path to the goal
		if(m_GP.points.size() <= 1) break;
		deltList << m_startPoint;	// add the current position of the begining of the path
		
		i=1;
		float step = m_GP.step;
		float pdist = m_startPoint.point.distance(m_GP.points[i].point);
		
		while(step > pdist)
		{
			deltList << m_GP.points[i];
			step -= pdist;
			pdist = m_GP.points[i].point.distance(m_GP.points[i+1].point);
			i++;
		}
		
		btVector3 v = m_GP.points[i].point - m_GP.points[i-1].point;	// vector between i and i-1
		m_startPoint.point = step * v.normalized() + m_GP.points[i-1].point;		
		m_GP.length = 0;
		m_GP.points.clear();
		
		this->generateCspace();
	}

	this->findPathA();
	
	m_GP.points = deltList + m_GP.points;									// prepend the delt position list
	m_GP.length = 0;
	for(i=0; i<m_GP.points.size()-1; i++) m_GP.length += m_GP.points[i].point.distance(m_GP.points[i+1].point);
	m_startPoint = m_GP.points[0];
	m_goalDistance = m_startPoint.point.distance(m_goalPoint.point);		// calculate the distance to the goal from the start point
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
// first path planner algorithm DFSearch
/////////////
bool pathPlan::findPathA(float length)
{
	float goalDist = m_midPoint.point.distance(m_goalPoint.point);	// find the distance from the current midpoint to the Goal
	
 	if(m_GP.length != 0 && length + goalDist > m_GP.length) 	// incase the new search path is farther than the shortest path
		return false;
	
	btCollisionObject *objBlock = this->isRayBlocked(m_midPoint,m_goalPoint);	// is the ray blocked?
	
	if(objBlock == NULL || objBlock == m_goalOccluded){							// check that the goal point is not occluded by the blocking object
		m_pointPath << m_goalPoint;												// add the goal point to the path
		length += goalDist;						

		if(length < m_GP.length || m_GP.length == 0)	// check if a new shortest path has been found
		{
			m_GP.length = length;
			m_GP.points = m_pointPath;								// save the new short path
			if(m_GP.saveOn) m_pathList.push_front(m_GP);
		}
		else if(m_GP.saveOn)
		{
			goalPath newPath;
			newPath.length = length;
			newPath.points = m_pointPath;
			newPath.color = btVector3(1,0,1);
			newPath.range = m_GP.range;
			m_pathList.push_back(newPath);
		}
	//	m_view->updateGL();
		return true;															// no intersection all done
	}
	
	QList<rankPoint> prospectPoints = getVisablePointsFrom(m_midPoint); 		// get all the visable points from the current location
	//prospectPoints = this->progressAngleBasedRank(prospectPoints, m_midPoint);	// compute the ranks based on progress angle from start-goal vector
	prospectPoints = this->angleBasedRank(prospectPoints, m_midPoint); 		// compute the ranks based on angle to goal from midPoint
	prospectPoints = this->prunePointsFrom(prospectPoints);						// remove points that are already in the point path
		
	if(prospectPoints.isEmpty()) return false;
	
	int i=0;
	btVector3 oldMidPoint = m_midPoint.point;
	while(i < prospectPoints.size() && (m_GP.breadth == 0 || i < m_GP.breadth))
	{
		float del = oldMidPoint.distance(prospectPoints[i].point);
		m_midPoint = prospectPoints[i];											// change the midPoint to the lowest rank point
		m_pointPath << m_midPoint;												// add the potential point to the global path list

		m_view->updateGL();
		
		if(this->findPathA(length+del)){										// recursive check for a path to the goal
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

void pathPlan::togglePathPoint()
{
	if(m_GP.points.isEmpty()) return;
	if(m_linkViewIndex >= m_GP.points.size()) m_linkViewIndex=-1;
	
	if(m_linkViewIndex == 0){
		m_displayList.push_front( &m_drawingList[PP_DEBUG] );			// turn on debug drawing
		displayRangeFan(true);
	}
	else if(m_linkViewIndex < 0){
		m_displayList.removeAll( &m_drawingList[PP_DEBUG] );	// turn off debug drawing
		displayRangeFan(false);
		m_linkViewIndex = 0;
		return;
	}
	
	hitPoints.clear();
	contactPoints.clear();
	
	m_view->getCamera()->cameraSetDirection(m_GP.points[m_linkViewIndex].point); // set the camera view to the path point

	// get All Visable points
		int i;
		rankPoint here = m_GP.points[m_linkViewIndex];
		rankPoint leftMost;
		rankPoint rightMost;	
		QList<rankPoint> list;

		if(m_CS) delete m_CS;
		m_CS = new cSpace(here.point,m_GP.range,m_view);					// create a new Configuration Space based on the start point
		m_CS->drawCspace(m_displayCS);
		
	// gather all objects extreme vertices
		for(i = 0; i < m_CS->m_ghostObjects.size(); ++i)									// get all points from individual objects
		{
			this->getExtremes(m_CS->m_ghostObjects[i],here,&leftMost,&rightMost);			// get the far left and right points around the obstacle
			if(m_CS->m_ghostObjects[i]->getUserPointer())	// check both extremes to make sure they are not inside of a grouped object
			{
				bool lState,rState;
				lState = rState = false;
				cSpace::overlapGroup* gp = static_cast<cSpace::overlapGroup*>(m_CS->m_ghostObjects[i]->getUserPointer());
				for(int j=0;j<gp->list.size();j++)
				{
					if(gp->list[j] == m_CS->m_ghostObjects[i]) continue;				// skip the object the extremes are from
					if(m_CS->isPointInsideObject(leftMost.point,gp->list[j])) lState = true;
					if(m_CS->isPointInsideObject(rightMost.point,gp->list[j])) rState = true;
				}
				if(!lState) list << leftMost;
				if(!rState) list << rightMost;
			}
			else list << leftMost << rightMost;
		}

	// begin Pruning points from the list

		btVector3 pt;
		i=0;
		while(i < list.size()){
			if(this->isRayBlocked(here,list[i],&pt)) {
				hitPoints << pt;				
				list.removeAt(i);
			}
			else{
				i++;
			}
		}

		contactPoints << list;
	
	//contactPoints = prunePointsFrom(contactPoints);
		contactPoints = angleBasedRank(contactPoints, m_GP.points[m_linkViewIndex]);
		contactPoints = quickSortRankLessthan(contactPoints);

		qDebug("_______________________");
		for(int i=0;i<contactPoints.size();i++){
			qDebug("rank %f",contactPoints[i].rank);
		}
		
		contactPoints.prepend(m_GP.points[m_linkViewIndex]);
	
	//if(this->isRayBlocked(m_pointPath[m_linkViewIndex],m_goalPoint) == NULL) qDebug("Clear to GOAL");
	m_linkViewIndex++;
}


// checks the ray between from and to, if it is clear NULL is returned else returns the object blocking
//btCollisionObject* pathPlan::isVectorBlocked(rankPoint from,rankPoint to, btVector3* point)
btCollisionObject* pathPlan::isRayBlocked(rankPoint from,rankPoint to, btVector3* point)
{
	QList<rankPoint> hitList;
	
	for(int i=0;i<m_CS->m_ghostObjects.size();i++){
		int k;
		rankPoint rp;
		rp.object = m_CS->m_ghostObjects[i];
		
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

/* OLD BLOCKED RAY CHECKING
// checks the ray between from and to, if it is clear NULL is returned else returns the object blocking
btCollisionObject* pathPlan::isRayBlocked(rankPoint from,rankPoint to, btVector3* point)
{
	// check the ray toward the TO point
	btCollisionWorld::ClosestRayResultCallback rayCBto(from.point,to.point);
	rayCBto.m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
	rayCBto.m_collisionFilterMask = btBroadphaseProxy::SensorTrigger;
	arena->getDynamicsWorld()->rayTest(from.point,to.point,rayCBto);
	
	if(rayCBto.hasHit()){
		if(point) *point = rayCBto.m_hitPointWorld;
		if(rayCBto.m_collisionObject == to.object) return NULL;	// do not count blocking object that are at either end of the ray 
		if(rayCBto.m_collisionObject == from.object) return NULL;
		cSpace::overlapGroup* gp = static_cast<cSpace::overlapGroup*>(rayCBto.m_collisionObject->getUserPointer());	// check if it has intersected a grouped object
		if(gp){	
			for(int i=0; i<gp->list.size(); i++){
				if(gp->list[i] == from.object) return NULL;	// if the blocking object is in the group ignore it
			}
		}
		return rayCBto.m_collisionObject;
	}
	
	// if neither ray has a hit return all clear
	return NULL;
}*/

// gets the extreme vertecies of the collision object with reference to the midPoint 
void pathPlan::getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right)
{
	float leftMax = 0;
	float rightMax = 0;
	QList<btVector3> ptList;
	
	btTransform objTrans = obj->getWorldTransform();	// the transform of the object being intersected
	ptList = m_CS->getTopShapePoints(obj);					// get all the top points of the obj

	for(int i = 0; i < ptList.size(); ++i)	// cycle through all top points
	{
		if(pivotPoint.object == obj && pivotPoint.corner == i) continue;	// ignore the point if it is equal to the pivot point
		
		btVector3 vert = ptList[i]; 
		//vert.setZ(objTrans.getOrigin().z());								// flatten vectors to 2D
		vert.setZ(m_startPoint.point.z());									// set all extremes to start height
		
			// get the cross product to find the direction, left or right side
		btVector3 xc = (objTrans.getOrigin() - pivotPoint.point).cross(vert - pivotPoint.point);
			// get the angle to find the extremes
		float angle = (objTrans.getOrigin() - pivotPoint.point).angle(vert - pivotPoint.point);

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
QList<rankPoint> pathPlan::getVisablePointsFrom(rankPoint here)
{
	int i;
	rankPoint leftMost;
	rankPoint rightMost;
	QList<rankPoint> list;
	
// gather all objects extreme vertices
	for(i = 0; i < m_CS->m_ghostObjects.size(); ++i)									// get all points from individual objects
	{
		this->getExtremes(m_CS->m_ghostObjects[i],here,&leftMost,&rightMost);			// get the far left and right points around the obstacle
		
		if(m_CS->m_ghostObjects[i]->getUserPointer())	// check both extremes to make sure they are not inside of a grouped object
		{
			bool lState,rState;
			lState = rState = false;
			cSpace::overlapGroup* gp = static_cast<cSpace::overlapGroup*>(m_CS->m_ghostObjects[i]->getUserPointer());
			for(int j=0;j<gp->list.size();j++)
			{
				if(gp->list[j] == m_CS->m_ghostObjects[i]) continue;				// skip the object the extremes are from
				if(m_CS->isPointInsideObject(leftMost.point,gp->list[j])) lState = true;
				if(m_CS->isPointInsideObject(rightMost.point,gp->list[j])) rState = true;
			}
			if(!lState) list << leftMost;
			if(!rState) list << rightMost;
		}
		else list << leftMost << rightMost;
	}

// begin Pruning points from the list
	i=0;
	while(i < list.size()){
		if(this->isRayBlocked(here,list[i])) 				// remove all remaining points that are blocked
			list.removeAt(i);
		else
			i++;	
	}

	return list;
}

// removes visible points not wanted
QList<rankPoint> pathPlan::prunePointsFrom(QList<rankPoint> list)
{
	// check each point if it is already on the path
	for(int j = 0; j < m_pointPath.size(); ++j)
	{
		// check all points in the visible list
		for(int i = 0; i < list.size(); ++i)
		{
			if(list[i].object == m_pointPath[j].object && list[i].corner == m_pointPath[j].corner)
			{
				list.removeAt(i);
				break;
			}
		}
	}
	
	// remove all points that are farther from the goal
	// float sqGoalDist = here.point.distance2(m_goalPoint.point);
	// 	while(i < list.size()){
	// 		if(here.point.distance2(list[i].point) > sqGoalDist)
	// 			list.removeAt(i);
	// 		else i++;
	// 	}
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
void pathPlan::displayCrowFly(bool x)
{
	if(x) m_displayList.push_front(&m_drawingList[PP_CROWFLY] );	// insert after the baseline of the path has been drawn
	else m_displayList.removeAll(&m_drawingList[PP_CROWFLY]);
}
void pathPlan::displaySavedPaths(bool x)
{
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
	if(x) m_displayList.push_front( &m_drawingList[PP_BASELINE] );	// make sure the base line of the path is drawn first
	else m_displayList.removeAll(&m_drawingList[PP_BASELINE]);
}
void pathPlan::displayLightTrail(bool x)
{
	if(x) m_displayList.push_back( &m_drawingList[PP_LIGHTTRAIL] );	// insert after the baseline of the path has been drawn
	else m_displayList.removeAll(&m_drawingList[PP_LIGHTTRAIL]);
}
void pathPlan::toggleCspace()
{
	m_displayCS = !m_displayCS;
	if(m_CS) m_CS->drawCspace(m_displayCS);
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

	glPointSize(3.0);
	glColor3f(1,1,1);
	glBegin(GL_POINTS);
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
		glColor3fv(m_pathList[j].color.m_floats);
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
	radarFan(cc.m_floats,m_GP.range);
}

void pathPlan::drawPathBaseLine()			// draws the line on the base of the shortest path in m_colorPath color
{
	glLineWidth(1.0);
	glNormal3f(0,0,1);
	glColor3fv(m_GP.color.m_floats);
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
		glColor4fv(m_GP.color.m_floats);
		glVertex3f(t.x(),t.y(),t.z());
		glColor4f(1,1,1,0.0);
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