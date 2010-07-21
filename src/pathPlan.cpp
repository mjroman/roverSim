#include "pathPlan.h"
#include "utility/definitions.h"
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

pathPlan::pathPlan(btVector3 start, btVector3 end, simGLView* glView)
:
cSpace(glView),
m_goalOccluded(NULL),
m_linkCount(0),
m_linkViewIndex(0),
m_doneBuilding(false)
{	
	memset(&m_startPoint,0,sizeof(rankPoint));
	memset(&m_midPoint,0,sizeof(rankPoint));
	memset(&m_goalPoint,0,sizeof(rankPoint));
	
	m_startPoint.point = start;
	m_midPoint.point = start;
	m_goalPoint.point = end;
	
	m_pointPath << m_startPoint;
	
	m_goalDistance = start.distance(end);
	
	for(int i=0;i<m_ghostObjects.size();i++)
	{
		// check if goal point is inside of a cspace object
		if(isPointInsideObject(m_goalPoint.point,m_ghostObjects[i]))
			m_goalOccluded = m_ghostObjects[i];
	}
	
	if(findPathA())
		qDebug("Mapping Complete");
	else
		qDebug("Incomplete map due to looping or local minima");
	m_doneBuilding = true;
}

pathPlan::~pathPlan()
{
	m_pointPath.clear();
	m_linkList.clear();
	contactPoints.clear();
	hitPoints.clear();
}

/////////////////////////////////////////
// Path creation
/////////////
// what is the difference in finding all the open paths and choosing the one with the lowest angle = miller way
// and building a path from the initial intersected object an back calculating from the extremes? = my way
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
}

/////////////////////////////////////////
// first road map algorithm
/////////////
bool pathPlan::findPathA()
{
	btCollisionObject *objBlock = this->isRayBlocked(m_midPoint,m_goalPoint);
	if(objBlock == NULL || objBlock == m_goalOccluded){		// check that the goal point is not occluded by the blocking object
		m_pointPath << m_goalPoint;							// add the goal point to the path
		
		return true;										// no intersection all done
	}
	
	if(m_linkCount > 100) {									// incase of endless loop
		qDebug("looping");
		return false;
	}
	m_linkCount++;
	
	// testing 
	// 	getExtremes(objBlock, m_midPoint, &lMost, &rMost);
	// 	if(!isRayBlocked(m_midPoint,lMost)) contactPoints << lMost;
	// 	if(!isRayBlocked(m_midPoint,rMost)) contactPoints << rMost;
	//contactPoints << lMost << rMost;
	//contactPoints << getVisablePointsFrom(m_midPoint);
	//return true;
	
	QList<rankPoint>	prospectPoints = getVisablePointsFrom(m_midPoint); 	// get all the visable points from the current location

	prospectPoints = this->prunePointsFrom(prospectPoints);					// remove points that are already in the point path
	if(prospectPoints.isEmpty()) return false;
	prospectPoints = this->angleBasedRank(prospectPoints, m_midPoint); 		// compute the ranks based on angle to goal from midPoint
	prospectPoints = this->quickSortRankLessthan(prospectPoints);			// sort the list with the smallest rank first to the goal
	
	int i=0;
	while(i < prospectPoints.size())
	{
		m_midPoint = prospectPoints[i];			// change the midPoint to the lowest rank point
		
		m_pointPath << m_midPoint;				// add the potential point to the global path list
		m_view->updateGL();
		if(this->findPathA()){					// if a path to the goal has been found
			prospectPoints.clear();				// clear all the prospecive points and return true
			return true;
		}
		else m_pointPath.removeLast();			// else remove the last midPoint and try the next
		i++;
	}
	return false;
}

void pathPlan::togglePathPoint()
{
	if(m_linkViewIndex >= m_pointPath.size()) m_linkViewIndex=0;
	
	hitPoints.clear();
	contactPoints.clear();
	
	m_view->getCamera()->cameraSetDirection(m_pointPath[m_linkViewIndex].point); // set the camera view to the path point

	// get All Visable points
		int i;
		rankPoint here = m_pointPath[m_linkViewIndex];
		rankPoint leftMost;
		rankPoint rightMost;	
		QList<rankPoint> list;

	// gather all objects extreme vertices
		for(i = 0; i < m_ghostObjects.size(); ++i)									// get all points from individual objects
		{
			this->getExtremes(m_ghostObjects[i],here,&leftMost,&rightMost);			// get the far left and right points around the obstacle
			if(m_ghostObjects[i]->getUserPointer())	// check both extremes to make sure they are not inside of a grouped object
			{
				bool lState,rState;
				lState = rState = false;
				overlapGroup* gp = static_cast<overlapGroup*>(m_ghostObjects[i]->getUserPointer());
				for(int j=0;j<gp->list.size();j++)
				{
					if(gp->list[j] == m_ghostObjects[i]) continue;				// skip the object the extremes are from
					if(isPointInsideObject(leftMost.point,gp->list[j])) lState = true;
					if(isPointInsideObject(rightMost.point,gp->list[j])) rState = true;
				}
				if(!lState) list << leftMost;
				if(!rState) list << rightMost;
			}
			else list << leftMost << rightMost;
		}

	// begin Pruning points from the list
		if(true){	// remove all remaining points that are blocked
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
		}
		contactPoints << list;
	
	//contactPoints = prunePointsFrom(contactPoints);
		contactPoints = angleBasedRank(contactPoints, m_pointPath[m_linkViewIndex]);
		contactPoints = quickSortRankLessthan(contactPoints);

		qDebug("_______________________");
		for(int i=0;i<contactPoints.size();i++){
			qDebug("rank %f",contactPoints[i].rank);
		}
		
		contactPoints.prepend(m_pointPath[m_linkViewIndex]);
	
	//if(this->isRayBlocked(m_pointPath[m_linkViewIndex],m_goalPoint) == NULL) qDebug("Clear to GOAL");
	m_linkViewIndex++;
}

// checks the ray between from and to, if it is clear NULL is returned else returns the object blocking
//btCollisionObject* pathPlan::isVectorBlocked(rankPoint from,rankPoint to, btVector3* point)
btCollisionObject* pathPlan::isRayBlocked(rankPoint from,rankPoint to, btVector3* point)
{
	QList<rankPoint> hitList;
	
	for(int i=0;i<m_ghostObjects.size();i++){
		int k;
		rankPoint rp;
		rp.object = m_ghostObjects[i];
		
		if(rp.object == to.object && rp.object == from.object) continue; // checking the edge of an object should be ignored
		
		QList<btVector3> shapePoints = this->getTopShapePoints(rp.object);
		for(int j=0;j<shapePoints.size();j++){
			if(j == shapePoints.size()-1) k=0;
			else k=j+1;
			if(this->segmentIntersection(from.point,to.point,shapePoints[j],shapePoints[k],&rp.point) > 0.){
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
		overlapGroup* gp = static_cast<overlapGroup*>(rayCBto.m_collisionObject->getUserPointer());	// check if it has intersected a grouped object
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
	ptList = getTopShapePoints(obj);					// get all the top points of the obj

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
	for(i = 0; i < m_ghostObjects.size(); ++i)									// get all points from individual objects
	{
		this->getExtremes(m_ghostObjects[i],here,&leftMost,&rightMost);			// get the far left and right points around the obstacle
		
		if(m_ghostObjects[i]->getUserPointer())	// check both extremes to make sure they are not inside of a grouped object
		{
			bool lState,rState;
			lState = rState = false;
			overlapGroup* gp = static_cast<overlapGroup*>(m_ghostObjects[i]->getUserPointer());
			for(int j=0;j<gp->list.size();j++)
			{
				if(gp->list[j] == m_ghostObjects[i]) continue;				// skip the object the extremes are from
				if(isPointInsideObject(leftMost.point,gp->list[j])) lState = true;
				if(isPointInsideObject(rightMost.point,gp->list[j])) rState = true;
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

// smooth the path by checking visibility back to start
QList<rankPoint> pathPlan::smoothPath()
{
	// after the point is added check if you can see back to any other points
		// if(m_pointPath.size() > 2)
		// {
		// 	for(i = 0; i < m_pointPath.size()-2; ++i)
		// 	{
		// 		if(this->isRayBlocked(m_pointPath[i],m_midPoint) == NULL){
		// 			m_pointPath.swap(i+1,m_pointPath.size()-1);
		// 			while(m_pointPath.size() > i+2){m_pointPath.removeLast();}
		// 		}
		// 	}
		// }
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
	return list;
}

// computes the rank of each point based on the distance from the pivot point
QList<rankPoint> pathPlan::rangeBasedRank(QList<rankPoint> list, rankPoint pivotPoint)
{
	for(int i = 0; i < list.size(); ++i)
	{
		list[i].rank = list[i].point.distance(pivotPoint.point);
	}
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
}

/////////////////////////////////////////
// Draw the path's
/////////////
void pathPlan::renderGLObject()
{
	int i;

	cSpace::renderGLObject();	// render the C-Space too
	
	glLineWidth(1.);
	
// // draws the roadmap construction
// 	if(m_doneBuilding) glColor3f(1,0.6,0);
// 	else glColor3f(0.1,0.6,0.2);
// 		//float color;
// 	glBegin(GL_LINES);
// 	glNormal3f(0,0,1);
// 	for(i = 0; i < m_linkList.size(); ++i)
// 	{
// 			//color = (float)i/(float)m_linkList.size();
// 			//glColor3f(0,1-color,0);
// 		btVector3 a = m_linkList[i].first.point;
// 		btVector3 b = m_linkList[i].second.point;
// 		glVertex3f(a.x(),a.y(),a.z());
// 		glVertex3f(b.x(),b.y(),b.z());
// 	}
// 	glEnd();
	
	// draws test points and lines	
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
		
// draws the crow flies line
	glBegin(GL_LINES);
	glLineWidth(1.0);
	glColor3f(0,0,1);
	glNormal3f(0,0,1);
	glVertex3fv(m_startPoint.point.m_floats);
	glVertex3fv(m_goalPoint.point.m_floats);
	glEnd();
	
// draws the path
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glBegin(GL_QUAD_STRIP);
	glNormal3f(0,0,1);
	for(i = 0; i < m_pointPath.size(); ++i)
	{	
		btVector3 t = m_pointPath[i].point;
		glColor4f(0.4,0.9,0.93,0.45);
		glVertex3f(t.x(),t.y(),t.z());
		glColor4f(1,1,1,0.0);
		glVertex3f(t.x(),t.y(),t.z()+1.5);
	}
	glEnd();
	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);

// draws the blue line on the base of the path
	glBegin(GL_LINE_STRIP);
	glColor3f(0.4,0.9,0.93);
	glNormal3f(0,0,1);
	for(i = 0; i < m_pointPath.size(); ++i)
	{
		glVertex3fv(m_pointPath[i].point.m_floats);
	}
	glEnd();
}