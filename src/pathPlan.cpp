#include "pathPlan.h"
#include "utility/glshapes.h"
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>


#define SPACEMARGIN	0.5


pathPlan::pathPlan(btVector3 start, btVector3 end, simGLView* glView)
:
simGLObject(glView),
m_linkCount(0),
m_doneBuilding(false)
{
	vertices[0] = btVector3(1,1,1);
	vertices[1] = btVector3(1,-1,1);
    vertices[2] = btVector3(-1,-1,1);
    vertices[3] = btVector3(-1,1,1);
    vertices[4] = btVector3(1,1,-1);
    vertices[5] = btVector3(1,-1,-1);
    vertices[6] = btVector3(-1,-1,-1);
    vertices[7] = btVector3(-1,1,-1);

	arena = physicsWorld::instance();

	memset(&m_startPoint,0,sizeof(rankPoint));
	memset(&m_midPoint,0,sizeof(rankPoint));
	memset(&m_goalPoint,0,sizeof(rankPoint));
	
	m_startPoint.point = start;
	m_midPoint.point = start;
	m_goalPoint.point = end;
	
	m_pointPath << m_startPoint;
	
	m_goalDistance = start.distance(end);
	
	generateCSpace();
	findPathA();
	//constructRoadMap();
	qDebug("Maping Complete");
	m_doneBuilding = true;
}

pathPlan::~pathPlan()
{
	deleteGhostGroup();
	m_pointPath.clear();
	m_linkList.clear();
}

void pathPlan::deleteGhostGroup()
{
	int i = m_ghostObjects.size();
	arena->setDraw(false); // do not draw
 	arena->idle();// pause simulation
	
	while(i>0){
		btCollisionObject* obj = m_ghostObjects[i-1];
		arena->getDynamicsWorld()->removeCollisionObject(obj);
		m_ghostObjects.pop_back();
		i = m_ghostObjects.size();
	}

	m_ghostShapes.clear();
	arena->resetBroadphaseSolver();
	arena->toggleIdle(); // unpause simulation
	arena->setDraw(true); // draw obstacles
}

/////////////////////////////////////////
// Path creation
/////////////
// what is the difference in finding all the open paths and choosing the one with the lowest angle = miller way
// and building a path from the initial intersected object an back calculating from the extremes? = my way
void pathPlan::constructRoadMap()
{
	int i;
	if(this->clearToGoal(m_midPoint) == NULL){
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
	btCollisionObject *objBlock = this->clearToGoal(m_midPoint);
	if(objBlock == NULL){
		m_pointPath << m_goalPoint;
		return true;// no intersection all done
	}
	
	// incase of endless loop
	if(m_linkCount > 100) {
		qDebug("looping");
		return false;
	}
	m_linkCount++;
	
	// testing 
	//this->getExtremes(objBlock,m_midPoint,&leftMost,&rightMost);
	//m_midPoint = leftMost;
	//testPoint = m_midPoint.point;
	
	QList<rankPoint>	prospectPoints = getVisablePointsFrom(m_midPoint);

// remove points that are already in the point path
	prospectPoints = this->prunePointsFrom(prospectPoints);
	if(prospectPoints.size() < 1) return false;
// compute the ranks of the remaining points based on angle to goal from midPoint
	prospectPoints = this->angleBasedRank(prospectPoints, m_midPoint);
// sort the list with the smallest angle to the goal
	prospectPoints = this->quickSortRankLessthan(prospectPoints);

	int i=0;
	while(i < prospectPoints.size())
	{
		// change the midPoint to the lowest rank point
		m_midPoint = prospectPoints[i];
		// add the potential point to the global list
		m_pointPath << m_midPoint;
		
		if(this->findPathA()){
			prospectPoints.clear();
			return true;
		}
		else m_pointPath.removeLast();
		i++;
	}
	return false;
}


// checks the ray between from and to, if it is clear NULL is returned else returns the object blocking
btCollisionObject* pathPlan::isRayBlocked(rankPoint from,rankPoint to)
{
	// check the ray toward the TO point
	btCollisionWorld::ClosestRayResultCallback rayCBto(from.point,to.point);
	rayCBto.m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
	rayCBto.m_collisionFilterMask = btBroadphaseProxy::SensorTrigger;
	arena->getDynamicsWorld()->rayTest(from.point,to.point,rayCBto);
	if(rayCBto.hasHit() && rayCBto.m_collisionObject != to.object && rayCBto.m_collisionObject != from.object)
		return rayCBto.m_collisionObject;
	
	// check the ray toward the FROM point
	// btCollisionWorld::ClosestRayResultCallback rayCBfrom(to.point,from.point);
	// rayCBfrom.m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
	// rayCBfrom.m_collisionFilterMask = btBroadphaseProxy::SensorTrigger;
	// arena->getDynamicsWorld()->rayTest(to.point,from.point,rayCBfrom);	
	// if(rayCBfrom.hasHit() && rayCBfrom.m_collisionObject != from.object) 
	// 	return rayCBfrom.m_collisionObject;
	
	// if ray neither ray has a hit return all clear
	return NULL;
}

// check if the path to the goal is clear
btCollisionObject* pathPlan::clearToGoal(rankPoint node)
{
	btCollisionObject *blockingObj = this->isRayBlocked(node,m_goalPoint);

// check that the goal is not being seen through an obstacle
	if(blockingObj == NULL)
	{
		if(!this->isPointThroughObject(node,m_goalPoint)) return NULL; // no intersection
		else return node.object;
	}
	else return blockingObj;
}

// gets the extreme vertecies of the collision object with reference to the midPoint 
void pathPlan::getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right)
{
	float leftMax = 0;
	float rightMax = 0;
	
	btTransform objTrans = obj->getWorldTransform();
	const btBoxShape* boxShape = static_cast<const btBoxShape*>(obj->getCollisionShape());
	btVector3 halfDims = boxShape->getHalfExtentsWithMargin();

	for(int i = 0; i < 4; ++i)
	{
		btVector3 vert = objTrans(halfDims * vertices[i]);
		vert.setZ(objTrans.getOrigin().z());
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
			}
		}
		// else if - or DOWN then right point
		else{
			if(angle > rightMax){
				rightMax = angle;
				(*right).point = vert;
				(*right).corner = i;
			}
		}
	}

	(*left).object = obj;
	(*right).object = obj;
}

bool pathPlan::isPointThroughObject(rankPoint objPoint,rankPoint testPoint)
{
	// if not on an object return false
	if(objPoint.object == 0) return false;
	// if the test point is already on the object return false
	if(objPoint.object == testPoint.object) return false;
	
	static rankPoint previous;
	rankPoint le,re;
	static btVector3 lv,rv;
	// if the object is the same don't need to recalculate the extremes
	if(previous.object != objPoint.object || previous.corner != objPoint.corner) 
	{
		this->getExtremes(objPoint.object,objPoint,&le,&re);
		lv = le.point - objPoint.point;
		rv = re.point - objPoint.point;
		previous = objPoint;
	}

	btVector3 pv = testPoint.point - objPoint.point;
	// get the cross product to determine if the point is between the edges
	if(lv.cross(pv).z() < 0 && rv.cross(pv).z() > 0) return true;
	else return false;
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

// returns a list of all the visable extremes of C Space obstacles
QList<rankPoint> pathPlan::getVisablePointsFrom(rankPoint here)
{
	int i;
	rankPoint leftMost;
	rankPoint rightMost;
	QList<rankPoint> list;
	
// gather all objects extreme vertices
	for(i = 0; i < m_ghostObjects.size(); ++i)
	{
		// get the far left and right points around the obstacle
		this->getExtremes(m_ghostObjects[i],here,&leftMost,&rightMost);
		list << leftMost << rightMost;
	}

// begin Pruning points from the list	
	// if here is on a C space object remove all the points between the two edges of the current obstacle  
	if(here.object)
	{
		i=0;
		while(i < list.size()){
				//is the point on the other side of the object where ray testing fails
			if(this->isPointThroughObject(here,list[i]))
				list.removeAt(i);
			else i++;
		}
	}

	// remove all the points that are blocked
	i=0;
	while(i < list.size()){
		if(this->isRayBlocked(here,list[i])) 
			list.removeAt(i);
		else i++;	
	}
	
	i=0;

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
// C-Space ghost object creation
/////////////
void pathPlan::generateCSpace()
{
	int i;
	btAlignedObjectArray<btCollisionObject*>* obstArray = arena->getObstacleObjectArray();
	
	deleteGhostGroup();
	// loop through all obstacle rigid bodies
	for(i=0;i<obstArray->size();i++){
		btCollisionObject*	colisObject = obstArray->at(i);

	// test colisobject if rigid body
		if(colisObject->getInternalType() == btCollisionObject::CO_RIGID_BODY)
		{
			// create CSpace
			// check if object is in-active
			if(colisObject->isActive()) continue;
			// check if the obstacle is on the terrain
			if(colisObject->getWorldTransform().getOrigin().z() < 0.0) continue;
			createGhostShape(colisObject);
		}
	}
}
void pathPlan::createGhostShape(btCollisionObject* bodyObj)
{
	// check what axis is up
	btTransform wt = bodyObj->getWorldTransform();
	btVector3 bodyOrigin = wt.getOrigin();
	btVector3 notUpVector;
	btTransform bodyTrans;
	bodyTrans.setIdentity();
	
	// if Z axis is up if the cosine is greater than 45deg
	// if(fabs((wt(btVector3(0,0,1)) - bodyOrigin).dot(btVector3(0,0,1))) > 0.707)
	// 		notUpVector.setValue(1,1,4);
	// 	else if(fabs((wt(btVector3(0,1,0)) - bodyOrigin).dot(btVector3(0,0,1))) > 0.707)// if y axis is up
	// 		notUpVector.setValue(1,4,1);
	// 	else
	// 		notUpVector.setValue(4,1,1);
	
	notUpVector.setValue(1,1,4);
	
	// get the shape of the object to draw a C-Space box around it
	btCollisionShape* colisShape = bodyObj->getCollisionShape();
	btVector3 halfDims;
	switch(colisShape->getShapeType()){
		case BOX_SHAPE_PROXYTYPE: {
			const btBoxShape* boxShape = static_cast<const btBoxShape*>(colisShape);
			halfDims = boxShape->getHalfExtentsWithMargin();
			//bodyTrans = wt;
			bodyTrans.setOrigin(wt.getOrigin());
			float yaw = atan2(wt.getBasis().getColumn(0).y(),wt.getBasis().getColumn(0).x());
			bodyTrans.setRotation(btQuaternion(0,0,yaw));
			break;
		}
		case SPHERE_SHAPE_PROXYTYPE: {
			const btSphereShape* sphereShape = static_cast<const btSphereShape*>(colisShape);
			float radius = sphereShape->getMargin();
			halfDims.setValue(radius,radius,radius);
			// use origin but set Z vector up so ghost obj's are level
			bodyTrans.setOrigin(wt.getOrigin());	
			notUpVector.setValue(1,1,0);// also set the notUpVector to (1,1,0);
			break;
		}
		case CONE_SHAPE_PROXYTYPE: {
			const btConeShape* coneShape = static_cast<const btConeShape*>(colisShape);
			float radius = coneShape->getRadius();
			float height = coneShape->getHeight();
			halfDims.setValue(radius,radius,height/2);
			bodyTrans.setOrigin(wt.getOrigin());	// use origin but set Z vector up so ghost obj's are level
			notUpVector.setValue(1,1,0);// also set the notUpVector to (1,1,0);
			break;
		}
		case CYLINDER_SHAPE_PROXYTYPE: {
			const btCylinderShape* cylShape = static_cast<const btCylinderShape*>(colisShape);
			if(notUpVector.z()) {
				float radius = cylShape->getRadius();
				float height = cylShape->getHalfExtentsWithMargin().y();
				halfDims.setValue(radius,radius,height/2);
			}
			else halfDims = cylShape->getHalfExtentsWithMargin();
			bodyTrans.setOrigin(wt.getOrigin()); 	// use origin but set Z vector up so ghost obj's are level
			notUpVector.setValue(1,1,0);// also set the notUpVector to (1,1,0);
			break;
		}
		default:
		halfDims = btVector3(1,1,1);
		bodyTrans = wt;
		break;
	}
	
	// create c-space object
	btGhostObject* ghostObj = new btGhostObject();
	// place it over the object
	ghostObj->setWorldTransform(bodyTrans);
	// grow the object and set the shape
	btVector3 lwh = halfDims + notUpVector * SPACEMARGIN;
	btCollisionShape* cshape = arena->createShape(BOX_SHAPE_PROXYTYPE, lwh);
	ghostObj->setCollisionShape(cshape);
	ghostObj->setCollisionFlags(btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	
	m_ghostShapes.push_back(cshape);
	m_ghostObjects.push_back(ghostObj);
	
	// add the object to the world
	arena->getDynamicsWorld()->addCollisionObject(ghostObj,btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::SensorTrigger);
}

/////////////////////////////////////////
// Draw the C-space objects and the path's
/////////////
void pathPlan::renderGLObject()
{
	int i;
	// draws the golden outline for the c-space ghost objects
	btScalar	glm[16];

	glColor3f(0.99f,0.82f,0.1f); // golden C-Space
	glLineWidth(1.5);

	for(i = 0; i < m_ghostObjects.size(); ++i)
	{
		m_ghostObjects[i]->getWorldTransform().getOpenGLMatrix(glm);
		const btBoxShape* boxShape = static_cast<const btBoxShape*>(m_ghostObjects[i]->getCollisionShape());
		btVector3 halfDims = boxShape->getHalfExtentsWithMargin();

		glPushMatrix();
		glMultMatrixf(glm);
			wireBox(halfDims.x(),halfDims.y(),halfDims.z());
		glPopMatrix();	
	}
	
// draws the roadmap construction
	if(false){
		if(m_doneBuilding) glColor3f(1,0.6,0);
		else glColor3f(0.1,0.6,0.2);
		//float color;
		glBegin(GL_LINES);
		glNormal3f(0,0,1);
		for(i = 0; i < m_linkList.size(); ++i)
		{
			//color = (float)i/(float)m_linkList.size();
			//glColor3f(0,1-color,0);
			btVector3 a = m_linkList[i].first.point;
			btVector3 b = m_linkList[i].second.point;
			glVertex3f(a.x(),a.y(),a.z());
			glVertex3f(b.x(),b.y(),b.z());
		}
		glEnd();
	}
	
// draws the path
	if(true){
		glColor4f(0,1,0,0.5);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glBegin(GL_QUAD_STRIP);
		glNormal3f(0,0,1);
		for(i = 0; i < m_pointPath.size(); ++i)
		{	
			btVector3 t = m_pointPath[i].point;
			glVertex3f(t.x(),t.y(),t.z());
			glVertex3f(t.x(),t.y(),t.z()+0.25);
		}
		glEnd();
		glDisable(GL_BLEND);
// draws the blue line on the path
		glBegin(GL_LINE_STRIP);
		glColor3f(0,1,1);
		glNormal3f(0,0,1);
		for(i = 0; i < m_pointPath.size(); ++i)
		{
			btVector3 t = m_pointPath[i].point;
			glVertex3f(t.x(),t.y(),t.z());
		}
		glEnd();
	}
	
// draws the crow flies line
	glBegin(GL_LINES);
	glColor3f(0,0,1);
	glNormal3f(0,0,1);
	glVertex3f(m_startPoint.point.x(),m_startPoint.point.y(),m_startPoint.point.z()+0.5);
	glVertex3f(m_goalPoint.point.x(),m_goalPoint.point.y(),m_goalPoint.point.z()+0.5);
	glEnd();

// draws test points and lines	
	// glPointSize(8.0);
	// glBegin(GL_POINTS);
	// glNormal3f(0,0,1);
	// glColor3f(0,0,1);
	// glEnd();
	// 
	// glBegin(GL_LINES);
	// glColor3f(1,1,0);
	// glNormal3f(0,0,1);
	// for(i = 0; i < prospectPoints.size(); ++i)
	// {
	// 	btVector3 v = prospectPoints[i].point;
	// 	btVector3 x = testPoint;
	// 	glVertex3f(x.x(),x.y(),x.z());
	// 	glVertex3f(v.x(),v.y(),v.z());
	// }
	// glEnd();
}