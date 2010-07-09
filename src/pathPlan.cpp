#include "pathPlan.h"
#include "utility/glshapes.h"
#include "utility/definitions.h"
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>


#define SPACEMARGIN	0.5


pathPlan::pathPlan(btVector3 start, btVector3 end, simGLView* glView)
:
simGLObject(glView),
m_linkCount(0),
m_doneBuilding(false)
{
	m_vertices[0] = btVector3(1,1,1);
	m_vertices[1] = btVector3(-1,1,1);
    m_vertices[2] = btVector3(-1,-1,1);
    m_vertices[3] = btVector3(1,-1,1);
    m_vertices[4] = btVector3(1,1,-1);
    m_vertices[5] = btVector3(1,-1,-1);
    m_vertices[6] = btVector3(-1,-1,-1);
    m_vertices[7] = btVector3(-1,1,-1);

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
	groupOverlapCSpace();
	
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
	for(int i=0; i<m_ghostGroups.size(); i++)
		m_ghostGroups[i].list.clear();
	m_ghostGroups.clear();
	contactPoints.clear();
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

void pathPlan::deleteGhostObject(btCollisionObject* obj)
{
	arena->setDraw(false); // do not draw
 	arena->idle();// pause simulation

	arena->getDynamicsWorld()->removeCollisionObject(obj);
	
	btCollisionShape* shape = obj->getCollisionShape();
	m_ghostShapes.remove(shape);
	m_ghostObjects.remove(obj);
	
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
	if(objBlock == NULL){			// made it to the goal 
		m_pointPath << m_goalPoint;	// add the goal point to the path
		return true;				// no intersection all done
	}
	

	if(m_linkCount > 100) {	// incase of endless loop
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
	
	QList<rankPoint>	prospectPoints = getVisablePointsFrom(m_midPoint);

// remove points that are already in the point path
	prospectPoints = this->prunePointsFrom(prospectPoints);
	if(prospectPoints.isEmpty()) return false;
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
	
	if(rayCBto.hasHit()){
		if(rayCBto.m_collisionObject == to.object) return NULL;	// do not count blocking object that are at either end of the ray 
		if(rayCBto.m_collisionObject == from.object) return NULL;
		overlapGroup* gp = (overlapGroup*)rayCBto.m_collisionObject->getUserPointer();	// check if it has intersected a grouped object
		if(gp){	
			for(int i=0; i<gp->list.size(); i++){
				if(gp->list[i] == from.object || gp->list[i] == to.object) return NULL;	// if the blocking object is in the group ignore it
			}
		}
		return rayCBto.m_collisionObject;
	}
	
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
	QList<btVector3> ptList;
	QList<btCollisionObject*> oList;
	
	btTransform objTrans = obj->getWorldTransform();	// the transform of the object being intersected
		
	// get all the top points of the obj
	overlapGroup* gp = (overlapGroup*)obj->getUserPointer();
	if(gp) oList = gp->list;
	else oList << obj;

	for(int j = 0; j < oList.size(); j++)
	{
		ptList = getTopShapePoints(oList[j]);
		
		for(int i = 0; i < ptList.size(); ++i)
		{
			btVector3 vert = ptList[i]; 
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

		(*left).object = oList[j];
		(*right).object = oList[j];
	}
}

// check if objPoint is on a CSpace object and if the path goes through it
// returns false if objPoint is not on an object or the testPoint is on the same object
bool pathPlan::isPointThroughObject(rankPoint objPoint,rankPoint testPoint)
{
	if(objPoint.object == 0) return false;	// return false if not on an object
	if(objPoint.object == testPoint.object) return false; // if the test point is already on the object return false
	
	static rankPoint previous;
	rankPoint le,re;
	static btVector3 lv,rv;
	// if the object is the same don't need to recalculate the extremes
	if(previous.object != objPoint.object || previous.corner != objPoint.corner) 
	{
		this->getExtremes(objPoint.object,objPoint,&le,&re); // get the extremes from objPoint
		lv = le.point - objPoint.point; // vector to left extreme point
		rv = re.point - objPoint.point; // vector to right extreme point
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
	// get all points from individual objects
	for(i = 0; i < m_ghostObjects.size(); ++i)
	{
		if(m_ghostObjects[i]->getUserPointer()) continue;
		// get the far left and right points around the obstacle
		this->getExtremes(m_ghostObjects[i],here,&leftMost,&rightMost);
		list << leftMost << rightMost;
	}
	// get all points from grouped objects
	for(i = 0; i < m_ghostGroups.size(); i++){
		this->getExtremes(m_ghostGroups[i].list[0],here,&leftMost,&rightMost);
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
		// check if object is in-active
		if(colisObject->isActive()) continue;
		// check if the obstacle is on the terrain
		if(colisObject->getWorldTransform().getOrigin().z() < 0.0) continue;
		// create CSpace
		createGhostShape(colisObject);
	}
}

btCollisionObject* pathPlan::createGhostObject(btCollisionShape* cshape,btTransform bodyTrans)
{
	// create a new C-Space object
	btGhostObject* ghostObj = new btGhostObject();
	// place it over the object
	ghostObj->setWorldTransform(bodyTrans);
	// link the shape to the c-space object
	ghostObj->setCollisionShape(cshape);
	ghostObj->setCollisionFlags(btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	
	m_ghostShapes.push_back(cshape);
	m_ghostObjects.push_back(ghostObj);
	
	// add the object to the world
	arena->getDynamicsWorld()->addCollisionObject(ghostObj,btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::SensorTrigger);
	return (btCollisionObject*)ghostObj;
}

// creates a C-Space hull shape object based on the collision object
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
		case CONVEX_HULL_SHAPE_PROXYTYPE: {
			halfDims = btVector3(1,1,1);
			bodyTrans.setOrigin(wt.getOrigin());
			float yaw = atan2(wt.getBasis().getColumn(0).y(),wt.getBasis().getColumn(0).x());
			bodyTrans.setRotation(btQuaternion(0,0,yaw));
			break;
		}
		default:
			halfDims = btVector3(1,1,1);
			bodyTrans = wt;
		break;
	}
	
	
	// grow the object and set the shape
	btVector3 lwh = halfDims + notUpVector * SPACEMARGIN;
	btCollisionShape* cshape = arena->createShape(BOX_SHAPE_PROXYTYPE, lwh);
	createGhostObject(cshape,bodyTrans); // create c-space object
}

// groups all overlapping CSpace boxes into a list, m_ghostGroups contains a list of all grouped objects
void pathPlan::groupOverlapCSpace()
{
	int i;
	overlapGroup* groupA;
	overlapGroup* groupB;
	// run a simulation step to update all the newly added C-Space ghost shapes
	arena->simulatStep();
	// get the number of objects that are in contact with another
	int totalManifolds = arena->getDynamicsWorld()->getDispatcher()->getNumManifolds();
	
	for(i=0;i<totalManifolds;i++){
		// a manifold holds the two overlapping bodies as well as the intersection points
		btPersistentManifold* contactManifold =  arena->getDynamicsWorld()->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* baseobjA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* baseobjB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		
		// C-Space objects are only GHOST type and must have at least one contact point
		if(baseobjA->getInternalType() == btCollisionObject::CO_GHOST_OBJECT &&
		   baseobjB->getInternalType() == btCollisionObject::CO_GHOST_OBJECT &&
		   contactManifold->getNumContacts() > 0)
		{
			// if the userPointer is set then the base object has already been added to a compound shape, use the new compound shape
			// base objects are not deleted until combining is complete
			// the userPointer of the base object is set to the new compound shape when it is created
			groupA = (overlapGroup*)baseobjA->getUserPointer();
			groupB = (overlapGroup*)baseobjB->getUserPointer();
			if(groupA == NULL && groupB == NULL){
				// add overlapping objects to the list
				overlapGroup gp;
				gp.list << baseobjA << baseobjB;
				gp.index = m_ghostGroups.size();
				m_ghostGroups << gp;
				baseobjA->setUserPointer(&m_ghostGroups.last());
				baseobjB->setUserPointer(&m_ghostGroups.last());
			}
			else{	// one of the objects is already a compound object
				if(groupA == groupB) continue; // make sure they are not pointing to the same object
				if(!groupA){ // objA is a base object add it to the group object
					groupB->list << baseobjA;
					baseobjA->setUserPointer(groupB);
				}
				else if(!groupB){ // objB is a base object add it to the group object
					groupA->list << baseobjB;
					baseobjB->setUserPointer(groupA);
				}
				else{ // both objects are in seperate groups add all objects from objB to objA
					for(int j=0;j<groupB->list.size();j++){
						btCollisionObject* obj = groupB->list[j]; // change all the pointers of the object in group B to point to group A
						obj->setUserPointer(groupA);
					}
					groupA->list << groupB->list;
					m_ghostGroups.removeAt(groupB->index);
				}
			}
		}
	}
}

// combines overlapping objects into one compound object, ray testing doesn't work with compound object FUCK YOU BULLET!
void pathPlan::compoundCSpace()
{
	int i;
	btCollisionObject* objA;
	btCollisionObject* objB;
	btCollisionObject* comboObject;
	QList<btCollisionObject*> oldObjectList;
	btTransform	transA;
	btTransform	transB;
	
	// run a simulation step to update all the newly added C-Space ghost shapes
	arena->simulatStep();
	// get the number of objects that are in contact with another
	int totalManifolds = arena->getDynamicsWorld()->getDispatcher()->getNumManifolds();

	for(i=0;i<totalManifolds;i++){
		// a manifold holds the two overlapping bodies as well as the intersection points
		btPersistentManifold* contactManifold =  arena->getDynamicsWorld()->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* baseobjA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* baseobjB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		
		// C-Space objects are only GHOST type and must have at least one contact point
		if(baseobjA->getInternalType() == btCollisionObject::CO_GHOST_OBJECT &&
		   baseobjB->getInternalType() == btCollisionObject::CO_GHOST_OBJECT &&
		   contactManifold->getNumContacts() > 0)
		{
			transA = baseobjA->getWorldTransform();
			transB = baseobjB->getWorldTransform();
			
			// if the userPointer is set then the base object has already been added to a compound shape, use the new compound shape
			// base objects are not deleted until combining is complete
			// the userPointer of the base object is set to the new compound shape when it is created
			objA = (btCollisionObject*)baseobjA->getUserPointer();
			objB = (btCollisionObject*)baseobjB->getUserPointer();
			if(objA == NULL && objB == NULL){
				// create a compound shape object and add the two base object to it
				btCompoundShape* cshape = new btCompoundShape();
				cshape->addChildShape(transA, baseobjA->getCollisionShape());
				cshape->addChildShape(transB, baseobjB->getCollisionShape());

				comboObject = createGhostObject(cshape,transA);

				baseobjA->setUserPointer(comboObject);
				baseobjB->setUserPointer(comboObject);
				if(!oldObjectList.contains(baseobjA)) oldObjectList << baseobjA; 	// add old object to delete list if it isn't already there
				if(!oldObjectList.contains(baseobjB)) oldObjectList << baseobjB; 	// add old object to delete list
			}
			else{	// one of the objects is already a compound object
				if(objA == objB) continue; // make sure they are not pointing to the same object
				if(!objA){ // objA is a base object add it to the compound object
					
					btCompoundShape* shp = static_cast<btCompoundShape*>(objB->getCollisionShape());	// get the compound shape
					shp->addChildShape(transA,baseobjA->getCollisionShape());						// add the base shape to the compound
					baseobjA->setUserPointer(objB);													// set the user pointer of the base object
					if(!oldObjectList.contains(baseobjA)) oldObjectList << baseobjA;				// add the old object to delete list
				}
				else if(!objB){ // objB is a base object add it to the compound object
					btCompoundShape* shp = static_cast<btCompoundShape*>(objA->getCollisionShape());	
					shp->addChildShape(transB,baseobjB->getCollisionShape());
					baseobjB->setUserPointer(objA);
					if(!oldObjectList.contains(baseobjB)) oldObjectList << baseobjB; 	// add old object to delete list
				}
				else{ // both objects are compound add all shapes from objB to compound objA
					comboObject = objA;
					btCompoundShape* comboShape = static_cast<btCompoundShape*>(comboObject->getCollisionShape());
					btCompoundShape* shapeB = static_cast<btCompoundShape*>(objB->getCollisionShape());
					for(int j=0; j < shapeB->getNumChildShapes();j++){
						comboShape->addChildShape(shapeB->getChildTransform(j),shapeB->getChildShape(j));	// add all shapes from B to the compound
					}
					baseobjB->setUserPointer(comboObject);
					if(!oldObjectList.contains(objB)) oldObjectList << objB; 	// add old compound object to delete list
				}
			}
		}
	}
	
	arena->setDraw(false); // do not draw
 	arena->idle();// pause simulation
	// delete old base ghost objects that have been reshaped
	for(i=0;i<oldObjectList.size();i++) {
		arena->getDynamicsWorld()->removeCollisionObject(oldObjectList[i]);
		m_ghostObjects.remove(oldObjectList[i]);
	}
	arena->resetBroadphaseSolver();
	arena->toggleIdle(); // unpause simulation
	arena->setDraw(true); // draw obstacles
	oldObjectList.clear();
}

// trys to merge CSpace objects together by clipping overlapping portions, doesn't really work well
void pathPlan::mergeCSpace()
{
	// run a simulation step to update all the newly added C-Space ghost shapes
	arena->simulatStep();
	
	int i;
	int shapeChanged;
	btCollisionObject* objA;
	btCollisionObject* objB;
	QList<btCollisionObject*> oldObjectList;
	QList<btVector3> objListA;
	QList<btVector3> objListB;
	btTransform	transA;
	btTransform	transB;
	// get the number of objects that are in contact with another
	int totalManifolds = arena->getDynamicsWorld()->getDispatcher()->getNumManifolds();

	for(i=0;i<totalManifolds;i++){
		// a manifold holds the two overlapping bodies as well as the intersection points
		btPersistentManifold* contactManifold =  arena->getDynamicsWorld()->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* baseobjA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* baseobjB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		
		// C-Space objects are only GHOST type and must have at least one contact point
		if(baseobjA->getInternalType() == btCollisionObject::CO_GHOST_OBJECT &&
		   baseobjB->getInternalType() == btCollisionObject::CO_GHOST_OBJECT &&
		   contactManifold->getNumContacts() > 0)
		{
			transA = baseobjA->getWorldTransform();
			transB = baseobjB->getWorldTransform();
			
			// if the userPointer is set then the base object has already been reshaped, use the new HULL shape
			// base objects are not deleted until reshaping is complete
			// the userPointer of the base object is set to the new HULL shape when it is created
			objA = (btCollisionObject*)baseobjA->getUserPointer();
			if(!objA) objA = baseobjA;
			
			objB = (btCollisionObject*)baseobjB->getUserPointer();
			if(!objB) objB = baseobjB;
			//qDebug("A %f,%f",transA.getOrigin().x(),transA.getOrigin().y());
			//qDebug("B %f,%f",transB.getOrigin().x(),transB.getOrigin().y());
			
			objListA.clear();
			objListB.clear();

			// get the top vertices of the objects, used as 2D representation of objects
			objListA = getTopShapePoints(objA);
			objListB = getTopShapePoints(objB);
			
			// get a new point list for polygon A clipped from B
			// inverseTimes(transB) is used to transform the points in polygonB to polygonA's reference
			QList<btVector3> newListA = clipAfromB(objListA,objListB, transA.inverseTimes(transB) ,&shapeChanged);		
			if(shapeChanged) {
				
				btCollisionObject* modobjA = createGhostHull(transA, newListA);	// create a new hull shape add it to the world
				baseobjA->setUserPointer(modobjA);								
				if(!oldObjectList.contains(objA)) oldObjectList << objA; 		// add old object to delete list if it isn't already there
			}
			
			// get a new point list for polygon B clipped from A
			QList<btVector3> newListB = clipAfromB(objListB,objListA, transB.inverseTimes(transA) ,&shapeChanged);
			if(shapeChanged) {
				
				btCollisionObject* modobjB = createGhostHull(transB, newListB);	// create a new hull shape add it to the world
				baseobjB->setUserPointer(modobjB);
				if(!oldObjectList.contains(objB)) oldObjectList << objB; // add old object to delete list
			}
		}
	}
	
	// delete old base ghost objects that have been reshaped
	for(i=0;i<oldObjectList.size();i++) deleteGhostObject(oldObjectList[i]);
	oldObjectList.clear();
}

// creates a new ghost hull object from a list of points that represent the top outline of the shape, must be convex
btCollisionObject* pathPlan::createGhostHull(btTransform bodyTrans, QList<btVector3> list)
{
	int i;
	
	if(list.isEmpty()) return NULL;
	
	btConvexHullShape* cshape = new btConvexHullShape();
	for(i=0;i<list.size();i++) cshape->addPoint(list[i]);
	// the list only contains the top half of the hull
	// it is duplicated in the negative z direction
	for(i=0;i<list.size();i++){
		list[i].setZ(-list[i].z());
		cshape->addPoint(list[i]);
	}
	return createGhostObject(cshape, bodyTrans);
}

// returns an empty list if lista polygon is inside listb polygon, mod is 0 if lista polygon is unchanged else 1
// clips the intersection part of polygonB from polygonA and returns a list of points for reshaped polygonA
QList<btVector3> pathPlan::clipAfromB(QList<btVector3> lista, QList<btVector3> listb, btTransform transab, int* mod)
{
	int i,nexti;
	bool side;		// holds wether the current index point is inside of polygon in listb
	int j,nextj;
	btVector3 xpoint;
	btVector3 startPoint;
	*mod = 0;
	
	// convert all the points in listb to lista reference frame
	//for(i=0;i<listb.size();i++) listb[i] = transab(listb[i]);
	
	//for(i=0;i<lista.size();i++) qDebug("ptA%d %f,%f",i,lista[i].x(),lista[i].y());
	//for(i=0;i<listb.size();i++) qDebug("ptB%d %f,%f",i,listb[i].x(),listb[i].y());
	
	i=0;
	// look for a point on lista outside of polygon listb
	while(isPointInsidePoly(lista[i],listb)){
		i++;
		// polygon from lista is inside of polygon listb
		if(i==lista.size()) {
			*mod = 1;
			lista.clear();
			return lista;
		}
	}
	
	// save the starting point for exit condition
	startPoint = lista[i];
	//qDebug("start %f,%f",startPoint.x(),startPoint.y());
	side = false;	// always start on the outside of polygon listb
	
	// starting at a point on polygonA outside of polygonB traveling in a CCW (right hand rule) look for intersections
	// with polygonB, SIDE keeps track of the current state of wether the index point is inside of polygonB or not
	// SIDE is used to tell when to delete, replace, or insert points for the new shape
	do{
		if(i == lista.size()-1) nexti = 0;
		else nexti = i+1;
		
		if(isPointInsidePoly(lista[nexti],listb) != side) // only add or replace points if there is a change of side
		{
			*mod = 1;
			// find the intersection point
			for(j=0;j<listb.size();j++){	// loop through all sides of polygon listb
				if(j == listb.size()-1) nextj = 0;
				else nextj = j+1;
				//qDebug("A p%d %f,%f p%d %f,%f",i,lista[i].x(),lista[i].y(),nexti,lista[nexti].x(),lista[nexti].y());
				//qDebug("B p%d %f,%f p%d %f,%f",j,listb[j].x(),listb[j].y(),nextj,listb[nextj].x(),listb[nextj].y());
				if(segmentIntersection(lista[i],lista[nexti],listb[j],listb[nextj],&xpoint) != 0) break;
			}
			
			if(side){	// if current index is inside (side=TRUE) of polygon b
				//qDebug("replaced pnt%d with intersection",i);
				lista.replace(i,xpoint);	// replace the point with the intersection point
				i++;
			}
			else {		// if the current index is outside (side=FALSE) polygon b
				//qDebug("inserted intersection before %d",nexti);
				if(nexti == 0) lista.append(xpoint);	// add new points to the end of the list if we are at the wraparound point
				else lista.insert(nexti,xpoint);		// add the intersection point to the list
				i+=2;
			}
			
			side = !side;
		}
		else if(side){	// if there is no change of side and the current index is still insde polygonB
			lista.removeAt(i);	// remove the current index and move to the next
		}
		else i++; // else keep moving around polygonA
		
		if(i == lista.size()) i=0;
	}while(startPoint != lista[i]);
	
	return lista;
}

// returns TRUE if pt is inside of the CONVEX polygon LS
bool pathPlan::isPointInsidePoly(btVector3 pt,QList<btVector3> ls)
{
	int i,iplus;
	double z;
	btVector3 p1, p2;
	
	// traveling around the polygon point list in a CCW (right hand rule) direction 
	// if z is negative for all sides, pt is inside of convex polygon ls
	for(i=0; i<ls.size(); i++)
	{
		if(i == ls.size()-1) iplus = 0;
		else iplus = i+1;
		
		p1 = ls[i];
		p2 = ls[iplus];
		
		// pt is on the right side z is positive, left is negative
		z = (pt.x() - p1.x())*(p2.y() - p1.y()) - (pt.y() - p1.y())*(p2.x() - p1.x());
		
		if(z >= 0) return false; // if the point is to the right of any side pt is outside the polygon
	}
 	return true;
}

// determins if there is an intersection between two lines represented by point pairs (p1,p2) and (p3,p4)
// returns 0 = no intersection 1 = intersection -1 = intersection is and endpoint of (p3,p4)
int pathPlan::segmentIntersection(btVector3 p1,btVector3 p2,btVector3 p3,btVector3 p4,btVector3* intsect)
{
	double z1,z2;
	float m1,m2,b1,b2;
	float num,dnm;
	int s1,s2;
	
	//qDebug("p1 %f,%f p2 %f,%f",p1.x(),p1.y(),p2.x(),p2.y());
	//qDebug("p3 %f,%f p4 %f,%f",p3.x(),p3.y(),p4.x(),p4.y());
	/////////////////////////////////////////////
	// Quick bounding box rejection check
	if(!(MAXIMUM(p1.x(), p2.x()) > MINIMUM(p3.x(), p4.x()) && 
		 MAXIMUM(p3.x(), p4.x()) > MINIMUM(p1.x(), p2.x()) &&
		 MAXIMUM(p1.y(), p2.y()) > MINIMUM(p3.y(), p4.y()) &&
		 MAXIMUM(p3.y(), p4.y()) > MINIMUM(p1.y(), p2.y())))
	{
		//qDebug("bounding rejection");
		intsect = NULL;
		return 0;
	}
	
	
	// Z1 and Z2 hold where P3 and P4 are respective of the vector from P1->P2
	// positive values indicate the point is to the RIGHT of P1->P2
	// negative values indicate the point is to the LEFT of P1->P2
	// 0 indicates the point is on P1->P2
	if((z1 = ((p3.x() - p1.x())*(p2.y() - p1.y())) - ((p3.y() - p1.y())*(p2.x() - p1.x()))) < 0)
		s1 = -1;
	else if(z1 > 0)
		s1 = 1;
	else
	{
		intsect->setValue(p3.x(),p3.y(),p1.z()); // return the point is on the line, modify z value to height of P1->P2
		return -1;
	}
	
	if((z2 = ((p4.x() - p1.x())*(p2.y() - p1.y())) - ((p4.y() - p1.y())*(p2.x() - p1.x()))) < 0)
		s2 = -1;
	else if(z2 > 0)
		s2 = 1;
	else
	{
		intsect->setValue(p4.x(),p4.y(),p1.z()); // return the point is on the line, modify z value to height of P1->P2
		return -1;
	}
	
	// do they straddle each other?
	if(s1 != s2) // if side 1 and side 2 are different the lines straddle
	{
		// FINDING THE INTERSECTION POINT
		num = p2.y() - p1.y();	// slope calculation rise/run
		dnm = p2.x() - p1.x();
		if(num == 0){
			 m1 = 0;		// horizontal line
			intsect->setY(p1.y());
		}
		else if(dnm == 0){
			 m1 = 1;		// vertical line
			intsect->setX(p1.x());
		}
		else m1 = num/dnm;
		b1 = p1.y() - m1*p1.x();	// find the y-intercept
		
		num = p4.y() - p3.y();
		dnm = p4.x() - p3.x();
		if(num == 0){
			m2 = 0;			// horizontal line
			intsect->setY(p3.y());
		}
		else if(dnm == 0){
			m2 = 1;			// vertical line
			intsect->setX(p3.x());
		}
		else m2 = num/dnm;
		b2 = p3.y() - m2*p3.x();	// find the y-intercept
		
		//qDebug("m1%f b1%f m2%f b2%f",m1,b1,m2,b2);
		if(m1 != 1 && m2 != 1) intsect->setX((b2-b1)/(m1-m2));
		if(m1 != 0 && m2 != 0) 
		{	
			if(m1 != 1) intsect->setY(m1*intsect->x() + b1);
			else intsect->setY(m2*intsect->x() + b2);
		}
		
		intsect->setZ(p1.z());
		//qDebug("intersection %f,%f,%f",intsect->x(),intsect->y(),intsect->z());
		
		// a valid intersection happens only ON the segment defined by P1->P2
		if(intsect->x() >= MINIMUM(p1.x(), p2.x()) && intsect->x() <= MAXIMUM(p1.x(), p2.x()) &&
		intsect->y() >= MINIMUM(p1.y(), p2.y()) && intsect->y() <= MAXIMUM(p1.y(),p2.y())) return 1;
	}
	
	intsect = NULL;
	return 0;
}

// takes and object and returns a list of points that represent the top of its geometric shape, assumes symmetric
QList<btVector3> pathPlan::getTopShapePoints(btCollisionObject* obj)
{
	QList<btVector3> list;
	
	btTransform trans = obj->getWorldTransform();
	btCollisionShape* colisShape = obj->getCollisionShape();

	switch(colisShape->getShapeType()){ 
		case BOX_SHAPE_PROXYTYPE: {
			const btBoxShape* boxShape = static_cast<const btBoxShape*>(colisShape);
			btVector3 halfDims = boxShape->getHalfExtentsWithMargin();
			list << trans(halfDims * m_vertices[0]);
			list << trans(halfDims * m_vertices[1]);
			list << trans(halfDims * m_vertices[2]);
			list << trans(halfDims * m_vertices[3]);
			break;
		}
		case CONVEX_HULL_SHAPE_PROXYTYPE: {
			const btConvexHullShape* hullShape = static_cast<const btConvexHullShape*>(colisShape);
			const btVector3* ptlist = hullShape->getUnscaledPoints();
			int numPts = hullShape->getNumPoints()/2;
			for(int i = 0; i < numPts; ++i)
			{
				list << trans(ptlist[i]);
			}
			break;
		}
		// case COMPOUND_SHAPE_PROXYTYPE: {
		// 			btCompoundShape* comboShape = static_cast<btCompoundShape*>(colisShape);
		// 			for(int i=0; i < comboShape->getNumChildShapes(); i++){
		// 				list << getTopShapePoints(comboShape->getChildShape(i));
		// 			}
		// 		}
	}
	return list;
}
/////////////////////////////////////////
// Draw the C-space objects and the path's
/////////////
void pathPlan::renderGLObject()
{
	int i;
// draws the golden outline for the c-space ghost objects
	btScalar	glm[16];

	glLineWidth(1.);
	glDisable(GL_LIGHTING);

	for(i = 0; i < m_ghostObjects.size(); ++i)
	{
		m_ghostObjects[i]->getWorldTransform().getOpenGLMatrix(glm);
		btCollisionShape* colisShape = m_ghostObjects[i]->getCollisionShape();
		
		switch(colisShape->getShapeType()){ 
			case BOX_SHAPE_PROXYTYPE: {
				glColor3f(0.99f,0.82f,0.1f); // golden C-Space
				if(m_ghostObjects[i]->getUserPointer()) glColor3f(0.8f,1,0.5);
				const btBoxShape* boxShape = static_cast<const btBoxShape*>(colisShape);
				btVector3 halfDims = boxShape->getHalfExtentsWithMargin();

				glPushMatrix();
				glMultMatrixf(glm);
				wireBox(halfDims.x(),halfDims.y(),halfDims.z());
				glPopMatrix();	
				break;
			}
			case CONVEX_HULL_SHAPE_PROXYTYPE: {
				glColor3f(0.99f,0.82f,0.5f);
				const btConvexHullShape* hullShape = static_cast<const btConvexHullShape*>(colisShape);
				const btVector3* ptlist = hullShape->getUnscaledPoints();
				int numPts = hullShape->getNumPoints();
				
				Vertex verts[numPts];
				for(int j=0;j<numPts;j++)
				{
					verts[j].x = ptlist[j].x();
					verts[j].y = ptlist[j].y();
					verts[j].z = ptlist[j].z();
				}
				
				glPushMatrix();
				glMultMatrixf(glm);
				wireSymmetricHull(verts,numPts);
				glPopMatrix();
				break;
			}
			case COMPOUND_SHAPE_PROXYTYPE: {
				glColor3f(0.f,1.f,0.f); //C-Space
				const btCompoundShape* shp = static_cast<const btCompoundShape*>(colisShape);
				for(int j=0; j<shp->getNumChildShapes(); j++){
					shp->getChildTransform(j).getOpenGLMatrix(glm);
					const btBoxShape* boxShape = static_cast<const btBoxShape*>(shp->getChildShape(j));
					btVector3 halfDims = boxShape->getHalfExtentsWithMargin();

					glPushMatrix();
					glMultMatrixf(glm);
					wireBox(halfDims.x(),halfDims.y(),halfDims.z());
					glPopMatrix();
				}
			}
			default:
			break;
		}
	}
	glEnable(GL_LIGHTING);
	
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
		
// draws the blue line on the path
		glBegin(GL_LINE_STRIP);
		glColor3f(0.4,0.9,0.93);
		glNormal3f(0,0,1);
		for(i = 0; i < m_pointPath.size(); ++i)
		{
			glVertex3fv(m_pointPath[i].point.m_floats);
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
	glPointSize(8.0);
	glBegin(GL_LINES);
	glNormal3f(1,1,1);
	glColor3f(0,1,0);
	for(i = 0; i < contactPoints.size(); ++i)
	{
		glVertex3fv(m_midPoint.point.m_floats);
		glVertex3fv(contactPoints[i].point.m_floats);
	}
	
	glEnd();
}