#include "cSpace.h"
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

cSpace::cSpace(btVector3 center, float range, simGLView* glView)
:
simGLObject(glView),
m_centerPoint(center),
m_detectRange(range),
m_detectRangeSq(range*range)
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
	
	drawCspace(false);
	
	generateCSpace();
	groupOverlapCSpace();
}

cSpace::~cSpace()
{
	deleteGhostGroup();
}

void cSpace::deleteGhostGroup()
{
	arena->setDraw(false); 					// do not draw
 	arena->idle();							// pause simulation
	
	for(int i=0;i<m_ghostObjects.size();i++ )
		arena->getDynamicsWorld()->removeCollisionObject(m_ghostObjects[i]);
		
	for(int i=0;i<m_ghostShapes.size();i++)
		delete m_ghostShapes[i];
	
	for(int i=0; i<m_ghostGroups.size(); i++)
		m_ghostGroups[i].list.clear();
		
	m_ghostShapes.clear();
	m_ghostObjects.clear();
	m_ghostGroups.clear();
	
	arena->resetBroadphaseSolver();			
	arena->toggleIdle(); 					// unpause simulation
	arena->setDraw(true); 					// draw obstacles
}

void cSpace::deleteGhostObject(btCollisionObject* obj)
{
	arena->setDraw(false); // do not draw
 	arena->idle();// pause simulation
	
	arena->getDynamicsWorld()->removeCollisionObject(obj);
	
	btCollisionShape* shape = obj->getCollisionShape();
	delete shape;
	m_ghostShapes.removeAll(shape);
	m_ghostObjects.removeAll(obj);
	// should probably remove the pointer in the group list but this function is not used right now
	arena->resetBroadphaseSolver();
	arena->toggleIdle(); // unpause simulation
	arena->setDraw(true); // draw obstacles
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// C-Space ghost object creation
/////////////
void cSpace::generateCSpace()
{
	int i;
	QList<btVector3> top;
	btVector3 cc = m_centerPoint * btVector3(1,1,0);
	btAlignedObjectArray<btCollisionObject*>* obstArray = arena->getObstacleObjectArray();
	
	deleteGhostGroup();
	
	// create CSpace by using the obstacle shape and growing it by SPACEMARGIN or about the rover's radius
	for(i=0;i<obstArray->size();i++){											// loop through all obstacle rigid bodies
		btCollisionObject* colisObject = obstArray->at(i);
		
		if(colisObject->isActive()) continue;									// check if object is in-active
	
		if(colisObject->getWorldTransform().getOrigin().z() < 0.0) continue;	// check if the obstacle is on the terrain

		top.clear();
		top = getVerticalOutlinePoints(colisObject);						// get the vertical projected outline of the obstacle

		int count = top.size();
		if(m_detectRange > 0)												// for a limited sensor range C-Space make sure the objects are in range
		{			
			for(int j=0; j < top.size(); j++){								// check if it is in-range, out of range, or intersects the range arc
				if(cc.distance2(top[j]) > m_detectRangeSq) count--;
			}
		}
		if(count == 0) continue;											// the whole obstacle is out of range

		btVector3 grow;
		btTransform	trans;
		trans.setIdentity();
		trans.setOrigin(colisObject->getWorldTransform().getOrigin());

		for(int j=0; j < top.size(); j++) 									
		{
			top[j].setZ(trans.getOrigin().z());								// set z position to the height of the obstacle
			top[j] = trans.invXform(top[j]);								// inverse transform from world frame to local object frame
			grow = top[j].normalized() * SPACEMARGIN;						// calculate the growth vector
			grow.setZ(0);													// remove the z components otherwise creates a lopsided object
			top[j] += grow;													// grow the obstacle vertex
		}

		if(m_detectRange <= 0) createGhostHull(trans,top);					// Global view contains all obstacles
		else if(count == top.size()) createGhostHull(trans,top); 			// the whole obstacle is within range
		else createGhostRangeClipHull(trans,top);							// part of the obstacle intersects the arc

	}
}

btCollisionObject* cSpace::createGhostObject(btCollisionShape* cshape,btTransform bodyTrans)
{
	btGhostObject* ghostObj = new btGhostObject();									// create a new C-Space object
	ghostObj->setWorldTransform(bodyTrans);											// place it over the object
	ghostObj->setCollisionShape(cshape);											// link the shape to the c-space object
	ghostObj->setCollisionFlags(btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	
	m_ghostShapes << cshape;
	m_ghostObjects << ghostObj;
	
	// add the object to the world
	arena->getDynamicsWorld()->addCollisionObject(ghostObj,btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::SensorTrigger);
	return static_cast<btCollisionObject*>(ghostObj);
}

// creates a new ghost hull object from a list of points that represent the top outline of the shape, must be convex
btCollisionObject* cSpace::createGhostHull(btTransform bodyTrans, QList<btVector3> list)
{
	int i;
	
	if(list.isEmpty()) return NULL;
	
	btConvexHullShape* cshape = new btConvexHullShape();
	for(i=0;i<list.size();i++) cshape->addPoint(list[i] + btVector3(0,0,2));	// the list only contains the outline of the hull
	for(i=0;i<list.size();i++) cshape->addPoint(list[i] + btVector3(0,0,-2));	// it is duplicated in the negative z direction
	return createGhostObject(cshape, bodyTrans);
}

// creates a new ghost hull object from a list of points but clips the hull where it intersects the range arc
// returns NULL if the polygon list is out of range
btCollisionObject* cSpace::createGhostRangeClipHull(btTransform bodyTrans, QList<btVector3> ls)
{
	int i=0;
	int nexti;
	int arcType;
	btVector3 cc = m_centerPoint;
	btVector3 xpoint1,xpoint2;
	btVector3 startCorner;
	bool side;
	bool nextSide;
	
	cc.setZ(bodyTrans.getOrigin().z());
	cc = bodyTrans.invXform(cc);
	
	i=0;
	while(cc.distance2(ls[i]) > m_detectRangeSq) // look for a point on ls inside the range
	{
		i++;
		if(i == ls.size()) return NULL;
	}
	startCorner = ls[i];	// save the starting point for exit condition
	side = true;			// always start in range
	
	// starting at a point on the polygon from ls inside the range arc traveling in a CCW (right hand rule) look for intersections
	// with the arc, SIDE keeps track of the current state of wether the index point is inside the arc or not
	// SIDE is used to tell when to delete, replace, or insert points for the new shape
	do{
		if(i == ls.size()-1) nexti = 0;
		else nexti = i+1;
		
		nextSide = (cc.distance2(ls[nexti]) > m_detectRangeSq) ? false : true;
		if(nextSide != side){							// only add or replace points if there is a change of side
			// find the intersection point
			arcType = arcIntersection(cc, m_detectRange, ls[i], ls[nexti], &xpoint1, &xpoint2);
			xpoint1.setZ(ls[0].z());
			xpoint2.setZ(ls[0].z());
			
			if(arcType == 2) xpoint1 = xpoint2;
			if(arcType == 0){
				if(side) side = !side;				// skip over a point if its at the end of a segment that intersects the arc
				i++;
			}
			else if(!side){								// if current index is out of range (side=FALSE)
				ls.replace(i,xpoint1);				// replace the point with the intersection point
				i++;
			}
			else {										// if the current index is out of range (side=FALSE)
				if(nexti == 0) ls.append(xpoint1);	// add new point to the end of the ls if we are at the wraparound point
				else ls.insert(nexti,xpoint1);		// add the intersection point to the ls
				i+=2;
			}
			side = !side;
		}
		else if(!side){									// if there is no change of side and the current index is still out of range
			ls.removeAt(i);							// remove the current index and move to the next
		}
		else i++; 										// else keep moving around polygon
		 
		if(i == ls.size()) i=0;
	}while(startCorner != ls[i]);
	
	return createGhostHull(bodyTrans,ls);
}

// groups all overlapping CSpace shapes into a list, m_ghostGroups contains a list of all grouped objects
// grouped objects are independant of the shape they contain, the objects are simply put into lists if 
// they are detected to intersect or overlap
void cSpace::groupOverlapCSpace()
{
	int i;
	overlapGroup* groupA;
	overlapGroup* groupB;

	arena->simulatStep();		// run a simulation step to update all the newly added C-Space ghost shapes
	
	int totalManifolds = arena->getDynamicsWorld()->getDispatcher()->getNumManifolds();	// get the number of objects that are in contact with another
	
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
			groupA = static_cast<cSpace::overlapGroup*>(baseobjA->getUserPointer());
			groupB = static_cast<cSpace::overlapGroup*>(baseobjB->getUserPointer());
			if(groupA == NULL && groupB == NULL){						// add new overlapping objects to the list
				
				overlapGroup gp;
				gp.list << baseobjA << baseobjB;
				gp.index = m_ghostGroups.size();
				m_ghostGroups << gp;
				baseobjA->setUserPointer(&m_ghostGroups.last());
				baseobjB->setUserPointer(&m_ghostGroups.last());
			}
			else{														// one of the objects is already a compound object
				if(groupA == groupB) continue; 							// make sure they are not pointing to the same object
				if(groupA == NULL){ 									// objA is a base object add it to the group object
					groupB->list << baseobjA;
					baseobjA->setUserPointer(groupB);
				}
				else if(groupB == NULL){ 								// objB is a base object add it to the group object
					groupA->list << baseobjB;
					baseobjB->setUserPointer(groupA);
				}
				else{ 													// both objects are in seperate groups add all objects from objB to objA
					for(int j=0;j<groupB->list.size();j++){
						btCollisionObject* obj = groupB->list[j]; 		// change all the pointers of the object in group B to point to group A
						obj->setUserPointer(groupA);
					}
					groupA->list << groupB->list;
					m_ghostGroups.removeAt(groupB->index);
					for(int j=groupB->index; j<m_ghostGroups.size();j++)
					{
						m_ghostGroups[j].index = j;
					}
				}
			}
		}
	}
}

/////////////////////////////////////////
// UTILITY FUNCTIONS
/////////////
// returns TRUE if pt is inside of the CONVEX polygon LS
bool cSpace::isPointInsidePoly(btVector3 pt,QList<btVector3> ls)
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

// returns TRUE if pt is inside of the top projection of the CONVEX object
bool cSpace::isPointInsideObject(btVector3 pt, btCollisionObject* obj)
{
	QList<btVector3> list = this->getTopShapePoints(obj);
	return isPointInsidePoly(pt,list);
}

// determins if there is an intersection between two lines represented by point pairs (p1,p2) and (p3,p4)
// returns 0 = no intersection 1 = intersection -1 = intersection is an endpoint of (p3,p4)
int cSpace::segmentIntersection(btVector3 p1,btVector3 p2,btVector3 p3,btVector3 p4,btVector3* intsect)
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

// determins if there is an intersection between a line segment p1-p2 and a circle with center cc and radius rad
// returns 0 if no intersection
// returns 1 if the segment is tangent or only intersects once at point intsect1
// returns 2 if the segment is intersected at point intsect2
// returns 3 if the segment intersects twice at point intsect1 and intsect2
int cSpace::arcIntersection(btVector3 cc, float rad, btVector3 p1, btVector3 p2, btVector3* intsect1, btVector3* intsect2)
{
	int sign;
	int type;
	float discrim;
	float dr,dx,dy,D;

	p1 = p1-cc;
	p2 = p2-cc;
	
	dx = p2.x() - p1.x();
	dy = p2.y() - p1.y();
	dr = sqrt(SQ(dx) + SQ(dy));
	D = p1.x()*p2.y() - p2.x()*p1.y();
	
	if(dy < 0) sign = -1;
	else sign = 1;
	
	discrim = SQ(rad)*(SQ(dx) + SQ(dy)) - SQ(D);
	type = 0;
	if(discrim >= 0){	// avoid imaginary numbers from lines that don't intersect the arc	
		//qDebug("p1(%f,%f) p2(%f,%f)",p1.x(),p1.y(),p2.x(),p2.y());
		intsect1->setX(((D*dy+sign*dx*sqrt(discrim))/SQ(dr)));
		intsect1->setY(((-D*dx+fabs(dy)*sqrt(discrim))/SQ(dr)));
		intsect1->setZ(cc.z());
		
		intsect2->setX(((D*dy-sign*dx*sqrt(discrim))/SQ(dr)));
		intsect2->setY(((-D*dx-fabs(dy)*sqrt(discrim))/SQ(dr)));
		intsect2->setZ(cc.z());
		
		
		// check the first point if it is between the line segment ends
		if(intsect1->x() >= MINIMUM(p1.x(),p2.x()) && 
		   intsect1->x() <= MAXIMUM(p1.x(),p2.x()) &&
		   intsect1->y() >= MINIMUM(p1.y(),p2.y()) &&
		   intsect1->y() <= MAXIMUM(p1.y(),p2.y()))
		{
			*intsect1 += cc;
			type = 1;							// the first point is inbetween the line segment ends
			if(discrim == 0) return type;		// the line is tangent
		}
		
		// check the second point if it is between the line segment ends
		if(intsect2->x() >= MINIMUM(p1.x(),p2.x()) &&
		   intsect2->x() <= MAXIMUM(p1.x(),p2.x()) &&
		   intsect2->y() >= MINIMUM(p1.y(),p2.y()) &&
		   intsect2->y() <= MAXIMUM(p1.y(),p2.y())) 
		{
			*intsect2 += cc;
			if(type) type = 3;				// the line segment crosses at two points between the line segment
			else							// the second point is between the line segment ends
			{ 
				type = 2;
			}
		}
		//qDebug("typ %d x1(%f,%f) x2(%f,%f)",type,intsect1->x(),intsect1->y(),intsect2->x(),intsect2->y());
	}
	return type;
}

// takes an object and returns a list of points that represent the top of its geometric shape in world coordinates, assumes symmetric
QList<btVector3> cSpace::getTopShapePoints(btCollisionObject* obj)
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
	}
	return list;
}

// returns a list of points that make up the vertical projected outline of the obstacle in world coordinates
// the vector list returned is a convex polygon where overlapping vertices are removed
QList<btVector3> cSpace::getVerticalOutlinePoints(btCollisionObject* obj)
{
	int i;
	QList<btVector3> list;
	btVector3 halfDims;

	btTransform trans = obj->getWorldTransform();
	btCollisionShape* colisShape = obj->getCollisionShape();

	switch(colisShape->getShapeType()){ 
		case BOX_SHAPE_PROXYTYPE: {
			const btBoxShape* boxShape = static_cast<const btBoxShape*>(colisShape);
			halfDims = boxShape->getHalfExtentsWithMargin();
			for(int i=0; i<8; i++)
			{
				list << trans(halfDims * m_vertices[i]) * btVector3(1,1,0);	// flatten to 2D
			}
			break;
		}
		case SPHERE_SHAPE_PROXYTYPE: {
			const btSphereShape* sphereShape = static_cast<const btSphereShape*>(colisShape);
			float radius = sphereShape->getMargin();
			halfDims.setValue(radius,radius,radius);
			for(int i=0; i<8; i++)
			{
				list << trans(halfDims * m_vertices[i]) * btVector3(1,1,0);	// flatten to 2D
			}
			break;
		}
		case CONE_SHAPE_PROXYTYPE: {
			const btConeShape* coneShape = static_cast<const btConeShape*>(colisShape);
			float radius = coneShape->getRadius();
			float height = coneShape->getHeight();
			halfDims.setValue(radius,radius,height/2);
			for(int i=0; i<8; i++)
			{
				list << trans(halfDims * m_vertices[i]) * btVector3(1,1,0);	// flatten to 2D
			}
			break;
		}
		case CYLINDER_SHAPE_PROXYTYPE: {
			const btCylinderShape* cylShape = static_cast<const btCylinderShape*>(colisShape);
			halfDims = cylShape->getHalfExtentsWithMargin();
			for(int i=0; i<8; i++)
			{
				list << trans(halfDims * m_vertices[i]) * btVector3(1,1,0);	// flatten to 2D
			}
			break;
		}
		case CONVEX_HULL_SHAPE_PROXYTYPE: {
			const btConvexHullShape* hullShape = static_cast<const btConvexHullShape*>(colisShape);
			const btVector3* ptlist = hullShape->getUnscaledPoints();
			for(int i = 0; i < hullShape->getNumPoints(); ++i)
			{
				list << trans(ptlist[i]) * btVector3(1,1,0);	// flatten to 2D
			}
			break;
		}
	}
	
	i=0;
	// remove any points that overlap
	while(i<list.size()){
		int j=i+1;
		while(j<list.size()){
			if(list[i].distance2(list[j]) < 0.01) list.removeAt(j);
			else j++;
		}
		i++;
	}
	
	// get only outter perimeter of points by using Jarvis's March
	QList<btVector3> outline;
	
	int index = 0;
	for(i=1;i<list.size();i++){						// find the lower left point in the list
		if(list[i].y() < list[index].y()) index = i;
		else if(list[i].y() == list[index].y())
			 if(list[i].x() < list[index].x()) index = i;
	}
	
	float z;
	int current;
	btVector3 p0,pi,pc;
		
	do{
		outline << list[index];
		p0 = list[index];
		if(index == list.size()-1) current = 0;
		else current = index + 1;
		
		for(i=0;i<list.size();i++){
			if(i == index || i == current) continue;
			pc = list[current];
			pi = list[i];
			z = (pi.x() - p0.x())*(pc.y() - p0.y()) - (pi.y() - p0.y())*(pc.x() - p0.x());
			if(z > 0) current = i;
		}
		index = current;
	}while(list[index] != outline.first());
	
	return outline;
}

/////////////////////////////////////////
// Drawing the C-space objects 
/////////////
void cSpace::renderGLObject()
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
				glColor3f(0.99f,0.82f,0.1f); // golden C-Space
				if(m_ghostObjects[i]->getUserPointer()) glColor3f(0.8f,1,0.5);
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
}

void cSpace::drawCspace(bool x)
{
	if(x)
		m_view->registerGLObject(this);
	else
		m_view->unregisterGLObject(this);
}



/////////////////////////////////////////
// un-used functions 
/////////////
// creates a C-Space hull shape object based on the collision object
btCollisionObject* cSpace::createGhostShape(btCollisionObject* bodyObj)
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
	return createGhostObject(cshape,bodyTrans); // create c-space object
}

// combines overlapping objects into one compound object, ray testing doesn't work with compound object FUCK YOU BULLET!
void cSpace::compoundCSpace()
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
		m_ghostObjects.removeOne(oldObjectList[i]);
	}
	arena->resetBroadphaseSolver();
	arena->toggleIdle(); // unpause simulation
	arena->setDraw(true); // draw obstacles
	oldObjectList.clear();
}
// trys to merge CSpace objects together by clipping overlapping portions, doesn't really work well
void cSpace::mergeCSpace()
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
// returns an empty list if lista polygon is inside listb polygon, mod is 0 if lista polygon is unchanged else 1
// clips the intersection part of polygonB from polygonA and returns a list of points for reshaped polygonA
QList<btVector3> cSpace::clipAfromB(QList<btVector3> lista, QList<btVector3> listb, btTransform transab, int* mod)
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

