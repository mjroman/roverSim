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


pathPlan::pathPlan(simGLView* glView)
:
simGLObject(glView)
{
	arena = physicsWorld::instance();
	
	generateCSpace();
}

pathPlan::~pathPlan()
{
	deleteGhostGroup();
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
			createGhostShape(colisObject);
		}
	}
}

/////////////////////////////////////////
// C-Space ghost object creation
/////////////
void pathPlan::createGhostShape(btCollisionObject* bodyObj)
{
	// check what axis is up
	btTransform wt = bodyObj->getWorldTransform();
	btVector3 bodyOrigin = wt.getOrigin();
	btVector3 notUpVector;
	btTransform bodyTrans;
	bodyTrans.setIdentity();
	
	// if Z axis is up if the cosine is greater than 45deg
	if(fabs((wt(btVector3(0,0,1)) - bodyOrigin).dot(btVector3(0,0,1))) > 0.707)
		notUpVector.setValue(1,1,0);
	else if(fabs((wt(btVector3(0,1,0)) - bodyOrigin).dot(btVector3(0,0,1))) > 0.707)// if y axis is up
		notUpVector.setValue(1,0,1);
	else
		notUpVector.setValue(0,1,1);
	
	// get the shape of the object to draw a C-Space box around it
	btCollisionShape* colisShape = bodyObj->getCollisionShape();
	btVector3 halfDims;
	switch(colisShape->getShapeType()){
		case BOX_SHAPE_PROXYTYPE: {
			const btBoxShape* boxShape = static_cast<const btBoxShape*>(colisShape);
			halfDims = boxShape->getHalfExtentsWithMargin();
			bodyTrans = wt;
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
	ghostObj->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	
	m_ghostShapes.push_back(cshape);
	m_ghostObjects.push_back(ghostObj);
	
	// add the object to the world
	arena->getDynamicsWorld()->addCollisionObject(ghostObj,btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::KinematicFilter);
}

void pathPlan::renderGLObject()
{
	// draws the golden outline for the c-space ghost objects
	btScalar	glm[16];

	glColor3f(0.99f,0.82f,0.1f); // golden C-Space
	glLineWidth(1.5);

	for(int i = 0; i < m_ghostObjects.size(); ++i)
	{
		m_ghostObjects[i]->getWorldTransform().getOpenGLMatrix(glm);
		const btBoxShape* boxShape = static_cast<const btBoxShape*>(m_ghostObjects[i]->getCollisionShape());
		btVector3 halfDims = boxShape->getHalfExtentsWithMargin();

		glPushMatrix();
		glMultMatrixf(glm);
			wireBox(halfDims.x(),halfDims.y(),halfDims.z());
		glPopMatrix();	
	}

}