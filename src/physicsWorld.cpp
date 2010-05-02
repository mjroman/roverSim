/*
 *  physicsWorld.cpp
 *  RoverSimulation
 *
 *  Created by Matt Roman on 9/28/09.
 *  Copyright 2009 University of Oklahoma. All rights reserved.
 *
 */

#include "physicsWorld.h"

// Use with Bullet Frameworks
#include <BulletCollision/btBulletCollisionCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <LinearMath/btDefaultMotionState.h>

#define SPACEMARGIN	0.5

physicsWorld *physicsWorld::m_pWorld = 0;

physicsWorld::physicsWorld(float x, float y, float z, float boundary)
:
m_idle(false),
m_draw(false),
m_ghostType(GHOST_GROUP),
m_obstacleType(OBSTACLE_GROUP),
m_roverType(ROVER_GROUP),
m_terrainType(TERRAIN_GROUP),
m_worldSize(x,y,z),
m_worldBoundary(boundary),
m_dynamicsWorld(0),
simTimeStep(0.1),
simFixedTimeStep(0.001),
simSubSteps(15)
{
    qDebug("Physics startup");

	//collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	
	// Build the broadphase
    //int maxProxies = 5000;
    btVector3 worldAabbMin(-m_worldBoundary,-m_worldBoundary,0);
    btVector3 worldAabbMax(m_worldBoundary+m_worldSize.x(),m_worldBoundary+m_worldSize.y(),m_worldSize.z());
    //m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
    m_broadphase = new btDbvtBroadphase();

	// The actual physics solver
	m_solver = new btSequentialImpulseConstraintSolver();
	
	// The Physics World
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	
    m_dynamicsWorld->setGravity(btVector3(0,0,-9.8));
}

physicsWorld::~physicsWorld()
{
    qDebug("deleting physics");
	
    //remove the rigidbodies from the dynamics world and delete them
	deleteGroup(GHOST_GROUP);
    deleteGroup(OBSTACLE_GROUP);
	
    delete m_dynamicsWorld;
    delete m_solver;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_collisionConfiguration;
}

void physicsWorld::deleteGroup(physicsGroupType groupType)
{
    int i=0;
    m_draw = false; // do not draw
    m_idle = true; // pause simulation
	
    //remove the rigidbodies from the dynamics world and delete them
    while(i < m_dynamicsWorld->getNumCollisionObjects())
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
       
        if(obj && *(int *)obj->getUserPointer() == groupType){
	 		btRigidBody* body = btRigidBody::upcast(obj);
            if(body && body->getMotionState())
                delete body->getMotionState();
            m_dynamicsWorld->removeCollisionObject(obj);
            delete obj;
        }
        else i++;
    }
    //qDebug("collObj:%d notDeleted:%d",i,m_dynamicsWorld->getNumCollisionObjects());
    
    //delete collision shapes arrays
    switch(groupType){
        case TERRAIN_GROUP:
            break;
        case ROVER_GROUP:
            break;
        case OBSTACLE_GROUP:
            m_obstacleShapes.clear();
			break;
		case GHOST_GROUP:
			m_ghostShapes.clear();
            break;
    }
	
    resetBroadphaseSolver();
    
    m_idle = false; // unpause simulation
    m_draw = true; // draw obstacles
}

void physicsWorld::resetBroadphaseSolver()
{
    // reset the broadphase
    m_broadphase->resetPool(m_dispatcher);
    m_solver->reset();
}

// resets the entire world. Stops drawing and simulation
// clears all obstacles and shapes and resets the broadphase
// to the new size and resets the solver.
void physicsWorld::resetWorld()
{
    //remove the rigidbodies from the dynamics world and delete them
	deleteGroup(GHOST_GROUP);
    deleteGroup(OBSTACLE_GROUP);
    deleteGroup(TERRAIN_GROUP);
	
    // Rebuild the broadphase volume
    //int maxProxies = 5000;
    btVector3 worldAabbMin(-m_worldBoundary,-m_worldBoundary,-m_worldBoundary);
    btVector3 worldAabbMax(m_worldBoundary+m_worldSize.x(),m_worldBoundary+m_worldSize.y(),m_worldSize.z());
    // create a new broadphase with the new volume
//    btBroadphaseInterface *nBroadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
    // set the dynamics world to the new broadphase
//    m_dynamicsWorld->setBroadphase(nBroadphase);
	
    // delete the old broadphase
//    delete m_broadphase;
	
    // set the new pointer to the global pointer
//    m_broadphase = nBroadphase;
	
    resetBroadphaseSolver();
	
    qDebug("new world %f,%f,%f",m_worldSize.x(),m_worldSize.y(),m_worldSize.z());
}

void physicsWorld::setWorldSize(btVector3 xyz)
{
    m_worldSize = xyz;
}

// sets the gravity vector in m/s^2
void physicsWorld::setGravity(btVector3 gv)
{
	m_dynamicsWorld->setGravity(gv);
}

// toggles on and off the simulation
void physicsWorld::toggleIdle(){
	if (m_idle) m_idle = false;
	else m_idle = true;
}

// Called to simulate a step in the physics world
void physicsWorld::simulatStep(){
    ///step the simulation
    if (m_dynamicsWorld && !m_idle)
    {
        m_dynamicsWorld->stepSimulation(simTimeStep,simSubSteps,simFixedTimeStep);
    }
}

int *physicsWorld::objectGroup(physicsGroupType type)
{
	switch (type){
		case TERRAIN_GROUP:
		return &m_terrainType;
		case ROVER_GROUP:
		return &m_roverType;
		case OBSTACLE_GROUP:
		return &m_obstacleType;
		case GHOST_GROUP:
		return &m_ghostType;
		default:
		return NULL;
	}
}

// creates a new physics collision shape, this has to be called
// when a new shape is needed or a different size of the previous shape.
// It only needs to be called once if a bunch of identical shapes are needed.
btCollisionShape* physicsWorld::createShape(int shapeTyp, btVector3 lwh)
{
        btCollisionShape* collShape = NULL;
	
	switch (shapeTyp) {
		case BOX_SHAPE_PROXYTYPE:
			collShape = new btBoxShape(lwh);
			break;
		case SPHERE_SHAPE_PROXYTYPE:
			collShape = new btSphereShape(lwh.x());
			break;
		case CONE_SHAPE_PROXYTYPE:
			collShape = new btConeShapeZ(lwh.y(),lwh.x());
			break;
		case CYLINDER_SHAPE_PROXYTYPE:
			collShape = new btCylinderShapeX(lwh);
			break;
	}
	return collShape;
}

void physicsWorld::createObstacleShape(int shapeTyp, btVector3 lwh)
{
    // adds the new shape to the obstacle array
    m_obstacleShapes.push_back(createShape(shapeTyp,lwh));
}

btRigidBody* physicsWorld::createRigidBody(float mass, btTransform trans, btCollisionShape* cShape, physicsGroupType groupType)
{
	// set the mass of the box
	btScalar massval(mass);
	bool isDynamic = (massval != 0.f);
	
	// set inertia of box and calculate the CG
	btVector3 bodyInertia(0,0,0);
	if(isDynamic) cShape->calculateLocalInertia(massval,bodyInertia);
	
	// using motionstate provides interpolation and only synchronizes 'active' objects
	btDefaultMotionState* bodyMotionState = new btDefaultMotionState(trans);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,bodyMotionState,cShape,bodyInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	
	body->setUserPointer(objectGroup(groupType));
	// add the box to the world
	m_dynamicsWorld->addRigidBody(body);
	return body;
}

// Places the last shape created in the physics world with mass and a yaw angle
btRigidBody* physicsWorld::placeShapeAt(btCollisionShape* bodyShape, btVector3 pos,float yaw, float massval, physicsGroupType groupType)
{
    // set the position of the box
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(pos);
    startTransform.setRotation(btQuaternion(DEGTORAD(yaw),0,0));
	
    return createRigidBody(massval,startTransform,bodyShape,groupType);
}

void physicsWorld::placeObstacleShapeAt(btVector3 pos,float yaw, float massval)
{
    // get the last collision shape that has been added to the shape array
    placeShapeAt(m_obstacleShapes[m_obstacleShapes.size()-1],pos,yaw,massval,OBSTACLE_GROUP);
}

void physicsWorld::createGhostShape(btRigidBody* bodyObj)
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
	
	btCollisionShape* colisShape = bodyObj->getCollisionShape();
	btVector3 halfDims;
	
	switch(colisShape->getShapeType()){
		case BOX_SHAPE_PROXYTYPE: {
			const btBoxShape* boxShape = static_cast<const btBoxShape*>(colisShape);
			halfDims = boxShape->getHalfExtentsWithMargin();
			bodyTrans = bodyObj->getWorldTransform();
			break;
		}
		case SPHERE_SHAPE_PROXYTYPE: {
			const btSphereShape* sphereShape = static_cast<const btSphereShape*>(colisShape);
			float radius = sphereShape->getMargin();
			halfDims.setValue(radius,radius,radius);
			// use origin but set Z vector up so ghost obj's are level
			bodyTrans.setOrigin(bodyObj->getWorldTransform().getOrigin());	
			notUpVector.setValue(1,1,0);// also set the notUpVector to (1,1,0);
			break;
		}
		case CONE_SHAPE_PROXYTYPE: {
			const btConeShape* coneShape = static_cast<const btConeShape*>(colisShape);
			float radius = coneShape->getRadius();
			float height = coneShape->getHeight();
			halfDims.setValue(radius,height,radius);
			bodyTrans.setOrigin(bodyObj->getWorldTransform().getOrigin());	// use origin but set Z vector up so ghost obj's are level
			notUpVector.setValue(1,0,1);// also set the notUpVector to (1,1,0);
			break;
		}
		case CYLINDER_SHAPE_PROXYTYPE: {
			const btCylinderShape* cylShape = static_cast<const btCylinderShape*>(colisShape);
			halfDims = cylShape->getHalfExtentsWithMargin();
			bodyTrans.setOrigin(bodyObj->getWorldTransform().getOrigin()); 	// use origin but set Z vector up so ghost obj's are level
			notUpVector.setValue(1,1,0);// also set the notUpVector to (1,1,0);
			break;
		}
		default:
		halfDims = btVector3(1,1,1);
		bodyTrans = bodyObj->getWorldTransform();
		break;
	}
	
	// create c-space object
	btGhostObject* ghostObj = new btGhostObject();
	ghostObj->setWorldTransform(bodyTrans);
	btVector3 lwh = halfDims + notUpVector * SPACEMARGIN;
	btCollisionShape* cshape = createShape(BOX_SHAPE_PROXYTYPE, lwh);
	m_ghostShapes.push_back(cshape);
	ghostObj->setCollisionShape(cshape);
	ghostObj->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	
	// add the object to the world
	ghostObj->setUserPointer(objectGroup(GHOST_GROUP));
	m_dynamicsWorld->addCollisionObject(ghostObj,btBroadphaseProxy::SensorTrigger,btBroadphaseProxy::KinematicFilter);
}





