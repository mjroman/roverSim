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
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <LinearMath/btDefaultMotionState.h>

physicsWorld *physicsWorld::m_pWorld = 0;

physicsWorld::physicsWorld(float x, float y, float z, float boundary)
:
m_idle(false),
m_draw(false),
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
	deleteObstacleGroup();
	
    delete m_dynamicsWorld;
    delete m_solver;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_collisionConfiguration;
}

void physicsWorld::deleteObstacleGroup()
{
	int i = m_obstacleObjects.size();
	m_draw = false; // do not draw
    m_idle = true; // pause simulation
	
	while(i>0){
		btCollisionObject* obj = m_obstacleObjects[i-1];
		m_dynamicsWorld->removeCollisionObject(obj);
		m_obstacleObjects.pop_back();
		i = m_obstacleObjects.size();
	}

	m_obstacleShapes.clear();
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
	deleteObstacleGroup();
	
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

btRigidBody* physicsWorld::createRigidBody(float mass, btTransform trans, btCollisionShape* cShape)
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
		
	// add the box to the world
	m_dynamicsWorld->addRigidBody(body);
	return body;
}

// Places the last shape created in the physics world with mass and a yaw angle
btRigidBody* physicsWorld::placeShapeAt(btCollisionShape* bodyShape, btVector3 pos,float yaw, float massval)
{
    // set the position of the box
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(pos);
    startTransform.setRotation(btQuaternion(0,0,DEGTORAD(yaw)));
	
    return createRigidBody(massval,startTransform,bodyShape);
}

/////////////////////////////////////////
// Obstacle rigid body creation functions
/////////////
void physicsWorld::createObstacleShape(int shapeTyp, btVector3 lwh)
{
    // adds the new shape to the obstacle array
    m_obstacleShapes.push_back(createShape(shapeTyp,lwh));
}
void physicsWorld::placeObstacleShapeAt(btVector3 pos,float yaw, float massval)
{
    // get the last collision shape that has been added to the shape array
	btRigidBody* body = placeShapeAt(m_obstacleShapes[m_obstacleShapes.size()-1],pos,yaw,massval);
	body->setDamping(0.5,0.75);
    m_obstacleObjects.push_back(body);
}





