/*
*  physicsWorld.h
*  RoverSimulation
*
*  Created by Matt Roman on 9/28/09.
*  Copyright 2009 University of Oklahoma. All rights reserved.
*
*/

#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

#include <QObject>
#include <QTimer>
#include "utility/definitions.h"
#include "utility/structures.h"

#include <LinearMath/btVector3.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btTransform.h>

#include <BulletCollision/CollisionShapes/btShapeHull.h>

#define	SIMTIMERRATE	10	// # of milliseconds for physics world timer loop
#define BT_EULER_DEFAULT_ZYX   // yaw, pitch, roll about Z, Y, X axis

class	btBroadphaseInterface;
class	btCollisionObject;
class	btCollisionShape;
class	btConvexHullShape;
class	btConvexShape;
class	btCollisionDispatcher;
class	btConstraintSolver;
class	btDynamicsWorld;
class	btRigidBody;
class	btDefaultCollisionConfiguration;

struct ShapeCache
{
	struct Edge { btVector3 n[2];int v[2]; };
	ShapeCache(btConvexShape* s) : m_shapehull(s) {}
	btShapeHull					m_shapehull;
	btAlignedObjectArray<Edge>	m_edges;
};

class physicsWorld : public QObject
{
	Q_OBJECT
protected:
	physicsWorld();
	static physicsWorld *m_pWorld;

public:
	float   simTimeStep;
	float   simFixedTimeStep;
	int     simSubSteps;

	static physicsWorld *initialize() {
		if(!m_pWorld){
			m_pWorld = new physicsWorld();
		}
		return m_pWorld;
	}

	virtual ~physicsWorld();

	static physicsWorld *instance() { return m_pWorld; }
	static physicsWorld *destroy() {
		delete m_pWorld;
		m_pWorld = 0;
		return 0;
	}

	btDynamicsWorld* getDynamicsWorld() { return m_dynamicsWorld; }
	void resetWorld();
	void setGravity(btVector3 gv);

	void hullCache(btConvexShape* shape,btAlignedObjectArray<ShapeCache*>* cacheArray);
	btCollisionShape* createShape(int shapeTyp, btVector3 lwh);

	btRigidBody* createRigidBody(float mass, btTransform trans, btCollisionShape* cShape);
	btRigidBody* placeShapeAt(btCollisionShape* bodyShape, btVector3 pos, float yaw, float massval);
	void createHullObstacleShape(btVector3* pts,int numPoints);
	
	void startSimTimer(int msec = SIMTIMERRATE);
	bool isIdle() { return m_timer->isActive(); }
	
public slots:
	void stopSimTimer();
	void simulateStep();

signals:
	void simCycled();
	
private:
	QTimer										*m_timer;

	btBroadphaseInterface*						m_broadphase;
	btCollisionDispatcher*						m_dispatcher;
	btDynamicsWorld*                            m_dynamicsWorld;
	btConstraintSolver*                         m_solver;
	btDefaultCollisionConfiguration*			m_collisionConfiguration;

	btAlignedObjectArray<ShapeCache*>			m_obstacleCaches;
};

#endif // PHYSICSWORLD_H
