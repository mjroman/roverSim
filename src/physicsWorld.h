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
#include "utility/definitions.h"
#include "utility/structures.h"

#include <LinearMath/btVector3.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btTransform.h>

#include <BulletCollision/CollisionShapes/btShapeHull.h>

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
private:
    bool                m_idle;
    bool                m_draw;
    btVector3           m_worldSize;
    float               m_worldBoundary;

protected:
	btAlignedObjectArray<ShapeCache*>			m_obstacleCaches;

    btBroadphaseInterface*						m_broadphase;
    btCollisionDispatcher*						m_dispatcher;
    btDynamicsWorld*                            m_dynamicsWorld;
    btConstraintSolver*                         m_solver;
    btDefaultCollisionConfiguration*			m_collisionConfiguration;

    physicsWorld(float x, float y, float z, float boundary);

    static physicsWorld *m_pWorld;

public:
    float   simTimeStep;
    float   simFixedTimeStep;
    int     simSubSteps;

    static physicsWorld *initialize(float x, float y, float z, float boundary) {
        if(!m_pWorld){
            m_pWorld = new physicsWorld(x, y, z, boundary);
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
    bool isIdle() const { return m_idle; }
	void toggleIdle() { m_idle = !m_idle; }
	void idle() { m_idle = true; }
	
    bool canDraw() const { return m_draw; }
	void setDraw(bool d) { m_draw = d; }
    void setWorldSize(btVector3 xyz);
    btVector3 worldSize() { return m_worldSize; }
    float worldBoundary() { return m_worldBoundary; }

    void resetBroadphaseSolver();
    void setGravity(btVector3 gv);
    
    void simulatStep();
    void resetWorld();

	void hullCache(btConvexShape* shape,btAlignedObjectArray<ShapeCache*>* cacheArray);
    btCollisionShape* createShape(int shapeTyp, btVector3 lwh);
	void createHullObstacleShape(btVector3* pts,int numPoints);

    btRigidBody* createRigidBody(float mass, btTransform trans, btCollisionShape* cShape);
    btRigidBody* placeShapeAt(btCollisionShape* bodyShape, btVector3 pos, float yaw, float massval);
};

#endif // PHYSICSWORLD_H
