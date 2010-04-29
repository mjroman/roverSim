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

#include <LinearMath/btVector3.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btTransform.h>


#define BT_EULER_DEFAULT_ZYX   // yaw, pitch, roll about Z, Y, X axis

enum physicsGroupType {
    TERRAIN_GROUP,
    ROVER_GROUP,
    OBSTACLE_GROUP
};

class	btBroadphaseInterface;
class	btCollisionShape;
class	btCollisionDispatcher;
class	btConstraintSolver;
class	btDynamicsWorld;
class	btRigidBody;
class	btDefaultCollisionConfiguration;

class physicsWorld : public QObject
{
private:
    bool                m_idle;
    bool                m_draw;
    int                 m_obstacleType;
    int                 m_roverType;
    int                 m_terrainType;
    btVector3           m_worldSize;
    float               m_worldBoundary;

    int *objectGroup(physicsGroupType type);

protected:
    btAlignedObjectArray<btCollisionShape*>     m_obstacleShapes;
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
    bool canDraw() const { return m_draw; }
    void setWorldSize(btVector3 xyz);
    btVector3 worldSize() { return m_worldSize; }
    float worldBoundary() { return m_worldBoundary; }

    void deleteGroup(physicsGroupType groupType);
    void resetBroadphaseSolver();
    void setGravity(btVector3 gv);
    void toggleIdle();
    void simulatStep();
    void resetWorld();

    btCollisionShape* createShape(int shapeTyp, btVector3 lwh);
    void createObstacleShape(int shapeTyp, btVector3 lwh);

    btRigidBody* createRigidBody(float mass, btTransform trans, btCollisionShape* cShape, physicsGroupType groupType);
    btRigidBody* placeShapeAt(btCollisionShape* bodyShape, btVector3 pos, float yaw, float massval, physicsGroupType groupType);
    btRigidBody* placeObstacleShapeAt(btVector3 pos,float yaw, float massval);
};

#endif // PHYSICSWORLD_H
