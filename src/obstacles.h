#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "physicsWorld.h"
#include "simglobject.h"
#include "tools/obstacletool.h"
#include "utility/definitions.h"
#include "utility/structures.h"
#include <QtGui>
#include <QSettings>
#include <QDomNode>
#include <QTextStream>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

class terrain;
class btRigidBody;
class btCollisionShape;
class btCollisionWorld;

class obstacles : public simGLObject
{
	Q_OBJECT
	public:		
		obstacles(terrain* gnd,simGLView* glView=0);
		~obstacles();
		
		QList<btCollisionObject*>* getObstacles() { return &m_obstacleObjects; }
		bool isSaved() { return m_saved; }
		void showTool() { oTool->show(); }
		void hideTool() { oTool->hide(); }
		void renderGLObject();
		
	public slots:
		void eliminate();
		void generate();
		void saveLayout(QString filename = NULL);
		void loadLayout();
		
		// pick and place functions
		void pickObstacle(btVector3 camPos, btVector3 mousePos);
		void moveObstacle(btVector3 camPos,btVector3 mousePos);
		void dropObstacle();
		void spinObstacle(float spin);
		void loftObstacle(float loft);
		void orientObstacle();
		
	signals:
		void obstaclesRemoved();
		void obstaclesRegenerated();
		
	private:
		QList<btCollisionShape*>     		m_obstacleShapes;
		QList<btCollisionObject*>			m_obstacleObjects;
		
		physicsWorld    					*arena;
		terrain         					*ground;
		obstacleTool						*oTool;
		bool								m_saved;
		
		pickValue							m_pickingObject;
		QSettings 							m_obstSettings;
		
		// rigid body creation
		btCollisionShape* createObstacleShape(int shapeType, btVector3& lwh, float& vol);
		btRigidBody* createObstacleObject(float mass, btCollisionShape* cShape, btTransform trans);
		
		void elementToObstacle(QDomElement element);
	
	public:
		// redefintion of ray reslut callback to exclude the object that is being picked
		// without this funny things happen to the motion of the object as it is move across the screen
		struct notMeRayResultCallback : public btCollisionWorld::RayResultCallback
		{
			btVector3					m_rayFromWorld;
			btVector3					m_rayToWorld;
			btVector3       			m_hitNormalWorld;
			btVector3       			m_hitPointWorld;
			//QList<rankPoint>			m_hitList;	// could contain all points intersected by ray
			btCollisionObject*			m_me;

			notMeRayResultCallback(btVector3 rayFrom,btVector3 rayTo,btCollisionObject* mine = NULL)
				:m_rayFromWorld(rayFrom),
				m_rayToWorld(rayTo),
				m_me(mine)
			{
				//m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
				//m_collisionFilterMask = btBroadphaseProxy::SensorTrigger;
			}
			//~cspaceRayResultCallback();
				//{ m_hitList.clear(); }

			virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
			{
				if (rayResult.m_collisionObject == m_me)
					return 1.0;

				m_closestHitFraction = rayResult.m_hitFraction;
				m_collisionObject = rayResult.m_collisionObject;

				// rankPoint hit;
				// hit.object = m_collisionObject;
				// hit.point.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
				// hit.rank = rayResult.m_hitFraction;
				// hit.corner = -1;
				// m_hitList << hit;
				if (normalInWorldSpace)
				{
					m_hitNormalWorld = rayResult.m_hitNormalLocal;
				} 
				else
				{
					m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
				}
				m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
				return rayResult.m_hitFraction;
			}
		};
};
#endif //OBSTACLES_H