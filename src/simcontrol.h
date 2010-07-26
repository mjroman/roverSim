#ifndef SIMCONTROL_H
#define	SIMCONTROL_H

#include <QTimer>
#include "physicsWorld.h"
#include "utility/structures.h"
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <QTextStream>

class terrain;
class SR2rover;
class skydome;
class simGLView;
class autoCode;
class cSpace;
class pathPlan;


class simControl : public QObject
{
    Q_OBJECT
	private:
		physicsWorld    *arena;
		terrain         *ground;
		skydome         *sky;
		SR2rover		*sr2;
		autoCode		*autoNav;
		cSpace			*configSpace;
		pathPlan		*path;
		QTimer          *simTimer;
	    double          delTime;
		simGLView		*glView;
		QList<WayPoint>	waypointList;
		pickValue		m_pickingObject;
		
		bool fileParser(QTextStream* stream, QString word, void* value);
		
	public:
		// obstacle members
		int				m_obstType;
		int				m_obstCount;
		float			m_dropHeight;
		btVector3		m_minObstSize;
		btVector3		m_maxObstSize;
		float			m_minObstYaw;
		float			m_maxObstYaw;
		float			m_obstDensity;
		
		simControl(simGLView* vw=0);
		~simControl();
		
		// simulation control functions
		void setGravity(btVector3 g);
		void startSimTimer(int msec);
		void stopSimTimer();
		
		void stepTimevals(float tStep,float fixedtStep,int subSteps);
		void pauseSim();
	
		// obstacles setting functions
		//void setObstacleData(int param, char *data);
	
		// terrain control functions
		terrain* getGround() { return ground; }
		void rescaleGround(btVector3 scale);
		
		// rover control functions
		SR2rover* getRover() { return sr2; }
		int	parameterRover(quint8 p);
		bool removeRover();
		
		// autonomous rover control object
		autoCode* getAutoNav() { return autoNav; }
		
		// waypoint functions
		QList<WayPoint>* getWaypointList() { return &waypointList; }
		void addWaypointAt(int uuid, float x,float y, WPstate st=WPstateNew, WPscience sc=WPscienceNone, int i = -1);
		
	public slots:
	// obstacle control functions
		void generateObstacles();
		void saveObstacles(QString filename);
		void loadObstacles(QString filename);
		void removeObstacles();
		void pickObstacle(btVector3 camPos,btVector3 mousePos);
		void moveObstacle(btVector3 camPos,btVector3 mousePos);
		void dropObstacle();
		void spinObstacle(float spin);
		void loftObstacle(float loft);
		void orientObstacle();
		void stepSim();
		
		void openNewGround(QString filename);
		void flattenGround();
		void setWaypointGroundHeight();
		void addWaypointAt(WayPoint wp, int index);
		void editWaypoint(int index);
		void resetWaypointStates();
		
		void generatePath();
		void testPath();
		
		void newRover(QWidget* parent);
		void showNavTool();
		
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
#endif  //SIMCONTROL_H