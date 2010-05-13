#ifndef PATHPLAN_H
#define PATHPLAN_H

#include <QList>
#include <OpenGL/gl.h>
#include "simglobject.h"
#include "physicsWorld.h"
#include "utility/structures.h"
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <LinearMath/btVector3.h>

class btCollisionObject;

typedef struct _rankPoint
{
	btCollisionObject* 	object;
	btVector3			point;
	float				rank;
	int					corner;
}rankPoint;

typedef struct _rankLink
{
	rankPoint	first;
	rankPoint	second;
	float		length;
}rankLink;

class pathPlan : public simGLObject
{
private:
	float										m_goalDistance;
	physicsWorld            					*arena;
	btAlignedObjectArray<btCollisionShape*>		m_ghostShapes;
	btAlignedObjectArray<btCollisionObject*>	m_ghostObjects;
	QList<rankPoint>							m_pointPath;
	rankPoint									m_startPoint;
	rankPoint									m_midPoint;
	rankPoint									m_goalPoint;
	int											m_linkCount;
	btVector3 									vertices[8];
	
	QList<rankLink>								m_linkList;
	bool										m_doneBuilding;
	//btVector3	leftP,rightP;
	//btVector3 	testPoint;
	
	void createGhostShape(btCollisionObject* bodyObj);
	btCollisionObject* isRayBlocked(rankPoint from,rankPoint to);
	btCollisionObject* clearToGoal(rankPoint node);
	void getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right);
	bool isPointThroughObject(rankPoint objPoint,rankPoint testPoint);
	QList<rankPoint> angleBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> rangeBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> quickSortRankLessthan(QList<rankPoint> list);
	QList<rankPoint> getVisablePointsFrom(rankPoint here);
	bool isNewPoint(rankPoint pt);
	bool isNewLink(rankLink link);
	
public:
	pathPlan(btVector3 start, btVector3 end, simGLView* glView = NULL);
	~pathPlan();
	
	void deleteGhostGroup();
	void generateCSpace();
	void findPathA();
	void constructRoadMap();
	void renderGLObject();
	
	// struct cspaceRayResultCallback : public btCollisionWorld::RayResultCallback
	// {
	// 	btVector3					m_rayFromWorld;
	// 	btVector3					m_rayToWorld;
	// 	QList<rankPoint>			m_hitList;
	// 	
	// 	cspaceRayResultCallback(btVector3 rayFrom,btVector3 rayTo)
	// 	:m_rayFromWorld(rayFrom),
	// 	m_rayToWorld(rayTo)
	// 	{
	// 		m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
	// 		m_collisionFilterMask = btBroadphaseProxy::SensorTrigger;
	// 	}
	// 	~cspaceRayResultCallback()
	// 	{ m_hitList.clear(); }
	// 	
	// 	virtual btScalar addSingleResult( btCollisionWorld::LocalRayResult& rayResult,bool bNormalInWorldSpace)
	//   	{
	// 		m_closestHitFraction = rayResult.m_hitFraction;
	// 		m_collisionObject = rayResult.m_collisionObject;
	// 		
	// 		rankPoint hit;
	// 		hit.object = m_collisionObject;
	// 		hit.point.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
	// 		hit.rank = rayResult.m_hitFraction;
	// 		hit.corner = -1;
	// 		m_hitList << hit;
	// 		return rayResult.m_hitFraction;
	// 	}
	// };
};
#endif // PATHPLAN_H