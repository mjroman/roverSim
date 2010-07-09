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

typedef struct _overlapGroup
{
	int index;
	QList<btCollisionObject*> list;
}overlapGroup;

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
	QList<overlapGroup>							m_ghostGroups;
	QList<rankPoint>							m_pointPath;
	rankPoint									m_startPoint;
	rankPoint									m_midPoint;
	rankPoint									m_goalPoint;
	int											m_linkCount;
	btVector3 									m_vertices[8];
	
	QList<rankLink>								m_linkList;
	bool										m_doneBuilding;
	rankPoint 	lMost,rMost;
	QList<rankPoint>	contactPoints;
	
	void deleteGhostObject(btCollisionObject* obj);
	btCollisionObject* isRayBlocked(rankPoint from,rankPoint to);
	btCollisionObject* clearToGoal(rankPoint node);
	void getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right);
	bool isPointThroughObject(rankPoint objPoint,rankPoint testPoint);
	QList<rankPoint> angleBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> rangeBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> quickSortRankLessthan(QList<rankPoint> list);
	QList<rankPoint> getVisablePointsFrom(rankPoint here);
	QList<rankPoint> prunePointsFrom(QList<rankPoint> list);
	QList<rankPoint> smoothPath();
	bool isNewPoint(rankPoint pt);
	bool isNewLink(rankLink link);
	
	btCollisionObject* createGhostObject(btCollisionShape* cshape,btTransform bodyTrans);
	void createGhostShape(btCollisionObject* bodyObj);
	btCollisionObject* createGhostHull(btTransform bodyTrans, QList<btVector3> list);
	QList<btVector3> clipAfromB(QList<btVector3> lista, QList<btVector3> listb, btTransform transab, int* mod=NULL);
	bool isPointInsidePoly(btVector3 pt,QList<btVector3> ls);
	int segmentIntersection(btVector3 p1,btVector3 p2,btVector3 p3,btVector3 p4,btVector3* intsec);
	QList<btVector3> getTopShapePoints(btCollisionObject* obj);
	
public:
	pathPlan(btVector3 start, btVector3 end, simGLView* glView = NULL);
	~pathPlan();
	
	void deleteGhostGroup();
	void generateCSpace();
	void groupOverlapCSpace();
	void compoundCSpace();
	void mergeCSpace();
	bool findPathA();
	void constructRoadMap();
	void renderGLObject();
	
	// struct cspaceRayResultCallback : public btCollisionWorld::RayResultCallback
	// 	{
	// 		btVector3					m_rayFromWorld;
	// 		btVector3					m_rayToWorld;
	// 		QList<rankPoint>			m_hitList;
	// 
	// 		cspaceRayResultCallback(btVector3 rayFrom,btVector3 rayTo)
	// 			:m_rayFromWorld(rayFrom),
	// 			m_rayToWorld(rayTo)
	// 		{
	// 			m_collisionFilterGroup = btBroadphaseProxy::SensorTrigger;
	// 			m_collisionFilterMask = btBroadphaseProxy::SensorTrigger;
	// 		}
	// 		~cspaceRayResultCallback()
	// 			{ m_hitList.clear(); }
	// 
	// 		virtual btScalar addSingleResult( btCollisionWorld::LocalRayResult& rayResult,bool btNormalInWorldSpace)
	// 		{
	// 			m_closestHitFraction = rayResult.m_hitFraction;
	// 			m_collisionObject = rayResult.m_collisionObject;
	// 
	// 			rankPoint hit;
	// 			hit.object = m_collisionObject;
	// 			hit.point.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
	// 			hit.rank = rayResult.m_hitFraction;
	// 			hit.corner = -1;
	// 			m_hitList << hit;
	// 			return rayResult.m_hitFraction;
	// 		}
	// 	};
};
#endif // PATHPLAN_H