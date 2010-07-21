#ifndef PATHPLAN_H
#define PATHPLAN_H

#include "cSpace.h"

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

class pathPlan : protected cSpace
{
private:
	float										m_goalDistance;
	QList<rankPoint>							m_pointPath;
	rankPoint									m_startPoint;
	rankPoint									m_midPoint;
	rankPoint									m_goalPoint;
	btCollisionObject*							m_goalOccluded;
	int											m_linkCount;
	int											m_linkViewIndex;
	
	// testing
	QList<rankLink>								m_linkList;
	bool										m_doneBuilding;
	rankPoint 	lMost,rMost;
	QList<rankPoint>	contactPoints;
	QList<btVector3> 	hitPoints;
	
//	btCollisionObject* isVectorBlocked(rankPoint from,rankPoint to, btVector3* point = NULL);
	btCollisionObject* isRayBlocked(rankPoint from,rankPoint to, btVector3* point = NULL);
	void getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right);
	QList<rankPoint> angleBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> rangeBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> quickSortRankLessthan(QList<rankPoint> list);
	QList<rankPoint> getVisablePointsFrom(rankPoint here);
	QList<rankPoint> prunePointsFrom(QList<rankPoint> list);
	QList<rankPoint> smoothPath();
	bool isNewPoint(rankPoint pt);
	bool isNewLink(rankLink link);
	
public:
	pathPlan(btVector3 start, btVector3 end, simGLView* glView = NULL);
	~pathPlan();
	
	bool findPathA();
	void constructRoadMap();
	void togglePathPoint();
	void renderGLObject();
};
#endif // PATHPLAN_H