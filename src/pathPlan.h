#ifndef PATHPLAN_H
#define PATHPLAN_H

#include <QList>
#include "simglobject.h"

class btCollisionObject;
class cSpace;

typedef struct _rankPoint
{
	btCollisionObject* 	object;
	btVector3			point;
	float				rank;
	int					corner;
}rankPoint;

// typedef struct _rankLink
// {
// 	rankPoint	first;
// 	rankPoint	second;
// 	float		length;
// }rankLink;


class pathPlan : public simGLObject
{	
private:
	cSpace*										m_CS;
	float										m_goalDistance;
	btVector3									m_colorPath;
	
	QList<rankPoint>							m_pointPath;
	rankPoint									m_startPoint;
	rankPoint									m_midPoint;
	rankPoint									m_goalPoint;
	
	btCollisionObject*							m_goalOccluded;
	int											m_linkCount;
	
	// debugging
	int											m_linkViewIndex;
	QList<rankPoint>							contactPoints;
	QList<btVector3> 							hitPoints;
	
	btCollisionObject* isRayBlocked(rankPoint from,rankPoint to, btVector3* point = NULL);
	void getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right);
	QList<rankPoint> angleBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> progressAngleBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> rangeBasedRank(QList<rankPoint> list, rankPoint pivotPoint);
	QList<rankPoint> quickSortRankLessthan(QList<rankPoint> list);
	QList<rankPoint> getVisablePointsFrom(rankPoint here);
	QList<rankPoint> prunePointsFrom(QList<rankPoint> list);
	QList<rankPoint> smoothPath();
	//bool isNewPoint(rankPoint pt);
	//bool isNewLink(rankLink link);
	
public:
	pathPlan(btVector3 start, btVector3 end, cSpace* space, simGLView* glView = NULL);
	~pathPlan();
	
	void setColor(btVector3 color) { m_colorPath = color; m_colorPath.m_floats[3] = 0.45; }
	bool findPathA();
	void togglePathPoint();
	void renderGLObject();
};
#endif // PATHPLAN_H