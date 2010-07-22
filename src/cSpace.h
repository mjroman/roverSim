#ifndef CSPACE_H
#define CSPACE_H

#include <QList>
#include "simglobject.h"
#include "physicsWorld.h"

// typedef struct _overlapGroup
// {
// 	int index;
// 	QList<btCollisionObject*> list;
// }overlapGroup;

class cSpace : public simGLObject
{
public:
	struct overlapGroup
	{
		int index;
		QList<btCollisionObject*> list;
	};
	
private:
	physicsWorld								*arena;
	btVector3									m_centerPoint;
	float										m_detectRange;
	float										m_detectRangeSq;
	btAlignedObjectArray<btCollisionShape*>		m_ghostShapes;
	QList<overlapGroup>							m_ghostGroups;
	btVector3 									m_vertices[8];
	
	void deleteGhostObject(btCollisionObject* obj);
	btCollisionObject* createGhostObject(btCollisionShape* cshape,btTransform bodyTrans);
 	btCollisionObject* createGhostShape(btCollisionObject* bodyObj);
	btCollisionObject* createGhostHull(btTransform bodyTrans, QList<btVector3> list);
	btCollisionObject* createGhostRangeClipHull(btTransform bodyTrans, QList<btVector3> ls);
	QList<btVector3> clipAfromB(QList<btVector3> lista, QList<btVector3> listb, btTransform transab, int* mod=NULL);
	
public:
	btAlignedObjectArray<btCollisionObject*>	m_ghostObjects;
	
	cSpace(btVector3 center, float range, simGLView* glView = NULL);
	~cSpace();
	
	bool isPointInsidePoly(btVector3 pt,QList<btVector3> ls);
	bool isPointInsideObject(btVector3 pt, btCollisionObject* obj);
	int segmentIntersection(btVector3 p1,btVector3 p2,btVector3 p3,btVector3 p4,btVector3* intsec);
	int arcIntersection(btVector3 cc, float rad, btVector3 p1, btVector3 p2, btVector3* intsect1, btVector3* intsect2);
	QList<btVector3> getTopShapePoints(btCollisionObject* obj);
	QList<btVector3> getVerticalOutlinePoints(btCollisionObject* obj);
	
	void setCenterPoint(btVector3 center) { m_centerPoint = center; }
	void setDetectRange(float range) { m_detectRange = range; m_detectRangeSq = range*range;}
	void deleteGhostGroup();
	void generateCSpace();
	void groupOverlapCSpace();
	void compoundCSpace();
	void mergeCSpace();
	void renderGLObject();
};
#endif // CSPACE_H