#ifndef PATHPLAN_H
#define PATHPLAN_H

#include <OpenGL/gl.h>
#include "simglobject.h"
#include "physicsWorld.h"
#include <LinearMath/btVector3.h>

class btCollisionObject;

class pathPlan : public simGLObject
{
private:
	physicsWorld            					*arena;
	btAlignedObjectArray<btCollisionShape*>		m_ghostShapes;
	btAlignedObjectArray<btCollisionObject*>	m_ghostObjects;
	
	void createGhostShape(btCollisionObject* bodyObj);
	
public:
	pathPlan(simGLView* glView = NULL);
	~pathPlan();
	
	void deleteGhostGroup();
	void generateCSpace();
	void renderGLObject();
	
};
#endif // PATHPLAN_H