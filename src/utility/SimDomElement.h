#ifndef SIMDOMELEMENT_H
#define SIMDOMELEMENT_H

#include <QDomElement>
#include "structures.h"

class btVector3;
class btMatrix3x3;
class btTransform;
class btRigidBody;
class pathPlan;

class SimDomElement : public QDomElement
{
public:
	SimDomElement();
	// Bullet physics
	static QDomElement vectorToNode(QDomDocument &doc, const btVector3 v);
	static QDomElement basisToNode(QDomDocument &doc, const btMatrix3x3 mx);
	static QDomElement transformToNode(QDomDocument &doc, const btTransform t);
	static QDomElement rigidBodyToNode(QDomDocument &doc, const btRigidBody* body);

	static QDomElement colorToNode(QDomDocument &doc, const QColor c);
	// path plan
	static QDomElement pathToNode(QDomDocument &doc, const pathPlan* path);
	
	static btVector3 elementToVector(QDomElement element);
	static btMatrix3x3 elementToMatrix(QDomElement element);
	static btTransform elementToTransform(QDomElement element);
	
	static QColor elementToColor(QDomElement element);
	static void elementToPath(QDomElement element, pathPlan* path);
};
#endif //SIMDOMELEMENT_H