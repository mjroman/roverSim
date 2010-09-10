#include "SimDomElement.h"
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include "../pathPlan.h"

SimDomElement::SimDomElement()
{
}

/////////////////////////////////////////
// XML node creation methods
/////////////
QDomElement SimDomElement::vectorToNode(QDomDocument &doc, const btVector3 v)
{
	QDomElement vector = doc.createElement( "vector" );
	
	vector.setAttribute( "X", QString::number(v.x(),'g',12));
	vector.setAttribute( "Y", QString::number(v.y(),'g',12));
	vector.setAttribute( "Z", QString::number(v.z(),'g',12));
	return vector;
}
QDomElement SimDomElement::basisToNode(QDomDocument &doc, const btMatrix3x3 mx)
{
	QDomElement matrix = doc.createElement( "matrix" );
	
	for(int i=0;i<3;i++)
		matrix.appendChild(vectorToNode(doc, mx.getRow(i)));
	return matrix;
}
QDomElement SimDomElement::transformToNode(QDomDocument &doc, const btTransform t)
{
	QDomElement trans = doc.createElement( "transform" );
	
	QDomElement origin = doc.createElement( "origin" );
	origin.appendChild(vectorToNode(doc, t.getOrigin()));
	trans.appendChild(origin);
	
	QDomElement rotate = doc.createElement( "rotation" );
	rotate.appendChild(basisToNode(doc, t.getBasis()));
	trans.appendChild(rotate);
	return trans;
}
QDomElement SimDomElement::rigidBodyToNode(QDomDocument &doc, const btRigidBody* body)
{
	QDomElement rigidBody = doc.createElement( "rigidBody" );
	
	rigidBody.setAttribute( "shape", QString::number(body->getCollisionShape()->getShapeType()));
	rigidBody.setAttribute( "mass", QString::number(1.0/body->getInvMass(),'g',10));
	
	QDomElement size = doc.createElement( "size" );
	const btBoxShape* boxShape = static_cast<const btBoxShape*>(body->getCollisionShape());
	size.appendChild(vectorToNode(doc, boxShape->getHalfExtentsWithMargin()));
	
	rigidBody.appendChild(size);
	rigidBody.appendChild(transformToNode(doc, body->getWorldTransform()));
	return rigidBody;
}

QDomElement SimDomElement::colorToNode(QDomDocument &doc, const QColor c)
{
	QDomElement color = doc.createElement( "color" );
	
	color.setAttribute( "Red", QString::number(c.redF()));
	color.setAttribute( "Green", QString::number(c.greenF()));
	color.setAttribute( "Blue", QString::number(c.blueF()));
	color.setAttribute( "Alpha", QString::number(c.alphaF()));
	return color;
}

QDomElement SimDomElement::pathToNode(QDomDocument &doc, const pathPlan* p)
{
	QDomElement path = doc.createElement( "path" );
	
	const goalPath *gp = p->getShortestPath();
	
	path.setAttribute( "range", QString::number(p->getRange()));
	path.setAttribute( "length", QString::number(gp->length));
	path.setAttribute( "time", QString::number(gp->time));
	path.setAttribute( "efficiency", QString::number(gp->efficiency));
	path.setAttribute( "state", QString::number(p->getState()));
	
	path.appendChild(colorToNode(doc, p->getColor()));
	
	QDomElement ptPath = doc.createElement( "pointPath" );
	for(int i=0; i<gp->points.size(); i++){
		ptPath.appendChild(vectorToNode(doc, gp->points[i].point));
	}
	
	path.appendChild(ptPath);
	return path;
}

/////////////////////////////////////////
// XML extraction methods
/////////////
btVector3 SimDomElement::elementToVector(QDomElement element)
{
	btVector3 vect;
	
	vect.setX(element.attribute("X").toFloat());
	vect.setY(element.attribute("Y").toFloat());
	vect.setZ(element.attribute("Z").toFloat());
	return vect;
}
btMatrix3x3 SimDomElement::elementToMatrix(QDomElement element)
{
	btVector3 vx,vy,vz;
	QDomElement vect = element.firstChildElement("vector");
	vx = elementToVector(vect);
	vect = vect.nextSiblingElement("vector");
	vy = elementToVector(vect);
	vect = vect.nextSiblingElement("vector");
	vz = elementToVector(vect);
	
	btMatrix3x3 mx(vx.x(),vx.y(),vx.z(),vy.x(),vy.y(),vy.z(),vz.x(),vz.y(),vz.z());
	return mx;
}
btTransform SimDomElement::elementToTransform(QDomElement element)
{
	btTransform trans;
	
	trans.setIdentity();
	QDomElement origin = element.firstChildElement("origin");
	trans.setOrigin(elementToVector(origin.firstChildElement("vector")));
	QDomElement rotate = element.firstChildElement("rotation");
	trans.setBasis(elementToMatrix(rotate.firstChildElement("matrix")));
	
	return trans;
}

