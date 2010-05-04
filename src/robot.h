#ifndef ROBOT_H
#define ROBOT_H

#include <OpenGL/gl.h>
#include <QString>
#include <QList>
#include "physicsWorld.h"
#include "simglobject.h"
#include "utility/definitions.h"
#include "utility/glshapes.h"

typedef struct _robotView {
	QString		name;
	btVector3	position;
	btVector3	pitchRollYaw;
	float		zoom;
	btVector3	upVect;
}robotView;

typedef struct _robotParts {
	int wheels;
    int passiveJoints;
    int motors;
    int bodyParts;
	int	sensors;
	int views;
}robotParts;

class btRigidBody;
class btGeneric6DofConstraint;
class btTypedConstraint;

class robot : public simGLObject
{
protected: 
	physicsWorld    	*arena;
	robotParts			m_partCount;
	float 				*m_motorAngle;
	float				*m_motorVelocity;
	float				*m_motorImpulse;
	btVector3   		m_wheelFriction;

	void drawFrame(btTransform &tr);
	
public:
	float                                       *m_previousPosition;
    btTypedConstraint                           **m_motorJoints;
    btTypedConstraint                           **m_passiveJoints;
    btVector3                                   *m_bodyAttachPoints;
    btRigidBody                                 **m_bodyParts;
    btAlignedObjectArray<btCollisionShape*>     m_robotShapes;
	btAlignedObjectArray<btCollisionObject*>	m_robotObjects;

	float       		pitch,roll,heading;
	btVector3   		position;
	float				odometer;
	LatLonCoord			GPSposition;
	float				voltage;
	
	//QList<robotView> 	viewList;
	
	robot(simGLView* glView);
	void initalloc(robotParts pn);
    virtual ~robot();
	void deleteRobotGroup();
	
    btTransform getRobotTransform();
    void placeRobotAt(btVector3 here);
	virtual void resetRobot();
    virtual void updateRobot();
	void updateMotorAngles();

	virtual void stopRobot();
	float getMotorAngle(int motor);
	void clearMotorAngle(int motor);
	float getMotorVelocity(int motor);
	void setMotorVelocity(int motor,float vel);
	float getMotorImpulse(int motor);
	void setMotorImpulse(int motor,float imp);
	void setWheelFriction(btVector3 fric) { m_wheelFriction = fric; }
};
#endif // ROBOT_H