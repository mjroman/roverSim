#include "robot.h"

// Use if Bullet frameworks are used
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <LinearMath/btDefaultMotionState.h>

robot::robot(simGLView* glView)
:
simGLObject(glView),
pitch(0),
roll(0),
heading(0)
{
	m_wheelFriction.setValue(3,5,5);
	position.setValue(0,0,0);
	// GPS of flegar hall
	GPSposition.lat.dd = 35.21002;
	GPSposition.lon.dd = 97.44330;
	
	arena = physicsWorld::instance(); // get the physics world object
	
	// add a few default views
}

void robot::initalloc(robotParts pn)
{
	m_partCount = pn;
// allocate memory for motor position,velocity and impulse data.
	m_motorAngle = new float[m_partCount.motors];
	memset(m_motorAngle,0,sizeof(m_motorAngle));
	m_motorVelocity = new float[m_partCount.motors];
	memset(m_motorVelocity,0,sizeof(m_motorVelocity));
	m_motorImpulse = new float[m_partCount.motors];
	memset(m_motorImpulse,0,sizeof(m_motorImpulse));
	
	m_previousPosition = new float[m_partCount.motors];			// allocate memory for motor position data
    memset(m_previousPosition,0,sizeof(m_previousPosition));	
	m_motorJoints = new btTypedConstraint *[m_partCount.motors];     	// allocate memory for motor joint pointers
    m_passiveJoints = new btTypedConstraint *[m_partCount.passiveJoints];  	// allocate memory for passive joint pointers	
	m_bodyAttachPoints = new btVector3 [m_partCount.bodyParts + m_partCount.wheels];    // create an array for all the body parts attachment points
}

robot::~robot()
{
	int i=0;
    //cleanup in the reverse order of creation/initialization
    while(i < arena->getDynamicsWorld()->getNumConstraints())
    {
        btTypedConstraint* constraint = arena->getDynamicsWorld()->getConstraint(i);
        arena->getDynamicsWorld()->removeConstraint(constraint);
        delete constraint;
    }
	
    //remove the rigidbodies from the dynamics world and delete them
	deleteRobotGroup();
	
	delete [] m_motorAngle;
	delete [] m_motorVelocity;
	delete [] m_motorImpulse;
	
	delete [] m_previousPosition;
	delete [] m_motorJoints;
    delete [] m_passiveJoints;
    delete [] m_bodyAttachPoints;
}

void robot::deleteRobotGroup()
{
	int i = m_robotObjects.size();
 	if(m_view) m_view->stopDrawing(); 			// do not draw
 	arena->stopSimTimer();			// pause simulation
	
	while(i>0){
		btRigidBody* obj = m_robotObjects[i-1];
		arena->getDynamicsWorld()->removeCollisionObject(obj);
		m_robotObjects.pop_back();
		i = m_robotObjects.size();
	}

	m_robotShapes.clear();
	arena->resetWorld();			// reset and unpause simulation
	if(m_view) m_view->startDrawing(); 		// draw obstacles
}

btTransform robot::getRobotTransform()
{
	return m_robotObjects[0]->getCenterOfMassTransform();
}

// sets the rover speeds to zero
// drops it on the terrain upright where it was
void robot::placeRobotAt(btVector3 here)
{
	int i;
    stopRobot();
	
    here.setZ(here.z() + 0.2);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(here);

    for(i=0;i < m_robotObjects.size();i++){
        btRigidBody* body = m_robotObjects[i];
        if(body && body->getMotionState()){
            btDefaultMotionState* bodyMotionState = (btDefaultMotionState*)body->getMotionState();
            btTransform bodyTrans;
            bodyTrans.setIdentity();
            bodyTrans.setOrigin(m_bodyAttachPoints[i]);
            //qDebug("body:%d  %f,%f,%f",i,trans(m_bodyAttachPoints[i]).x(),trans(m_bodyAttachPoints[i]).y(),trans(m_bodyAttachPoints[i]).z());

            bodyMotionState->m_startWorldTrans = bodyTrans * trans;
            bodyMotionState->m_graphicsWorldTrans = bodyMotionState->m_startWorldTrans;
            body->setCenterOfMassTransform( bodyMotionState->m_startWorldTrans );
            body->setInterpolationWorldTransform( bodyMotionState->m_startWorldTrans );
            body->setAngularVelocity(btVector3(0,0,0));
            body->setLinearVelocity(btVector3(0,0,0));
			body->activate(true);
        }
    }
    arena->resetWorld();
    
	for(i=0;i<m_partCount.motors;i++)
		clearMotorAngle(i);
}

void robot::resetRobot()
{
    btVector3 place = m_robotObjects[0]->getCenterOfMassPosition();
    placeRobotAt(place);
}

void robot::updateRobot()
{
	// update rover position
    position = m_robotObjects[0]->getCenterOfMassPosition();

    // update rover heading, pitch, and roll
    btTransform roverTrans = m_robotObjects[0]->getCenterOfMassTransform();
    btVector3 column = roverTrans.getBasis().getColumn(0);
    btVector3 row = roverTrans.getBasis().getRow(2);
    heading = atan2(-column.y(),column.x());
    if(heading < 0) heading = TWOPI + heading;
    roll = atan2(column.z(),sqrt(row.y()*row.y() + row.z()*row.z()));
    pitch = atan2(row.y(),row.z());
	
	// update all motor position angles
	updateMotorAngles();
}

void robot::updateMotorAngles()
{
	int i;
	for(i=0;i<m_partCount.motors;i++){
		btHingeConstraint* mj = static_cast<btHingeConstraint*>(m_motorJoints[i]);
    	mj->enableAngularMotor(true,m_motorVelocity[i],m_motorImpulse[i]);
		float delmj = mj->getHingeAngle() - m_previousPosition[i];
        if(delmj < -PI) delmj = TWOPI + delmj;
        else if(delmj > PI) delmj -= TWOPI;
		m_motorAngle[i] += delmj;
        m_previousPosition[i] = mj->getHingeAngle();
	}
}

void robot::stopRobot()
{
	int i;
	for(i=0;i<m_partCount.motors;i++) m_motorVelocity[i] = 0;
}
float robot::getMotorAngle(int motor)
{
	return m_motorAngle[motor];
}
void robot::clearMotorAngle(int motor)
{
	m_previousPosition[motor] = 0;
	m_motorAngle[motor] = 0;
}
float robot::getMotorVelocity(int motor)
{
	return m_motorVelocity[motor];
}
void robot::setMotorVelocity(int motor,float vel)
{
	m_motorVelocity[motor] = vel;
}
float robot::getMotorImpulse(int motor)
{
	return m_motorImpulse[motor];
}
void robot::setMotorImpulse(int motor,float imp)
{
	m_motorImpulse[motor] = imp;
}


void robot::drawFrame(btTransform &tr)
{
        const float fSize = 0.5f;
        glLineWidth(1.0);
        glBegin(GL_LINES);

        // x
        glColor3f(255.f,0,0);
        btVector3 vX = tr*btVector3(fSize,0,0);	
        glVertex3d(tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
        glVertex3d(vX.getX(), vX.getY(), vX.getZ());

        // y
        glColor3f(0,255.f,0);
        btVector3 vY = tr*btVector3(0,fSize,0);
        glVertex3d(tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
        glVertex3d(vY.getX(), vY.getY(), vY.getZ());

        // z
        glColor3f(0,0,255.f);
        btVector3 vZ = tr*btVector3(0,0,fSize);
        glVertex3d(tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
        glVertex3d(vZ.getX(), vZ.getY(), vZ.getZ());

        glEnd();
}


