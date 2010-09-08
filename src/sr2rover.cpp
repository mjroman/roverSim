#include "sr2rover.h"

// Use if Bullet frameworks are used
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <LinearMath/btDefaultMotionState.h>

#define WHEELBASE   0.65
#define WHEELTRAC   0.55
#define WHEELRAD    0.1
#define WHEELWIDTH  0.1
#define MAXSPEED    20

// body HALF dimensions
#define BODYLENGTH  0.22
#define BODYWIDTH   0.17
#define BODYHEIGHT  0.1

// solar panel dimensions
#define PANELLENGTH     0.8
#define PANELWIDTH      0.65
#define PANELTHICKNESS  0.01
#define PANELOFFSET     -0.06

// SR2 motor and joint definitions
#define RFWHEEL			0
#define RRWHEEL			1
#define LFWHEEL			2
#define LRWHEEL			3
#define RSUSPEN			0
#define LSUSPEN			1

#define PANELLASERHEIGHT	0.48
#define PROFILELASERHEIGHT	0.475

static void solarPanel()
{
    // solar panel
    int i,j;
    static btVector3 vertices[16];

    for(i=0;i<2;i++){
        vertices[0+i*8].setValue(PANELWIDTH/2 - 0.045,PANELLENGTH/2 + PANELOFFSET,i*PANELTHICKNESS);
        vertices[1+i*8].setValue(-(PANELWIDTH/2 - 0.045),PANELLENGTH/2 + PANELOFFSET,i*PANELTHICKNESS);
        vertices[2+i*8].setValue(-PANELWIDTH/2,(PANELLENGTH/2 - 0.09) + PANELOFFSET,i*PANELTHICKNESS);
        vertices[3+i*8].setValue(-PANELWIDTH/2,-(PANELLENGTH/2 - 0.09) + PANELOFFSET,i*PANELTHICKNESS);
        vertices[4+i*8].setValue(-(PANELWIDTH/2 - 0.045),-PANELLENGTH/2 + PANELOFFSET,i*PANELTHICKNESS);
        vertices[5+i*8].setValue(PANELWIDTH/2 - 0.045,-(PANELLENGTH/2) + PANELOFFSET,i*PANELTHICKNESS);
        vertices[6+i*8].setValue(PANELWIDTH/2,-(PANELLENGTH/2 - 0.09) + PANELOFFSET,i*PANELTHICKNESS);
        vertices[7+i*8].setValue(PANELWIDTH/2,(PANELLENGTH/2 - 0.09) + PANELOFFSET,i*PANELTHICKNESS);
    }

    glBegin(GL_POLYGON);
    glNormal3f(0,0,-1);
    for(i=7;i>=0;i--){
        glVertex3f(vertices[i].x(),vertices[i].y(),vertices[i].z());
    }
    glEnd();

    glBegin(GL_QUADS);
    for(i=0;i<8;i++){
        if(i==7) j=0;
        else j=i+1;
        btVector3 Normal = (vertices[i+8]-vertices[i]).cross(vertices[j]-vertices[i]);
        glNormal3f(Normal.x(),Normal.y(),Normal.z());
        glVertex3f(vertices[i].x(),vertices[i].y(),vertices[i].z());
        glVertex3f(vertices[j].x(),vertices[j].y(),vertices[j].z());
        glVertex3f(vertices[j+8].x(),vertices[j+8].y(),vertices[j+8].z());
        glVertex3f(vertices[i+8].x(),vertices[i+8].y(),vertices[i+8].z());
    }
    glEnd();
}

SR2rover::SR2rover(simGLView* glView)
:
robot(glView),
m_gearTrain(192),
m_encoderRes(16),
m_rightEncoder(0),
m_leftEncoder(0),
panAngle(0),
tiltAngle(-15)
{
	int i;

// rpc struct hold the number of motor and passive joints, body parts, wheels and sensors.
	robotParts rpc;
	rpc.wheels = 4;
	rpc.passiveJoints = 2;
	rpc.motors = 4;
	rpc.bodyParts = 3;
	rpc.sensors = 3;
	
	initalloc(rpc);
	
	for(i=0;i<rpc.motors;i++) m_motorImpulse[i] = 0.1;

    // construct SR2rover and suspension constraints
    this->constructRover(btVector3(1,1,arena->worldSize().z()));
    this->stopRobot();

    if(m_view) {
        m_view->getCamera()->cameraSetRoverPointer(this);
       // m_view->getCamera()->cameraView = RoverFollow;
    }

    // add laser scanner sensors
    // the transform frame of reference is from the rover body center
    btTransform frame;
	laserScanner *laser;
	
    // body laser
    frame.setIdentity();
    frame.setOrigin(btVector3(0,BODYLENGTH+0.02,-(BODYHEIGHT-0.01)));
    frame.getBasis().setEulerZYX(0,PI,0);
    laser = new laserScanner(frame,HALFPI,DEGTORAD(5),0);
	m_laserList << laser;

    // panel laser
    frame.setIdentity();
	frame.setOrigin(btVector3(0,BODYLENGTH+0.1,BODYHEIGHT+0.03));
    frame.getBasis().setEulerZYX(-PI/4,0,0);
    laser = new laserScanner(frame,HALFPI,DEGTORAD(5),0);
	m_PLheights = new float[(int)(HALFPI/DEGTORAD(5))];
	m_laserList << laser;
	
    // profile laser
    frame.setIdentity();
    frame.setOrigin(btVector3(-0.1,BODYLENGTH+0.08,BODYHEIGHT+0.025));
    frame.getBasis().setEulerZYX(-HALFPI,0,-HALFPI);
    laser = new laserScanner(frame,HALFPI,DEGTORAD(5),DEGTORAD(-45));
	m_PRheights = new float[(int)(HALFPI/DEGTORAD(5))];
	m_laserList << laser;
	
	//mudParticle = new GLParticle(1,1,1,glView);
	m_view->printText("New SR2 rover");
}

SR2rover::~SR2rover()
{  
	int i;
	if(m_view) m_view->getCamera()->cameraSetRoverPointer(0);
	
    glDeleteLists(m_aWheel,1);
    glDeleteLists(m_aSuspension,1);
    glDeleteLists(m_aBody,1);
    glDeleteLists(m_aSPanel,1);

	for(i = 0; i < m_laserList.size(); ++i)
	{
		delete m_laserList[i];
	}
	delete [] m_PLheights;
	delete [] m_PRheights;
	m_laserList.clear();
	//delete mudParticle;
	m_view->printText("SR2 deleted");
}

void SR2rover::constructRover(const btVector3& positionOffset)
{
//////////////////////////////////////////////////////////////////////////////
// body shape geometry
    btCollisionShape* bodyShape = new btBoxShape(btVector3(BODYWIDTH,BODYLENGTH,BODYHEIGHT));
    m_robotShapes.push_back(bodyShape);
// wheel shape
    btCollisionShape* wheelShape = new btCylinderShapeX(btVector3(WHEELWIDTH/2.0,WHEELRAD,WHEELRAD));
    m_robotShapes.push_back(wheelShape);
// suspension shape
    btCollisionShape* susShape = new btCylinderShapeX(btVector3(btScalar(0.03),0.05,0.05));
    m_robotShapes.push_back(susShape);
	
//////////////////////////////////////////////////////////////////////////////
// construct rigid bodies
	int partIndex = 0;

    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(positionOffset);
    btQuaternion rot(0,0,-45*PI/180.);
    offset.setRotation(rot);

    float fHeight = 0.5;
    btVector3 vRoot = btVector3(0,0,fHeight);
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(vRoot);

// construct main body
    m_bodyAttachPoints[partIndex] = vRoot;
    m_robotObjects.push_back(arena->createRigidBody(1,offset*transform,bodyShape)); // create a box rigid body for the rover
	partIndex++;

// construct suspension parts
    // right suspension
    m_bodyAttachPoints[partIndex].setValue(BODYWIDTH+0.03,0,fHeight-0.06); // coordinates in world at startup
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_robotObjects.push_back(arena->createRigidBody(0.5,offset*transform,susShape)); // create a cylinder body for suspension
	partIndex++;
    // left suspension
    m_bodyAttachPoints[partIndex].setValue(-BODYWIDTH-0.03,0,fHeight-0.06);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_robotObjects.push_back(arena->createRigidBody(0.5,offset*transform,susShape)); // create a cylinder body for suspension
	partIndex++;

// construct wheel parts
    // front right wheel
    m_bodyAttachPoints[partIndex].setValue(WHEELTRAC/2,WHEELBASE/2,fHeight-.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_robotObjects.push_back(arena->createRigidBody(0.25,offset*transform,wheelShape));
	partIndex++;
    // rear right wheel
    m_bodyAttachPoints[partIndex].setValue(WHEELTRAC/2,-WHEELBASE/2,fHeight-0.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_robotObjects.push_back(arena->createRigidBody(0.25,offset*transform,wheelShape));
	partIndex++;
    // front left wheel
    m_bodyAttachPoints[partIndex].setValue(-WHEELTRAC/2,WHEELBASE/2,fHeight-0.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_robotObjects.push_back(arena->createRigidBody(0.25,offset*transform,wheelShape));
	partIndex++;
    // rear left wheel
    m_bodyAttachPoints[partIndex].setValue(-WHEELTRAC/2,-WHEELBASE/2,fHeight-0.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_robotObjects.push_back(arena->createRigidBody(0.25,offset*transform,wheelShape));
	partIndex++;

// set rigid body damping and friction parameters
    for(int i=0;i < m_robotObjects.size();i++){
        //m_robotObjects[i]->setActivationState(DISABLE_DEACTIVATION);
        m_robotObjects[i]->setDamping(0.05, 0.85);
        m_robotObjects[i]->setSleepingThresholds(0.025f, 0.5f);
        if(i>=m_partCount.bodyParts) m_robotObjects[i]->setAnisotropicFriction(m_wheelFriction);
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// add Joint Constraints
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    btTransform frameA,frameB,frameC;
    btHingeConstraint* joint;

// Right Suspension
    frameA.setIdentity();
    frameB.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(BODYWIDTH,0,-0.06)); // the location of the joint
    frameB = m_robotObjects[1]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_robotObjects[0],*m_robotObjects[1], frameA, frameB);
    joint->setLimit(-DEGTORAD(75),DEGTORAD(75),0.5,0.5,0.5);
    m_passiveJoints[RSUSPEN] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);
    joint->enableMotor(true);
    joint->setMaxMotorImpulse(0.5);
    joint->setMotorTarget(0,0.1);

// Right Front Axle
    frameA.setIdentity();
    frameB.setIdentity();
    frameC.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(WHEELTRAC/2,WHEELBASE/2,-0.25)); // the location of the joint
    frameB = m_robotObjects[1]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    frameC = m_robotObjects[3]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_robotObjects[1],*m_robotObjects[3], frameB, frameC);
    m_motorJoints[RFWHEEL] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Right Rear Axle
    frameA.setIdentity();
    frameB.setIdentity();
    frameC.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(WHEELTRAC/2,-WHEELBASE/2,-0.25)); // the location of the joint
    frameB = m_robotObjects[1]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    frameC = m_robotObjects[4]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_robotObjects[1],*m_robotObjects[4], frameB, frameC);
    m_motorJoints[RRWHEEL] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Left Suspension
    frameA.setIdentity();
    frameB.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(-BODYWIDTH,0,-0.06)); // the location of the joint
    frameB = m_robotObjects[2]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_robotObjects[0],*m_robotObjects[2], frameA, frameB);
    joint->setLimit(-DEGTORAD(75),DEGTORAD(75),0.5,0.5,0.5);
    m_passiveJoints[LSUSPEN] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Left front wheel
    frameA.setIdentity();
    frameB.setIdentity();
    frameC.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(-WHEELTRAC/2,WHEELBASE/2,-0.25)); // the location of the joint
    frameB = m_robotObjects[2]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    frameC = m_robotObjects[5]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_robotObjects[2],*m_robotObjects[5], frameB, frameC);
    m_motorJoints[LFWHEEL] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Left rear wheel
    frameA.setIdentity();
    frameB.setIdentity();
    frameC.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(-WHEELTRAC/2,-WHEELBASE/2,-0.25)); // the location of the joint
    frameB = m_robotObjects[2]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    frameC = m_robotObjects[6]->getWorldTransform().inverse() * m_robotObjects[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_robotObjects[2],*m_robotObjects[6], frameB, frameC);
    m_motorJoints[LRWHEEL] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

    if(m_view) generateGLLists();
}

    // create drawing lists
void SR2rover::generateGLLists()
{
    m_aWheel = glGenLists(1);
    glNewList(m_aWheel, GL_COMPILE);
        wheel(WHEELRAD+0.01,WHEELWIDTH);	// add a little extra to the graphics, makes it look like soft soil
    glEndList();

    // create suspension list
    m_aSuspension = glGenLists(1);
    glNewList(m_aSuspension, GL_COMPILE);
        glTranslatef(-0.02,0,0);
        tube(0.03, 0.04);
        glTranslatef(0.02,0,0);
        glRotated(90, 0, 0, 1);
        glRotated(30, 0, 1, 0);
        tube(0.015, 0.37);
        glRotated(120, 0, 1, 0);
        tube(0.015, 0.37);
    glEndList();

    // body list
    m_aBody = glGenLists(1);
    glNewList(m_aBody, GL_COMPILE);
        box(BODYWIDTH,BODYLENGTH,BODYHEIGHT);
    glEndList();

    // solar panel list
    m_aSPanel = glGenLists(1);
    glNewList(m_aSPanel, GL_COMPILE);
        glPushMatrix();
        glTranslatef(0,0,0.15);

        solarPanel();
        {
            glEnable(GL_TEXTURE_2D);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glBindTexture(GL_TEXTURE_2D, m_view->getTexture(2));
            glNormal3f(0,0,1);
            glBegin(GL_QUADS);
            glTexCoord2d(0,0);glVertex3f(PANELWIDTH/2,PANELLENGTH/2+PANELOFFSET,PANELTHICKNESS);
            glTexCoord2d(1,0);glVertex3f(-PANELWIDTH/2,PANELLENGTH/2+PANELOFFSET,PANELTHICKNESS);
            glTexCoord2d(1,1);glVertex3f(-PANELWIDTH/2,-PANELLENGTH/2+PANELOFFSET,PANELTHICKNESS);
            glTexCoord2d(0,1);glVertex3f(PANELWIDTH/2,-PANELLENGTH/2+PANELOFFSET,PANELTHICKNESS);
            glEnd();
            glDisable(GL_BLEND);
            glDisable(GL_TEXTURE_2D);
        }
        glTranslatef(0,0.03,0);
        glRotated(270,0,1,0);
        tube(0.01,0.4);
        glPopMatrix();
    glEndList();
}

void SR2rover::setRightSpeed(float spd)
{
    rightSpeed = spd;
    if(rightSpeed > MAXSPEED) rightSpeed = MAXSPEED;
    else if(rightSpeed < -MAXSPEED) rightSpeed = -MAXSPEED;
	m_motorVelocity[RFWHEEL] = rightSpeed;
	m_motorVelocity[RRWHEEL] = rightSpeed;
}
void SR2rover::setLeftSpeed(float spd)
{
    leftSpeed = spd;
	if(leftSpeed > MAXSPEED) leftSpeed = MAXSPEED;
    else if(leftSpeed < -MAXSPEED) leftSpeed = -MAXSPEED;
	m_motorVelocity[LFWHEEL] = leftSpeed;
	m_motorVelocity[LRWHEEL] = leftSpeed;
}
void SR2rover::incRightSpeed(float spd)
{
    rightSpeed += spd;
    if(rightSpeed > MAXSPEED) rightSpeed = MAXSPEED;
    else if(rightSpeed < -MAXSPEED) rightSpeed = -MAXSPEED;
	m_motorVelocity[RFWHEEL] = rightSpeed;
	m_motorVelocity[RRWHEEL] = rightSpeed;
}
void SR2rover::incLeftSpeed(float spd)
{
    leftSpeed += spd;
	if(leftSpeed > MAXSPEED) leftSpeed = MAXSPEED;
    else if(leftSpeed < -MAXSPEED) leftSpeed = -MAXSPEED;
	m_motorVelocity[LFWHEEL] = leftSpeed;
	m_motorVelocity[LRWHEEL] = leftSpeed;
}
void SR2rover::stopRobot()
{
	leftSpeed = rightSpeed = 0;
	robot::stopRobot();
}
void SR2rover::resetRobot()
{
	panAngle = 0;
	tiltAngle = -15;
	robot::resetRobot();
}

void SR2rover::updateRobot()
{
    // update rover position,pose and all motors to keep roundoff errors at a minimum
	robot::updateRobot();
	
	// update the right side motors
	m_rightEncoder = ((getMotorAngle(RFWHEEL) + getMotorAngle(RRWHEEL))/(2*TWOPI)) * m_encoderRes * m_gearTrain;
    
    // update the left side motors
    m_leftEncoder = ((getMotorAngle(LFWHEEL) + getMotorAngle(LRWHEEL))/(2*TWOPI)) * m_encoderRes * m_gearTrain;

	// update the odometer, I'm getting some roundoff error here
	odometer = WHEELRAD * (getMotorAngle(RFWHEEL) + getMotorAngle(RRWHEEL) + getMotorAngle(LFWHEEL) + getMotorAngle(LRWHEEL))/4.0;
	
	// update the body angle of the SR2 robot.
	// the body of this rover is connected through a differential mechanism therefore, the body
	// is always half the angle between the left and right suspension angles
    btHingeConstraint* rightHinge = static_cast<btHingeConstraint*>(m_passiveJoints[RSUSPEN]);
    btHingeConstraint* leftHinge = static_cast<btHingeConstraint*>(m_passiveJoints[LSUSPEN]);
    differentialAngle = (rightHinge->getHingeAngle() - leftHinge->getHingeAngle())/2;
    rightHinge->setMotorTarget(differentialAngle,0.1);
    //qDebug("%f  %f  %f",RADTODEG(diffAngle),RADTODEG(rightHinge->getHingeAngle()),RADTODEG(leftHinge->getHingeAngle()));

    // activate all rigid bodies only if a motor is in motion to reduce errors
   	if(leftSpeed || rightSpeed)
	{
        for(int i=0;i<m_robotObjects.size();i++)
            m_robotObjects[i]->activate(true);
	}

    // update rover sensors
	for(int i = 0; i < m_laserList.size(); ++i)
	{
		m_laserList[i]->update(m_robotObjects[0]->getWorldTransform());
	}
	
	//btTransform wheelTF = m_robotObjects[1]->getCenterOfMassTransform();
	//btVector3 mudPos = wheelTF(btVector3(0.05,-0.35,-0.3));
	//mudParticle->setPosition(mudPos.x(),mudPos.y(),mudPos.z());
	emit updated();
}

void SR2rover::paintLasers(int state)
{
	for(int i = 0; i < m_laserList.size(); ++i)
	{
		m_laserList[i]->setBeamVisable(state);
	}
}

void SR2rover::paintBodyLaser(bool state)
{
	m_laserList[BODYLASER]->setBodyVisable(state);
}
	
/////////////////////////////////////////
// converts the panel laser points to height values
/////////////
float* SR2rover::getPanelLaserHeights()
{
	int i,j;
	laserScanner* ls = m_laserList[PANELLASER];
	float* ranges = ls->getData();
	int segments = ls->getDataSize();
	float halfSegments = segments/2;
	
	for(i = 0; i < halfSegments; ++i)
	{
		j=halfSegments-i;
		m_PLheights[i] = PANELLASERHEIGHT - cos(j*ls->dTheta)*ranges[i]/(SQRTTWO);
	}
	for(i = halfSegments; i < segments; ++i)
	{
		j=i-halfSegments;
		m_PLheights[i] = PANELLASERHEIGHT - cos(j*ls->dTheta)*ranges[i]/(SQRTTWO);
	}
	return m_PLheights;
}
/////////////////////////////////////////
// converts the profile laser points to height values
/////////////
float* SR2rover::getProfileLaserHeights()
{
	int i;
	laserScanner* ls = m_laserList[PROFILELASER];
	float* ranges = ls->getData();
	int segments = ls->getDataSize();
	
	for(i = 0; i < segments; ++i)
	{
		m_PRheights[i] = PROFILELASERHEIGHT - cos(i*ls->dTheta)*ranges[i];
	}
	return m_PRheights;
}

/////////////////////////////////////////
// displays laser beams
/////////////
void SR2rover::toggleSensors()
{
    static int state = 1;
    paintLasers(state);
	state++;
	if(state > 3) state = 0;
}

void SR2rover::renderGLObject()
{
    int i=0;
    btScalar	glm[16];

    // draw suspension
    glColor3f(0.8f,0.8f,0.8f);
    for(i=1;i<3;i++){
        btDefaultMotionState* objMotionState = (btDefaultMotionState*)m_robotObjects[i]->getMotionState();
        objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);
        glPushMatrix();
        glMultMatrixf(glm);
        glCallList(m_aSuspension);
        glPopMatrix();
    }

    // draw body
    {
        btDefaultMotionState* objMotionState = (btDefaultMotionState*)m_robotObjects[0]->getMotionState();
        objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);
        glPushMatrix();
        glMultMatrixf(glm);

        if(m_view->getCamera()->cameraView != RoverView) {
            glCallList(m_aBody);
            glCallList(m_aSPanel);
            glTranslatef(0,0.03,0.55);
            glRotatef(panAngle,0,0,1);
            glRotatef(tiltAngle,1,0,0);
            glTranslatef(0.,0.03,0.02);

            // draw the pancam case
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, m_view->getTexture(1));
            glBegin(GL_QUADS);
            glNormal3f(0,1,0);
            glTexCoord2d(1,1); glVertex3f(0.07,0.013,0.04);
			glTexCoord2d(1,0.54); glVertex3f(0.07,0.013,-0.04);
			glTexCoord2d(0,0.54); glVertex3f(-0.07,0.013,-0.04);
            glTexCoord2d(0,1); glVertex3f(-0.07,0.013,0.04);  
            glEnd();
            glDisable(GL_TEXTURE_2D);

            glColor3f(1.f,0.73f,0.06f);
            glBegin(GL_QUADS);
            glNormal3f(0,-1,0);
            glVertex3f(0.07,0.012,0.04);
			glVertex3f(-0.07,0.012,0.04);
			glVertex3f(-0.07,0.012,-0.04);
            glVertex3f(0.07,0.012,-0.04);
            glEnd();
            box(0.06,0.0125,0.03);
        }
        else glCallList(m_aSPanel);
		
        glPopMatrix();
    }

    // draw wheels
    for(i=m_partCount.bodyParts;i<m_robotObjects.size();i++){
            btDefaultMotionState* objMotionState = (btDefaultMotionState*)m_robotObjects[i]->getMotionState();
            objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);

            glPushMatrix();
            glMultMatrixf(glm);
            glCallList(m_aWheel);
            glPopMatrix();
    }

    // draw body scanner
	for(i = 0; i < m_laserList.size(); ++i)
	{
		m_laserList[i]->drawLaser(m_robotObjects[0]->getWorldTransform());
	}
}
