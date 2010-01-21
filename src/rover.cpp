#include "rover.h"
#include "utility/glshapes.h"

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

rover::rover(simGLView* glView)
:
simGLObject(glView),
m_rightEncoder(0),
m_leftEncoder(0),
m_gearTrain(192),
m_encoderRes(16),
m_numWheels(4),
m_numPassive(2),
m_numMotors(4),
m_numBodyParts(3)
{
    qDebug("rover startup");

    wheelFriction.setValue(3,5,5);
    motorImpulse = 0.1;
    panAngle = 0;
    tiltAngle = -15;

    arena = physicsWorld::instance(); // get the physics world object

    // construct rover and suspension constraints
    this->constructRover(btVector3(1,1,arena->worldSize().z()));
    this->stopRover();

    if(m_view) {
        m_view->getCamera()->cameraSetRoverPointer(this);
        m_view->getCamera()->cameraView = RoverFollow;
    }

    // add laser scanner sensors
    // the transform frame of reference is from the rover body center
    btTransform frame;

    // body laser
    frame.setIdentity();
    frame.setOrigin(btVector3(0,BODYLENGTH+0.02,-(BODYHEIGHT-0.01)));
    frame.getBasis().setEulerZYX(0,PI,0);
    bodyLaser = new laserScanner(frame,HALFPI,DEGTORAD(5),0);

    // panel laser
    frame.setIdentity();
    frame.setOrigin(btVector3(0,BODYLENGTH+0.1,BODYHEIGHT+0.03));
    frame.getBasis().setEulerZYX(-PI/4,0,0);
    panelLaser = new laserScanner(frame,HALFPI,DEGTORAD(5),0);

    // profile laser
    frame.setIdentity();
    frame.setOrigin(btVector3(-0.1,BODYLENGTH+0.08,BODYHEIGHT+0.025));
    frame.getBasis().setEulerZYX(-HALFPI,0,-HALFPI);
    profileLaser = new laserScanner(frame,HALFPI,DEGTORAD(5),DEGTORAD(-45));

    this->resetEncoders();
}

rover::~rover()
{   
    int i=0;
	
    qDebug("deleting rover");

    if(m_view) m_view->getCamera()->cameraSetRoverPointer(0);
    
    //cleanup in the reverse order of creation/initialization
    while(i < arena->getDynamicsWorld()->getNumConstraints())
    {
        btTypedConstraint* constraint = arena->getDynamicsWorld()->getConstraint(i);
        arena->getDynamicsWorld()->removeConstraint(constraint);
        delete constraint;
    }
	
    //remove the rigidbodies from the dynamics world and delete them
    arena->deleteGroup(ROVER_GROUP);
	
    m_roverShapes.clear();

    glDeleteLists(m_aWheel,1);
    glDeleteLists(m_aSuspension,1);
    glDeleteLists(m_aBody,1);
    glDeleteLists(m_aSPanel,1);

    delete [] m_motorJoints;
    delete [] m_passiveJoints;
    delete [] m_bodyParts;
    delete [] m_bodyAttachPoints;
    delete bodyLaser;
    delete panelLaser;
    delete profileLaser;
}

void rover::constructRover(const btVector3& positionOffset)
{
//////////////////////////////////////////////////////////////////////////////
// body shape geometry
    btCollisionShape* bodyShape = new btBoxShape(btVector3(BODYWIDTH,BODYLENGTH,BODYHEIGHT));
    m_roverShapes.push_back(bodyShape);
// wheel shape
    btCollisionShape* wheelShape = new btCylinderShapeX(btVector3(WHEELWIDTH/2.0,WHEELRAD,WHEELRAD));
    m_roverShapes.push_back(wheelShape);
// suspension shape
    btCollisionShape* susShape = new btCylinderShapeX(btVector3(btScalar(0.03),0.05,0.05));
    m_roverShapes.push_back(susShape);
	
//////////////////////////////////////////////////////////////////////////////
// construct rigid bodies
    m_bodyParts = new btRigidBody *[m_numBodyParts + m_numWheels];        // allocate memory for rigid bodies that make up the rover
    m_bodyAttachPoints = new btVector3 [m_numBodyParts + m_numWheels];    // create an array for all the body parts attachment points
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
    m_bodyParts[partIndex] = arena->createRigidBody(1,offset*transform,bodyShape,ROVER_GROUP); // create a box rigid body for the rover
    partIndex++;

// construct suspension parts
    // right suspension
    m_bodyAttachPoints[partIndex].setValue(BODYWIDTH+0.03,0,fHeight-0.06); // coordinates in world at startup
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_bodyParts[partIndex] = arena->createRigidBody(0.5,offset*transform,susShape,ROVER_GROUP); // create a cylinder body for suspension
    partIndex++;
    // left suspension
    m_bodyAttachPoints[partIndex].setValue(-BODYWIDTH-0.03,0,fHeight-0.06);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_bodyParts[partIndex] = arena->createRigidBody(0.5,offset*transform,susShape,ROVER_GROUP); // create a cylinder body for suspension
    partIndex++;

// construct wheel parts
    // front right wheel
    m_bodyAttachPoints[partIndex].setValue(WHEELTRAC/2,WHEELBASE/2,fHeight-.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_bodyParts[partIndex] = arena->createRigidBody(0.25,offset*transform,wheelShape,ROVER_GROUP);
    partIndex++;
    // rear right wheel
    m_bodyAttachPoints[partIndex].setValue(WHEELTRAC/2,-WHEELBASE/2,fHeight-0.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_bodyParts[partIndex] = arena->createRigidBody(0.25,offset*transform,wheelShape,ROVER_GROUP);
    partIndex++;
    // front left wheel
    m_bodyAttachPoints[partIndex].setValue(-WHEELTRAC/2,WHEELBASE/2,fHeight-0.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_bodyParts[partIndex] = arena->createRigidBody(0.25,offset*transform,wheelShape,ROVER_GROUP);
    partIndex++;
    // rear left wheel
    m_bodyAttachPoints[partIndex].setValue(-WHEELTRAC/2,-WHEELBASE/2,fHeight-0.25);
    transform.setIdentity();
    transform.setOrigin(m_bodyAttachPoints[partIndex]);
    m_bodyParts[partIndex] = arena->createRigidBody(0.25,offset*transform,wheelShape,ROVER_GROUP);
    partIndex++;

// set rigid body damping and friction parameters
    for(int i=0;i<m_numBodyParts+m_numWheels;i++){
        //m_bodyParts[i]->setActivationState(DISABLE_DEACTIVATION);
        m_bodyParts[i]->setDamping(0.05, 0.85);
        m_bodyParts[i]->setSleepingThresholds(0.025f, 0.5f);
        if(i>=m_numBodyParts) m_bodyParts[i]->setAnisotropicFriction(wheelFriction);
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// add Joint Constraints
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    m_motorJoints = new btTypedConstraint *[m_numMotors];             // allocate memory for motor joint pointers
    m_passiveJoints = new btTypedConstraint *[m_numPassive];              // allocate memory for passive joint pointers
    m_previousPosition = new float[m_numMotors];
    memset(m_previousPosition,0,sizeof(m_previousPosition));

    btTransform frameA,frameB,frameC;
    btHingeConstraint* joint;

// Right Suspension
    frameA.setIdentity();
    frameB.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(BODYWIDTH,0,-0.06)); // the location of the joint
    frameB = m_bodyParts[1]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_bodyParts[0],*m_bodyParts[1], frameA, frameB);
    joint->setLimit(-DEGTORAD(75),DEGTORAD(75),0.5,0.5,0.5);
    m_passiveJoints[0] = joint;
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
    frameB = m_bodyParts[1]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    frameC = m_bodyParts[3]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_bodyParts[1],*m_bodyParts[3], frameB, frameC);
    m_motorJoints[0] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Right Rear Axle
    frameA.setIdentity();
    frameB.setIdentity();
    frameC.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(WHEELTRAC/2,-WHEELBASE/2,-0.25)); // the location of the joint
    frameB = m_bodyParts[1]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    frameC = m_bodyParts[4]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_bodyParts[1],*m_bodyParts[4], frameB, frameC);
    m_motorJoints[1] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Left Suspension
    frameA.setIdentity();
    frameB.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(-BODYWIDTH,0,-0.06)); // the location of the joint
    frameB = m_bodyParts[2]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_bodyParts[0],*m_bodyParts[2], frameA, frameB);
    joint->setLimit(-DEGTORAD(75),DEGTORAD(75),0.5,0.5,0.5);
    m_passiveJoints[1] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Left front wheel
    frameA.setIdentity();
    frameB.setIdentity();
    frameC.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(-WHEELTRAC/2,WHEELBASE/2,-0.25)); // the location of the joint
    frameB = m_bodyParts[2]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    frameC = m_bodyParts[5]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_bodyParts[2],*m_bodyParts[5], frameB, frameC);
    m_motorJoints[2] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

// Left rear wheel
    frameA.setIdentity();
    frameB.setIdentity();
    frameC.setIdentity();
    frameA.getBasis().setEulerZYX(0,PI/2,0);
    frameA.setOrigin(btVector3(-WHEELTRAC/2,-WHEELBASE/2,-0.25)); // the location of the joint
    frameB = m_bodyParts[2]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    frameC = m_bodyParts[6]->getWorldTransform().inverse() * m_bodyParts[0]->getWorldTransform() * frameA;
    joint = new btHingeConstraint(*m_bodyParts[2],*m_bodyParts[6], frameB, frameC);
    m_motorJoints[3] = joint;
    arena->getDynamicsWorld()->addConstraint(joint,true);

    generateGLLists();
}

    // create drawing lists
void rover::generateGLLists()
{
    m_aWheel = glGenLists(1);
    glNewList(m_aWheel, GL_COMPILE);
        wheel(WHEELRAD+0.01,WHEELWIDTH);
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

void rover::setRightSpeed(float spd)
{
    rightSpeed = spd;
}
void rover::setLeftSpeed(float spd)
{
    leftSpeed = spd;
}
void rover::incRightSpeed(float spd)
{
    rightSpeed += spd;
}
void rover::incLeftSpeed(float spd)
{
    leftSpeed += spd;
}
void rover::stopRover()
{
    setRightSpeed(0);
    setLeftSpeed(0);
}
void rover::updateMotors()
{
    // set max, min, and off motor speeds
    if(rightSpeed > MAXSPEED) rightSpeed = MAXSPEED;
    else if(rightSpeed < -MAXSPEED) rightSpeed = -MAXSPEED;
    if(leftSpeed > MAXSPEED) leftSpeed = MAXSPEED;
    else if(leftSpeed < -MAXSPEED) leftSpeed = -MAXSPEED;
    //qDebug("left:%f right:%f",leftSpeed,rightSpeed);
	
    // update the right side motors
    btHingeConstraint* frontRight = static_cast<btHingeConstraint*>(m_motorJoints[0]);
    frontRight->enableAngularMotor(true,rightSpeed,motorImpulse);
    btHingeConstraint* rearRight = static_cast<btHingeConstraint*>(m_motorJoints[1]);
    rearRight->enableAngularMotor(true,rightSpeed,motorImpulse);

    { // update right encoder
        float dFront = frontRight->getHingeAngle() - m_previousPosition[0];
        if(dFront < -PI) dFront = TWOPI + dFront;
        else if(dFront > PI) dFront -= TWOPI;
        m_previousPosition[0] = frontRight->getHingeAngle();

        float dRear = rearRight->getHingeAngle() - m_previousPosition[1];
        if(dRear < -PI) dRear = TWOPI + dRear;
        else if(dRear > PI) dRear -= TWOPI;
         m_previousPosition[1] = rearRight->getHingeAngle();

        m_rightEncoder += ((dFront + dRear)/(2*TWOPI)) * m_encoderRes * m_gearTrain;
    }

    // update the left side motors
    btHingeConstraint* frontLeft = static_cast<btHingeConstraint*>(m_motorJoints[2]);
    frontLeft->enableAngularMotor(true,leftSpeed,motorImpulse);
    btHingeConstraint* rearLeft = static_cast<btHingeConstraint*>(m_motorJoints[3]);
    rearLeft->enableAngularMotor(true,leftSpeed,motorImpulse);

    { // update left encoder
        float dFront = frontLeft->getHingeAngle() - m_previousPosition[2];
        if(dFront < -PI) dFront = TWOPI + dFront;
        else if(dFront > PI) dFront -= TWOPI;
        m_previousPosition[2] = frontLeft->getHingeAngle();

        float dRear = rearLeft->getHingeAngle() - m_previousPosition[3];
        if(dRear < -PI) dRear = TWOPI + dRear;
        else if(dRear > PI) dRear -= TWOPI;
         m_previousPosition[3] = rearLeft->getHingeAngle();

        m_leftEncoder += ((dFront + dRear)/(2*TWOPI)) * m_encoderRes * m_gearTrain;
    }

    btHingeConstraint* rightHinge = static_cast<btHingeConstraint*>(m_passiveJoints[0]);
    btHingeConstraint* leftHinge = static_cast<btHingeConstraint*>(m_passiveJoints[1]);
    float diffAngle = (rightHinge->getHingeAngle() - leftHinge->getHingeAngle())/2;
    rightHinge->setMotorTarget(diffAngle,0.1);
    //qDebug("%f  %f  %f",RADTODEG(diffAngle),RADTODEG(rightHinge->getHingeAngle()),RADTODEG(leftHinge->getHingeAngle()));

    // activate all rigid bodies
   if(leftSpeed || rightSpeed){
        for(int i=0;i<m_numWheels+m_numBodyParts;i++)
            m_bodyParts[i]->activate(true);
   }
}
void rover::resetEncoders()
{
    m_leftEncoder = 0;
    m_rightEncoder = 0;

    for(int i=0;i<m_numMotors;i++) m_previousPosition[i] = 0;
}

// sets the rover speeds to zero
// drops it on the terrain upright where it was
void rover::placeRoverAt(btVector3 here)
{
    stopRover();
	
    here.setZ(here.z() + 0.2);
    btTransform trans;
    trans.setIdentity();
    trans.setOrigin(here);

    for(int i=0;i<m_numBodyParts+m_numWheels;i++){
        btRigidBody* body = m_bodyParts[i];
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
        }
    }
    arena->resetBroadphaseSolver();
    resetEncoders();
}

void rover::resetRover()
{
    btVector3 place = m_bodyParts[0]->getCenterOfMassPosition();
    placeRoverAt(place);
}

void rover::updateRover()
{
    // update rover motors to keep roundoff errors to a minimum
    updateMotors();

    // update rover sensors
    bodyLaser->update(m_bodyParts[0]->getWorldTransform());
    panelLaser->update(m_bodyParts[0]->getWorldTransform());
    profileLaser->update(m_bodyParts[0]->getWorldTransform());

    // update rover position
    position = m_bodyParts[0]->getCenterOfMassPosition();

    // update rover heading, pitch, and roll
    btTransform roverTrans = m_bodyParts[0]->getCenterOfMassTransform();
    btVector3 column = roverTrans.getBasis().getColumn(0);
    btVector3 row = roverTrans.getBasis().getRow(2);
    heading = atan2(-column.y(),column.x());
    if(heading < 0) heading = TWOPI + heading;
    roll = atan2(column.z(),sqrt(row.y()*row.y() + row.z()*row.z()));
    pitch = atan2(row.y(),row.z());
}

void rover::paintLasers(bool state)
{
    bodyLaser->setBeamVisable(state);
    panelLaser->setBeamVisable(state);
    profileLaser->setBeamVisable(state);
}

void rover::toggleSensors()
{
    static bool state = true;
    paintLasers(state);
    state = !state;
}

btTransform rover::getRoverTransform()
{
    return m_bodyParts[0]->getCenterOfMassTransform();
}

void rover::renderGLObject()
{
    int i=0;
    btScalar	glm[16];

    // draw suspension
    glColor3f(0.8f,0.8f,0.8f);
    for(i=1;i<3;i++){
        btDefaultMotionState* objMotionState = (btDefaultMotionState*)m_bodyParts[i]->getMotionState();
        objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);
        glPushMatrix();
        glMultMatrixf(glm);

        glCallList(m_aSuspension);
        glPopMatrix();
    }

    // draw body
    {
        btDefaultMotionState* objMotionState = (btDefaultMotionState*)m_bodyParts[0]->getMotionState();
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
            glTexCoord2d(0,1); glVertex3f(0.07,0.013,0.04);
            glTexCoord2d(1,1); glVertex3f(-0.07,0.013,0.04);
            glTexCoord2d(1,0.55); glVertex3f(-0.07,0.013,-0.04);
            glTexCoord2d(0,0.55); glVertex3f(0.07,0.013,-0.04);
            glEnd();
            glDisable(GL_TEXTURE_2D);

            glColor3f(1.f,0.73f,0.06f);
            glBegin(GL_QUADS);
            glNormal3f(0,-1,0);
            glVertex3f(0.07,0.012,0.04);
            glVertex3f(0.07,0.012,-0.04);
            glVertex3f(-0.07,0.012,-0.04);
            glVertex3f(-0.07,0.012,0.04);
            glEnd();
            box(0.06,0.0125,0.03);
        }
        else glCallList(m_aSPanel);

        glPopMatrix();
    }

    // draw wheels
    for(i=m_numBodyParts;i<m_numWheels+m_numBodyParts;i++){
            btDefaultMotionState* objMotionState = (btDefaultMotionState*)m_bodyParts[i]->getMotionState();
            objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);

            glPushMatrix();
            glMultMatrixf(glm);
            glCallList(m_aWheel);
            glPopMatrix();
    }

    // draw body scanner
    bodyLaser->drawLaser(m_bodyParts[0]->getWorldTransform(),true);
    panelLaser->drawLaser(m_bodyParts[0]->getWorldTransform(),true);
    profileLaser->drawLaser(m_bodyParts[0]->getWorldTransform(),true);

    /*for(i=0;i<m_numBodyParts;i++){
        drawFrame(m_bodyParts[i]->getWorldTransform());
    }*/

}

void rover::drawFrame(btTransform &tr)

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

