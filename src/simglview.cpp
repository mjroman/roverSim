#include <QtGui>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

#include "simglview.h"
#include "utility/definitions.h"
#include "utility/glshapes.h"
#include "simGLObject.h"

// Use if Bullet frameworks are used
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btDefaultMotionState.h>

GLfloat lightAmbient[]={0.2f,0.2f,0.2f,1.0f};
GLfloat lightZeroPos[]={50.0f,0.0f,200.0f,1.0f};
GLfloat lightZero[]={0.88f,0.86f,0.8f,1.0};
GLfloat lightOnePos[]={50.f,50.f,100.f,0.};
GLfloat lightOne[]={0.1f,0.1f,0.1f,1.0};
GLfloat obstacleMaterial[4] = { 0.02f, 0.52f, 0.51f, 1.0f };
GLfloat obstacleEmission[4] = { 0.02f,0.52f,0.51f, 0.f };
GLfloat reflectance3[4] = { 0.0f, 0.9f, 0.9f, 1.0f };
GLfloat ghostColor[4] = { 0.9f, 0.9f, 0.9f, 0.1f};

simGLView::simGLView(QWidget *parent) : QGLWidget(parent)
{
    qDebug("glView startup");
    this->setMinimumSize(80,50);

    m_viewAngle = 1.0;
    m_eye = new camera(btVector3(-5,-5,5),btVector3(0,0,0));

    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    m_timer->start(100);

    arena = physicsWorld::instance(); // get the physics world object
	
}

QSize simGLView::sizeHint() const
{
    return QSize(800, 400);
}

void simGLView::stopDrawing()
{
	m_timer->stop();
}
void simGLView::startDrawing()
{
	m_timer->start(100);
}
simGLView::~simGLView()
{
	m_timer->stop();
	delete m_timer;
	
    int i=0;
    for(i=0;i<3;i++) this->deleteTexture(m_texture[i]);
    delete m_eye;
    qDebug("deleting glView");
	qDebug("#################################################################");
}

void simGLView::registerGLObject(simGLObject *obj)
{
    renderList.append(obj);
}

void simGLView::unregisterGLObject(simGLObject *obj)
{
    int x = renderList.indexOf(obj);
    if(x != -1) renderList.removeAt(x);
}

void simGLView::loadTextures()
{
    m_texture[0] = this->bindTexture(QPixmap(QString(":/textures/src/textures/domePan1.png")),GL_TEXTURE_2D,GL_RGBA);
    m_texture[1] = this->bindTexture(QPixmap(QString(":/textures/src/textures/pancam.png")),GL_TEXTURE_2D,GL_RGBA);
    m_texture[2] = this->bindTexture(QPixmap(QString(":/textures/src/textures/solarPanel2.png")),GL_TEXTURE_2D,GL_RGBA);
	m_texture[3] = this->bindTexture(QPixmap(QString(":/textures/src/textures/spark.png")),GL_TEXTURE_2D,GL_RGBA);
	m_texture[4] = this->bindTexture(QPixmap(QString(":/textures/src/textures/mud.png")),GL_TEXTURE_2D,GL_RGBA);
	glBindTexture(GL_TEXTURE_2D, m_texture[0]);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
}

GLuint simGLView::getTexture(int n)
{
    return m_texture[n];
}

void simGLView::initializeGL()
{
    qDebug("GL initialize");

    this->loadTextures();

    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);
    //glEnable(GL_NORMALIZE);
    //glEnable(GL_CULL_FACE);
}

void simGLView::resizeGL(int width, int height)
{
    float ratio;
    glViewport(0,0,width,height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    ratio = (float)width / (float)height;
    gluPerspective(35/m_viewAngle, ratio, NEARCLIPPING, FARCLIPPING);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void simGLView::setViewAngle(float angle)
{
    if(angle > 35) angle = 35;
    else if(angle < 0.5) angle = 0.5;
    m_viewAngle = angle;
    this->resizeGL(this->width(),this->height());
}

void simGLView::paintGL()
{
    if(arena->canDraw()){
        glClearColor(0.07,0.07,0.07,0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        m_eye->cameraUpdate();

        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lightAmbient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZero);
        glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPos);

        glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOne);
        glLightfv(GL_LIGHT1, GL_POSITION, lightOnePos);

        drawWorld();

        for(int i=0;i<renderList.size();i++)
            renderList[i]->renderGLObject();

        overlayGL();
        emit refreshView();
    }
}

void simGLView::overlayGL()
{
    int height = this->frameSize().height();
    int width = this->frameSize().width();

    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, width, 0, height);
    glColor3f(0, 1, 0);

    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex2f(width/2-5,height/2);
    glVertex2f(width/2+5,height/2);
    glVertex2f(width/2,height/2+5);
    glVertex2f(width/2,height/2-5);
    glEnd();

    glEnable(GL_LIGHTING);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

//////////////////////////////////////////////////////////////////////////////////////////
// user input
/////////////
void simGLView::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePoint = event->pos();
}

void simGLView::mouseMoveEvent(QMouseEvent *event)
{
    QPoint del;
	
    del = event->pos() - m_lastMousePoint;
	
    m_eye->cameraMouseMove(del,event);
    m_lastMousePoint = event->pos();
}

void simGLView::wheelEvent(QWheelEvent *event)
{
    m_eye->cameraMouseWheel(event);
}

//////////////////////////////////////////////////////////////////////////////////////////
// drawing methods
//////////////////
void simGLView::drawTest()
{
    // Test openGL drawing
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, ghostColor);
    glTranslatef(0,0,1);
    sphere(1,10,10);
	
    glMaterialfv(GL_FRONT, GL_DIFFUSE, obstacleMaterial);
    glTranslatef(2,2,0);
    box(0.5,0.5,0.5);
	
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, reflectance3);
    glTranslatef(0,-2,-1);
    cone(1,3,10);
}

void simGLView::drawPlane(btScalar constant,const btVector3 normal)
{
    btVector3 planeOrigin = normal * constant;
    btVector3 vec0,vec1;
    btPlaneSpace1(normal,vec0,vec1);
    btScalar vecLen = 75.f;
    btVector3 pt0 = planeOrigin + (vec0*vecLen + vec1*vecLen);
    btVector3 pt1 = planeOrigin - (vec0*vecLen + vec1*vecLen);
    btVector3 pt2 = planeOrigin + (vec0*vecLen - vec1*vecLen);
    btVector3 pt3 = planeOrigin - (vec0*vecLen - vec1*vecLen);
	
    glNormal3f(normal.x(),normal.y(),normal.z());
    glBegin(GL_QUADS);
    glVertex3f(pt0.getX(),pt0.getY(),pt0.getZ());
    glVertex3f(pt3.getX(),pt3.getY(),pt3.getZ());
    glVertex3f(pt1.getX(),pt1.getY(),pt1.getZ());
    glVertex3f(pt2.getX(),pt2.getY(),pt2.getZ());
    glEnd();
}

void simGLView::drawWorld()
{
    btScalar	glm[16];
    btDynamicsWorld *pWorld = arena->getDynamicsWorld();
    const int	numObjects = pWorld->getNumCollisionObjects();
	

    glColor3f(0.02f,0.52f,0.51f);
    //glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    //glMaterialfv(GL_FRONT, GL_EMISSION, obstacleEmission);
    for(int i=0;i<numObjects;i++){
        btCollisionObject*	colisObject = pWorld->getCollisionObjectArray()[i];
        btRigidBody*		body = btRigidBody::upcast(colisObject);
		
        if(body && body->getMotionState())
        {
            btDefaultMotionState* objMotionState = (btDefaultMotionState*)body->getMotionState();
            objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);
        }
        else
        {
            colisObject->getWorldTransform().getOpenGLMatrix(glm);
        }

        if(*(int*)body->getUserPointer() == ROVER_GROUP) continue;
        if(*(int*)body->getUserPointer() == TERRAIN_GROUP) continue;

        btCollisionShape* colisShape = colisObject->getCollisionShape();
		
        glPushMatrix();
        glMultMatrixf(glm);

        switch (colisShape->getShapeType()) {
			case BOX_SHAPE_PROXYTYPE: {
				const btBoxShape* boxShape = static_cast<const btBoxShape*>(colisShape);
				btVector3 halfDims = boxShape->getHalfExtentsWithMargin();

                                box(halfDims.x(),halfDims.y(),halfDims.z());
				break;
			}
			case SPHERE_SHAPE_PROXYTYPE:
                        {
				const btSphereShape* sphereShape = static_cast<const btSphereShape*>(colisShape);
				float radius = sphereShape->getMargin();//radius doesn't include the margin, so draw with margin
                                sphere(radius,10,10);
				break;
			}
			case CONE_SHAPE_PROXYTYPE:
                        {
				const btConeShape* coneShape = static_cast<const btConeShape*>(colisShape);
				//int upIndex = coneShape->getConeUpIndex();
				float radius = coneShape->getRadius();//+coneShape->getMargin();
				float height = coneShape->getHeight();//+coneShape->getMargin();
                                cone(radius, height, 20);
				break;
			}
			case CYLINDER_SHAPE_PROXYTYPE:
                        {
				const btCylinderShape* cylShape = static_cast<const btCylinderShape*>(colisShape);
				btVector3 halfDims = cylShape->getHalfExtentsWithMargin();
                                cylinder(halfDims.y(),halfDims.x(),10);
				break;
			}
                        case STATIC_PLANE_PROXYTYPE:
                        {
				const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(colisShape);
				btScalar planeConst = staticPlaneShape->getPlaneConstant();
				btVector3 planeNormal = staticPlaneShape->getPlaneNormal();
				drawPlane(planeConst, planeNormal);
				break;
			}
			default:
				break;
        }
		
        glPopMatrix();
    }
}
