#include <QtGui>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

#include "simglview.h"
#include "utility/glshapes.h"
#include "simGLObject.h"

// Use if Bullet frameworks are used
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btAlignedObjectArray.h>

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

void simGLView::printText(QString st)
{
	emit outputText(st);
}

void simGLView::toggleDrawing()
{
	static bool x = false;
	if(x){
		m_timer->stop();
		emit outputText("Drawing Paused");
	}
	else{
		m_timer->start(100);
		emit outputText("Resumed Drawing");
	}
	x = !x;
}
simGLView::~simGLView()
{
	m_timer->stop();
	delete m_timer;
	
    int i=0;
    for(i=0;i<3;i++) this->deleteTexture(m_textureList[i]);
	m_textureList.clear();
    delete m_eye;
    qDebug("deleting glView");
	qDebug("#################################################################");
}

void simGLView::registerGLObject(simGLObject *obj)
{
	int x = renderList.indexOf(obj);
    if(x == -1) renderList.append(obj);
}

void simGLView::unregisterGLObject(simGLObject *obj)
{
    int x = renderList.indexOf(obj);
    if(x != -1) renderList.removeAt(x);
}

void simGLView::loadTextures()
{
	//m_textureList << this->bindTexture(QPixmap(QString("/Users/mattroman/Documents/code/roverSim/src/textures/domePan1.png")),GL_TEXTURE_2D,GL_RGBA);
	m_textureList << this->bindTexture(QPixmap(QString("/Users/mattroman/Documents/code/roverSim/src/textures/skydome3.bmp")),GL_TEXTURE_2D,GL_RGBA);
    m_textureList << this->bindTexture(QPixmap(QString(":/textures/src/textures/pancam.png")),GL_TEXTURE_2D,GL_RGBA);
    m_textureList << this->bindTexture(QPixmap(QString(":/textures/src/textures/solarPanel2.png")),GL_TEXTURE_2D,GL_RGBA);
	m_textureList << this->bindTexture(QPixmap(QString(":/textures/src/textures/spark.png")),GL_TEXTURE_2D,GL_RGBA);
	m_textureList << this->bindTexture(QPixmap(QString(":/textures/src/textures/mud.png")),GL_TEXTURE_2D,GL_RGBA);
	
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
}

GLuint simGLView::getTexture(int n)
{
    return m_textureList[n];
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
    glEnable(GL_CULL_FACE);

	// fog
	//glEnable(GL_FOG); // DON'T FORGET TO SET THE CLEAR COLOR
	GLfloat fogColor[4] = {0.85, 0.95, 0.95, .9};
	glFogi (GL_FOG_MODE, GL_LINEAR);
	glFogfv (GL_FOG_COLOR, fogColor);
	glFogf (GL_FOG_DENSITY, 0.8);
	glHint (GL_FOG_HINT, GL_FASTEST);
	glFogf (GL_FOG_START, 40);	// ONLY USED FOR LINEAR FOG EQUATION
	glFogf (GL_FOG_END, 200);	// ONLY USED FOR LINEAR FOG EQUATION
	// volumetric fog testing
	// glFogf(GL_FOG_START, 0.);
	// 	glFogf(GL_FOG_END, 1.0);
	// 	glFogi(GL_FOG_COORD_SRC, GL_FOG_COORD);
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
		//glClearColor(0.5,0.5,0.5,0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();

        m_eye->cameraUpdate();

        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lightAmbient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZero);
        glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPos);

        glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOne);
        glLightfv(GL_LIGHT1, GL_POSITION, lightOnePos);

		drawObstacles();
		drawWaypoints();
		
        for(int i=0;i<renderList.size();i++)
            renderList[i]->renderGLObject();
		
		if(m_pickObject) drawPickingHalo();
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
	glLineWidth(1.0);
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

void simGLView::toggleFog()
{
	static int i=1;
	if(i>5) {
		i=0;
		glDisable(GL_FOG);
	}
	else {
		glEnable(GL_FOG);
		//glFogf(GL_FOG_DENSITY,0.1*i);
		glFogf (GL_FOG_END, 80*i);
	}
	i++;
}

//////////////////////////////////////////////////////////////////////////////////////////
// user input
/////////////
void simGLView::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePoint = event->pos();
	if(this->hasMouseTracking() && event->modifiers() == Qt::NoModifier) 
		emit dropPicked();
	else if(!this->hasMouseTracking() && event->modifiers() & Qt::AltModifier) 
		emit pickingVector(m_eye->cameraPosition(),mouseRayTo(event->pos()));
}

void simGLView::mouseMoveEvent(QMouseEvent *event)
{
    QPoint del;
	
    del = event->pos() - m_lastMousePoint;
	m_lastMousePoint = event->pos();
	
	if(this->hasMouseTracking() && event->modifiers() == Qt::NoModifier){
		emit movingVector(m_eye->cameraPosition(),mouseRayTo(event->pos()));
	}
    else 
		m_eye->cameraMouseMove(del,event);
}

void simGLView::wheelEvent(QWheelEvent *event)
{
	if(this->hasMouseTracking() && event->modifiers() & Qt::AltModifier)
		emit spinPicked((float)event->delta());
	else if(this->hasMouseTracking() && event->modifiers() & Qt::ControlModifier)
		emit loftPicked((float)event->delta());
	else
    	m_eye->cameraMouseWheel((float)event->delta());
}

btVector3 simGLView::mouseRayTo(QPoint mousePoint)
{
	float tanfov = tanf(0.5*DEGTORAD(35/m_viewAngle));
	
	btVector3 rayFrom = m_eye->cameraPosition();
	btVector3 rayForward = (m_eye->cameraDirection() - m_eye->cameraPosition());
	rayForward.normalize();
	rayForward *= FARCLIPPING;
	
	btVector3 hor = rayForward.cross(m_eye->cameraUpDirection());
	hor.normalize();
	btVector3 vertical = hor.cross(rayForward);
	vertical.normalize();
	
	hor *= 2.f * FARCLIPPING * tanfov;
	vertical *= 2.f * FARCLIPPING * tanfov;

	float aspect;

	if (this->width() > this->height()) {
		aspect = this->width() / (float)this->height();
		hor*=aspect;
	} 
	else {
		aspect = this->height() / (float)this->width();
		vertical*=aspect;
	}
	
	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/float(this->width());
	btVector3 dVert = vertical * 1.f/float(this->height());
	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	
	rayTo += mousePoint.x() * dHor;
	rayTo -= mousePoint.y() * dVert;
	return rayTo;
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

// draws all the obstacles in the world in Red
void simGLView::drawObstacles()
{
    btScalar	glm[16];
	btAlignedObjectArray<btCollisionObject*> *obstacleArray;
	obstacleArray = arena->getObstacleObjectArray();
	
	glColor3f(1.0f,0.0f,0.0f);
    //glColor3f(0.02f,0.52f,0.51f);	// tron blue
	//glColor3f(0.1f,0.0f,0.5f); // dark blue
	//glColor3f(0.7,0.0,0.7);	// dark purple
    //glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    //glMaterialfv(GL_FRONT, GL_EMISSION, obstacleEmission);
    for(int i=0; i < obstacleArray->size(); i++){
        btCollisionObject*	colisObject = obstacleArray->at(i);
		
		// test colisobject if rigid body
		btRigidBody* body = btRigidBody::upcast(colisObject);

		if(body && body->getMotionState())
		{
			btDefaultMotionState* objMotionState = (btDefaultMotionState*)body->getMotionState();
			objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);
		}
		else
			colisObject->getWorldTransform().getOpenGLMatrix(glm);

        btCollisionShape* colisShape = colisObject->getCollisionShape();
		
        glPushMatrix();
        glMultMatrixf(glm);

		switch (colisShape->getShapeType()) {
			case BOX_SHAPE_PROXYTYPE: 
			{
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
				if (colisShape->isConvex())
				{
					ShapeCache* sc = (ShapeCache*)colisShape->getUserPointer();
					if(!sc) break;
					btShapeHull* hull = &sc->m_shapehull;
					
					if (hull->numTriangles() > 0)
					{
						int index = 0;
						const unsigned int* idx = hull->getIndexPointer();
						const btVector3* vtx = hull->getVertexPointer();

						glBegin (GL_TRIANGLES);

						for (int i = 0; i < hull->numTriangles (); i++)
						{
							int i1 = index++;
							int i2 = index++;
							int i3 = index++;
							btAssert(i1 < hull->numIndices () &&
								i2 < hull->numIndices () &&
								i3 < hull->numIndices ());

							int index1 = idx[i1];
							int index2 = idx[i2];
							int index3 = idx[i3];
							btAssert(index1 < hull->numVertices () &&
								index2 < hull->numVertices () &&
								index3 < hull->numVertices ());

							btVector3 v1 = vtx[index1];
							btVector3 v2 = vtx[index2];
							btVector3 v3 = vtx[index3];
							btVector3 normal = (v2-v1).cross(v3-v1);
							normal.normalize ();
							glNormal3f(normal.getX(),normal.getY(),normal.getZ());
							glVertex3f (v1.x(), v1.y(), v1.z());
							glVertex3f (v2.x(), v2.y(), v2.z());
							glVertex3f (v3.x(), v3.y(), v3.z());

						}
						glEnd ();
					}
				}
				break;
        }	
        glPopMatrix();
    }	
}

void simGLView::drawWaypoints()
{
	int i;
	
	for(i=0;i<WPlist->size();i++){
		WayPoint wp = WPlist->at(i);
		switch (wp.state){
			case WPstateNew:
				glColor3f(1,1,1);
				break;
			case WPstateOld:
				glColor3f(0.7,0.0,0.7);
				break;
			case WPstateCurrent:
				glColor3f(0,1,0);
				break;
			case WPstateSkipped:
				glColor3f(1,1,0);
				break;
			default:
				glColor3f(0,1,1);
				break;
		}
		
		glPushMatrix();
		glTranslatef(wp.position.x(),wp.position.y(),wp.position.z());
		box(0.1,0.1,1);
		glPopMatrix();
	}
}

void simGLView::drawFrame()
{
        float fSize = 2;
        glLineWidth(2.0);
        glBegin(GL_LINES);

        // x red
        glColor3f(255.f,0,0);
		glVertex3d(0,0,0);
		glVertex3d(fSize,0,0);
        //btVector3 vX = tr*btVector3(fSize,0,0);	
        //glVertex3d(tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
        //glVertex3d(vX.getX(), vX.getY(), vX.getZ());

        // y green
        glColor3f(0,255.f,0);
		glVertex3d(0,0,0);
		glVertex3d(0,fSize,0);
        // btVector3 vY = tr*btVector3(0,fSize,0);
        // glVertex3d(tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
        // glVertex3d(vY.getX(), vY.getY(), vY.getZ());
        
        // z blue
        glColor3f(0,0,255.f);
		glVertex3d(0,0,0);
        glVertex3d(0,0,fSize);
		// btVector3 vZ = tr*btVector3(0,0,fSize);
        // glVertex3d(tr.getOrigin().getX(), tr.getOrigin().getY(), tr.getOrigin().getZ());
        // glVertex3d(vZ.getX(), vZ.getY(), vZ.getZ());

        glEnd();
}

void simGLView::drawPickingHalo()
{
	glPushMatrix();
	glTranslatef(m_pickObject->hitPoint.x(),m_pickObject->hitPoint.y(),m_pickObject->hitPoint.z());
	glColor4f(1,1,0.2,0.25);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	// due to translucency of halo culling must be done and the object must be drawn twice for front and back culling
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_BACK);	
	conePoint(0.7,2.0,20);
	glCullFace(GL_FRONT);
	conePoint(0.7,2.0,20);
	glCullFace(GL_BACK);
	//glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
	glPopMatrix();
	
	btScalar	glm[16];
	btDefaultMotionState* objMotionState = (btDefaultMotionState*)m_pickObject->rigidbody->getMotionState();
	objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);
	
	glDisable(GL_LIGHTING);
	glPushMatrix();
    glMultMatrixf(glm);

	// draw an xyz axis frame
	drawFrame();
	
	// draw a yellow mark on the selected axis of rotation
	glLineWidth(5.0);
	glColor3f(1,1,0);
	glBegin(GL_LINES);
	btVector3 ax = m_pickObject->rotAxis * 2.0;
	glVertex3fv(ax.m_floats);
	glVertex3f(0,0,0);
	glEnd();
	
	glPopMatrix();
	glEnable(GL_LIGHTING);
}