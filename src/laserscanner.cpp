#include "laserscanner.h"

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

static void tube(float rad,float width)
{
        int i;

        glBegin(GL_QUAD_STRIP);
        for(i=0;i<=360;i+=20){
                glNormal3f(0, sin(DEGTORAD(i)), cos(DEGTORAD(i)));
                glVertex3f(0,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
                glVertex3f(width,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
        }
        glEnd();

        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(1, 0, 0);
        glVertex3f(width,0,0);
        for(i=360;i>=0;i-=20){
                glVertex3f(width,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
        }
        glEnd();

        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(-1, 0, 0);
        glVertex3f(0,0,0);
        for(i=360;i>=0;i-=20){
                glVertex3f(0,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
        }
        glEnd();
}
static void hokuyoCase(void)
{
        glPushMatrix();
        glColor3f(0, 0, 0);
        glRotated(-90, 0, 1, 0);
        glTranslated(-0.016, 0, 0);
        tube(0.02,0.029);
        glColor3f(0, 1, 0);
        glBegin(GL_TRIANGLES);
        glNormal3f(1, 0, 0);
        glVertex3f(0.0295,0,-0.005);
        glVertex3f(0.0295,0.02,0);
        glVertex3f(0.0295,0,0.005);
        glEnd();
        glColor3f(1, 1, 1);
        glBegin(GL_QUAD_STRIP);
        glNormal3f(0, 0, 1);
        glVertex3f(-0.041, -0.025, 0.025);
        glVertex3f(0, -0.025, 0.025);
        glVertex3f(-0.041, 0.025, 0.025);
        glVertex3f(0, 0.025, 0.025);
        glNormal3f(0, 1, 0);
        glVertex3f(-0.041, 0.025, -0.025);
        glVertex3f(0, 0.025, -0.025);
        glNormal3f(0, 0, -1);
        glVertex3f(-0.041, -0.025, -0.025);
        glVertex3f(0, -0.025, -0.025);
        glNormal3f(0, -1, 0);
        glVertex3f(-0.041, -0.025, 0.025);
        glVertex3f(0, -0.025, 0.025);
        glEnd();
        glBegin(GL_QUADS);
        glNormal3f(1, 0, 0);
        glVertex3f(0,-0.025,0.025);
        glVertex3f(0,-0.025,-0.025);
        glVertex3f(0,0.025,-0.025);
        glVertex3f(0,0.025,0.025);
        glNormal3f(-1, 0, 0);
        glVertex3f(-0.041,-0.025,0.025);
        glVertex3f(-0.041,0.025,0.025);
        glVertex3f(-0.041,0.025,-0.025);
        glVertex3f(-0.041,-0.025,-0.025);
        glEnd();
        glPopMatrix();
}


laserScanner::laserScanner(btTransform xForm,float th,float delta,float offset)
:
m_rangeData(NULL)
{
    maxRange = 3.0;
    minRange = 0.1;
    theta = th;
    dTheta = delta;
    offsetTheta = offset;
    arena = physicsWorld::instance(); // get the physics world object

    m_rangeData = NULL;
    m_beamVector = NULL;
    configure();

    m_displayBeam = false;
    m_scanTrans = xForm;

    // generate the openGL display list
    m_aScanner = glGenLists(1);
    glNewList(m_aScanner, GL_COMPILE);
    hokuyoCase();
    glEndList();
}

laserScanner::~laserScanner()
{
    glDeleteLists(m_aScanner,1);
    delete [] m_rangeData;
    delete [] m_beamVector;
}

void laserScanner::configure()
{
    int newSize = theta / dTheta;
    float *newData = new float[newSize];
    btVector3 *newVector = new btVector3[newSize];

    for(int i=0;i<newSize;i++){
        float phi = (HALFPI - offsetTheta) - (theta/2 - i*dTheta);
        newVector[i].setX(cos(phi));
        newVector[i].setY(sin(phi));
        newVector[i].setZ(0);
        newData[i] = maxRange;
    }

    float *oldData = m_rangeData;
    btVector3 *oldVector = m_beamVector;
    m_rangeData = newData;
    m_beamVector = newVector;
    m_dataSize = newSize;
    if(oldData) delete [] oldData;
    if(oldVector) delete [] oldVector;
}

int laserScanner::getData(float *data)
{
    data = m_rangeData;
    return m_dataSize;
}

void laserScanner::update(btTransform botTrans)
{
    btTransform tTrans = botTrans * m_scanTrans;
    btVector3 rayFrom = tTrans.getOrigin();

    for(int i=0;i<m_dataSize;i++){
        btVector3 rayTo = tTrans(m_beamVector[i] * maxRange);
        btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
        arena->getDynamicsWorld()->rayTest(rayFrom,rayTo,rayCallback);
        if(rayCallback.hasHit()){
            btVector3 hitPoint = rayCallback.m_hitPointWorld;
            hitPoint -= rayFrom;
            m_rangeData[i] = hitPoint.length();
        }
        else m_rangeData[i] = maxRange;
    }
}

void laserScanner::drawLaser(btTransform botTrans,bool bodyOn)
{
    int i;
    btScalar    glm[16];

    btTransform tTrans = botTrans * m_scanTrans;
    tTrans.getOpenGLMatrix(glm);

    glPushMatrix();
    glMultMatrixf(glm);
    // draw laser body
    if(bodyOn) glCallList(m_aScanner);

    glDisable(GL_LIGHTING);

    if(m_displayBeam) {
        glColor3f(1.,0.3,0.3);
        glBegin(GL_LINES);
        for(i=0;i<m_dataSize;i++){
            btVector3 beam = m_beamVector[i] * m_rangeData[i];
            glVertex3f(0,0,0);
            glVertex3f(beam.x(),beam.y(),beam.z());
        }
        glEnd();
    }
    else {
        glColor3f(0.,1.,0.);
        glPointSize(3);
        glBegin(GL_POINTS);
        for(i=0;i<m_dataSize;i++){
            btVector3 beam = m_beamVector[i] * m_rangeData[i];
            glVertex3f(beam.x(),beam.y(),beam.z());
        }
        glEnd();
    }
    glEnable(GL_LIGHTING);
    glPopMatrix();
}
