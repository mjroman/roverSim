#include "simcontrol.h"
#include "skydome.h"
#include "terrain.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "simGLView.h"
#include "utility/rngs.h"


// Use if Bullet Frameworks are added
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

simControl::simControl(simGLView* vw)
	:	ground(NULL),sky(NULL),sr2(NULL),autoNav(NULL),glView(vw),
m_obstType(),
m_obstCount(50),
m_dropHeight(5),
m_minObstYaw(0),
m_maxObstYaw(45),
m_obstDensity(5)
{
	qDebug("simControl startup");
   	arena = physicsWorld::instance(); // get the physics world object
	
	m_minObstSize = btVector3(0.1,0.1,0.25);
	m_maxObstSize = btVector3(1,1,0.5);
	
	ground = new terrain(QString(":/textures/src/textures/defaultTerrain.png"), glView);
	this->generateObstacles();
	
	//sky = new skydome(glView);
	
	simTimer = new QTimer(this);
    connect(simTimer, SIGNAL(timeout()), this, SLOT(stepSim()));
    this->startSimTimer(10);
}

simControl::~simControl()
{
	simTimer->stop();
	delete simTimer;
	if(autoNav) delete autoNav;
	if(sr2) delete sr2;
	if(sky) delete sky;
	if(ground) delete ground;
	qDebug("deleting simControl");
}

void simControl::startSimTimer(int msec)
{
    simTimer->start(msec);
}

void simControl::stopSimTimer()
{
    simTimer->stop();
}

void simControl::stepSim()
{
    if(sr2) sr2->updateRobot();
    arena->simulatStep();
}

void simControl::stepTimevals(float tStep,float fixedtStep,int subSteps)
{
	//qDebug("tStep=%f fixed=%f sub=%d",tStep,fixedtStep,subSteps);
    arena->simTimeStep = tStep;
    arena->simFixedTimeStep = fixedtStep;
    arena->simSubSteps = subSteps;
}

void simControl::pauseSim()
{
	arena->toggleIdle();
}

void simControl::setGravity(btVector3 g)
{
	arena->setGravity(g);
}

void simControl::openNewGround(QString filename)
{
	this->removeRover();
	ground->openTerrain(filename);
	
	// generate new obstacles
	this->generateObstacles();
}

void simControl::flattenGround()
{
	this->removeRover();
	ground->terrainFlatten();
	
	// generate new obstacles
	this->generateObstacles();
}

void simControl::rescaleGround(btVector3 scale)
{
	this->removeRover();
	ground->terrainRescale(scale);
	
	// generate new obstacles
	this->generateObstacles();
}

void simControl::removeObstacles()
{
	arena->deleteGroup(OBSTACLE_GROUP);
}

void simControl::generateObstacles()
{
    btVector3   tempPlace,tempSize;
    float   alphaYaw,volume;
    int i;
	
    arena->deleteGroup(OBSTACLE_GROUP);
	
    if(m_obstCount == 0) return;
	qDebug("generating new obstacles %d",m_obstCount);
    for(i=0;i<m_obstCount;i++)
    {
        tempPlace.setX(Randomn()*arena->worldSize().x());
        tempPlace.setY(Randomn()*arena->worldSize().y());
		// keep all obstacles away from rover start position
        if(tempPlace.x() < 5 && tempPlace.y() < 5){ tempPlace.setX(5);} 
        tempPlace.setZ(ground->maxHeight() + m_dropHeight);
		
        tempSize.setX(m_minObstSize.x() + Randomn()*(m_maxObstSize.x() - m_minObstSize.x()));
        tempSize.setY(m_minObstSize.y() + Randomn()*(m_maxObstSize.y() - m_minObstSize.y()));
        tempSize.setZ(m_minObstSize.z() + Randomn()*(m_maxObstSize.z() - m_minObstSize.z()));
        alphaYaw = (m_minObstYaw + Randomn()*(m_maxObstYaw - m_minObstYaw));
		
        switch(m_obstType)
        {
            case 0:
                arena->createObstacleShape(BOX_SHAPE_PROXYTYPE,tempSize);
                // volume = L x W x H
                volume = 8 * tempSize.x() * tempSize.y() * tempSize.z();
                break;
            case 1:
                arena->createObstacleShape(SPHERE_SHAPE_PROXYTYPE,tempSize);
                // volume  = 4/3 x PI x r^3
                volume = 4 * PI * ((tempSize.x() * tempSize.x() * tempSize.x()) / 3);
                break;
            case 2:
                arena->createObstacleShape(CONE_SHAPE_PROXYTYPE,tempSize);
                // volume = 1/3 x PI x r^2 x H
                volume = (PI * tempSize.x() * tempSize.x() * tempSize.y()) / 3;
                break;
            case 3:
                tempSize.setZ(tempSize.y());
                arena->createObstacleShape(CYLINDER_SHAPE_PROXYTYPE,tempSize);
                // volume = PI x r^2 x H
                volume = PI * tempSize.y() * tempSize.y() *tempSize.x();
                break;
            default:
                arena->createObstacleShape(BOX_SHAPE_PROXYTYPE,tempSize);
                volume = tempSize.x() * tempSize.y() * tempSize.z();
                break;
        }
        float mass = m_obstDensity * (volume);
		
        arena->placeObstacleShapeAt(tempPlace,alphaYaw,mass);
    }
}

bool simControl::removeRover()
{
    if(sr2){
		delete autoNav;
        delete sr2;
		autoNav = 0;
        sr2 = 0;
        return true;
    }
    return false;
}

void simControl::newRover()
{
	if(autoNav) delete autoNav;
	if(!ground) return;
	qDebug("new rover");
	if(!sr2) sr2 = new SR2rover(glView);
	sr2->waypointList.clear();
	
	// add a few test waypoints
	sr2->addWaypointAt(657,90.0,90.0);
	//sr2->addWaypointAt(32,10.0,2.0);
	//sr2->addWaypointAt(535,25.0,15.0);
	//sr2->addWaypointAt(657,6.0,14.0);
	this->setWaypointGroundHeight();
	
	sr2->placeRobotAt(btVector3(1,1,ground->terrainHeightAt(btVector3(1,1,0))));
	autoNav = new autoCode(sr2);
}

void simControl::setWaypointGroundHeight()
{
	if(!sr2->waypointList.size()) return; // return if no waypoints
	
	int i=0;
	while(i < sr2->waypointList.size()){
		btVector3 position(sr2->waypointList[i].position.x,sr2->waypointList[i].position.y,0);
		// set the z position of the waypoint for visability
		sr2->waypointList[i].position.z = ground->terrainHeightAt(position);
		i++;
	}
}