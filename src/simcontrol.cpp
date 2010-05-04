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
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>

simControl::simControl(simGLView* vw)
	:	ground(NULL),sky(NULL),sr2(NULL),autoNav(NULL),glView(vw),
m_obstType(0),
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
	
	// add a few test waypoints
	addWaypointAt(657,30.0,30.0);
	addWaypointAt(658,1.0,14.0);
	
	glView->setWaypointList(&waypointList);
	
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

/////////////////////////////////////////
// Simulation timing and gravity functions
/////////////
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

/////////////////////////////////////////
// Terrain access functions
/////////////
void simControl::openNewGround(QString filename)
{
	this->removeRover();
	ground->openTerrain(filename);
	
	// generate new obstacles
	this->generateObstacles();
	// restore waypoint Z height
	this->setWaypointGroundHeight();
}
void simControl::flattenGround()
{
	this->removeRover();
	ground->terrainFlatten();
	
	// generate new obstacles
	this->generateObstacles();
	// restore waypoint Z height
	this->setWaypointGroundHeight();
}
void simControl::rescaleGround(btVector3 scale)
{
	this->removeRover();
	ground->terrainRescale(scale);
	
	// generate new obstacles
	this->generateObstacles();
	// restore waypoint Z height
	this->setWaypointGroundHeight();
}

/////////////////////////////////////////
// Obstacle generation functions
/////////////
void simControl::generateObstacles()
{
    btVector3   tempPlace,tempSize;
    float   alphaYaw,volume;
    int i;
	
	arena->deleteGhostGroup();
    arena->deleteObstacleGroup();
	
    if(m_obstCount == 0) return;
	//qDebug("generating new obstacles %d",m_obstCount);
    for(i=0;i<m_obstCount;i++)
    {
        tempPlace.setX(Randomn()*arena->worldSize().x());
        tempPlace.setY(Randomn()*arena->worldSize().y());
		tempPlace.setZ(0);
		// keep all obstacles away from rover start position
        if(tempPlace.x() < 5 && tempPlace.y() < 5){ tempPlace.setX(5);} 
        //tempPlace.setZ(ground->maxHeight() + m_dropHeight);
		tempPlace.setZ(ground->terrainHeightAt(tempPlace) + m_dropHeight);
		
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
				qDebug("default obstacle");
                arena->createObstacleShape(BOX_SHAPE_PROXYTYPE,tempSize);
                volume = tempSize.x() * tempSize.y() * tempSize.z();
                break;
        }
        float mass = m_obstDensity * (volume);
		
        arena->placeObstacleShapeAt(tempPlace,alphaYaw,mass);
    }
}
void simControl::removeObstacles()
{
	arena->deleteGhostGroup();
	arena->deleteObstacleGroup();
}
void simControl::generateCSpace()
{
	int i;
	btAlignedObjectArray<btCollisionObject*>* obstArray = arena->getObstacleObjectArray();
	
	arena->deleteGhostGroup();
	// loop through all obstacle rigid bodies
	for(i=0;i<obstArray->size();i++){
		btCollisionObject*	colisObject = obstArray->at(i);

	// test colisobject if rigid body
		if(colisObject->getInternalType() == btCollisionObject::CO_RIGID_BODY)
		{
			// create CSpace
			// check if object is in-active
			if(colisObject->isActive()) continue;
			arena->createGhostShape(colisObject);
		}
	}
}
/////////////////////////////////////////
// Rover generation functions
/////////////
void simControl::newRover(QWidget* parent)
{
	if(autoNav) delete autoNav;
	if(!ground) return;
	qDebug("new rover");
	if(!sr2) sr2 = new SR2rover(glView);
	
	this->setWaypointGroundHeight();
	this->resetWaypointStates();
	
	sr2->placeRobotAt(btVector3(1,1,ground->terrainHeightAt(btVector3(1,1,0))));
	autoNav = new autoCode(sr2, &waypointList, parent);
	glView->setFocus(Qt::OtherFocusReason);
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
void simControl::showNavTool()
{
	if(autoNav) autoNav->show();
}

/////////////////////////////////////////
// Waypoint editing functions
/////////////
// sets the waypoint Z height based on height of terrain at wp location
void simControl::setWaypointGroundHeight()
{
	int i=0;
	while(i < waypointList.size()){
		btVector3 position(waypointList[i].position.x,waypointList[i].position.y,0);
		// set the z position of the waypoint for visability
		waypointList[i].position.z = ground->terrainHeightAt(position);
		i++;
	}
}
// if a waypoint is moved recalculate its Z position
void simControl::editWaypoint(int index)
{
	WayPoint wp = waypointList[index];
	wp.position.z = ground->terrainHeightAt(btVector3(wp.position.x,wp.position.y,0));
	waypointList.replace(index,wp);
}
void simControl::addWaypointAt(WayPoint wp, int index)
{
	wp.position.z = ground->terrainHeightAt(btVector3(wp.position.x,wp.position.y,0));
	if(index<0)waypointList << wp;
	else waypointList.insert(index,wp);
}
void simControl::addWaypointAt(int uuid, float x,float y, WPstate st, WPscience sc,int i)
{
	WayPoint wp;
	wp.uuid = uuid;
	wp.position.x = x;
	wp.position.y = y;
	wp.position.z = ground->terrainHeightAt(btVector3(x,y,0));
	wp.state = st;
	wp.science = sc;
	if(i<0)waypointList << wp;
	else waypointList.insert(i,wp);
}
// reset all the waypoint states to NEW
void simControl::resetWaypointStates()
{
	WayPoint wp;
	for(int i = 0; i < waypointList.size(); ++i)
	{
		wp = waypointList[i];
		wp.state = WPstateNew;
		waypointList.replace(i,wp);
	}
}