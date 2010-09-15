#include "simcontrol.h"
#include "skydome.h"
#include "terrain.h"
#include "obstacles.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "pathPlan.h"
#include "tools/pathtool.h"
#include "simGLView.h"

// Use if Bullet Frameworks are added
#include <BulletCollision/btBulletCollisionCommon.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>

#define LAYOUTHEADER	"RoverSim Obstacle Layout File"

simControl::simControl(simGLView* vw)
:
sky(NULL),
sr2(NULL),
autoNav(NULL),
m_pTool(NULL),
glView(vw)
{
   	arena = physicsWorld::instance(); // get the physics world object
	
	//ground = new terrain(QString(":/textures/src/textures/defaultTerrain.png"), glView);
	ground = new terrain("NULL", glView);
	
	blocks = new obstacles(ground,glView);
	
	sky = new skydome(glView);
	
	// add a few test waypoints
	addWaypointAt(657,50.0,50.0);
	addWaypointAt(658,1.0,14.0);
	
	glView->setWaypointList(&waypointList);
	
	connect(ground,SIGNAL(newTerrain()),blocks,SLOT(eliminate()));
	connect(ground,SIGNAL(newTerrain()),this,SLOT(removeRover()));
	connect(ground,SIGNAL(newTerrain()),this,SLOT(setWaypointGroundHeight()));
	blocks->generate();
	
	arena->startSimTimer();
}

simControl::~simControl()
{
	this->removeRover();
	
	if(sky) delete sky;
	if(blocks) delete blocks;
	if(ground) delete ground;
}

/////////////////////////////////////////
// Simulation timing and gravity functions
/////////////
void simControl::stepTimevals(float tStep,float fixedtStep,int subSteps)
{
	//qDebug("tStep=%f fixed=%f sub=%d",tStep,fixedtStep,subSteps);
    arena->simTimeStep = tStep;
    arena->simFixedTimeStep = fixedtStep;
    arena->simSubSteps = subSteps;
}
void simControl::pauseSim()
{
	if(arena->isIdle()){
		arena->startSimTimer();
		glView->printText("Resumed Simulation");
	}
	else{
		arena->stopSimTimer();
		glView->printText("Simulation Paused");
	}
		
}
void simControl::setGravity(btVector3 g)
{
	arena->setGravity(g);
}

/////////////////////////////////////////
// Rover generation functions, includes AutoNavigation and PathPlanning Tools
/////////////
void simControl::newRover(QWidget* parent, btVector3 start)
{
	if(!sr2) sr2 = new SR2rover(glView);
	
	this->resetWaypointStates();
	
	start.setZ(ground->terrainHeightAt(btVector3(1,1,0)));
	sr2->placeRobotAt(start);
	
	autoNav = new autoCode(sr2, &waypointList, parent);
	m_pTool = new pathTool(sr2, blocks, glView);
	m_pTool->setGoalPoint(autoNav->getCurrentWaypoint().position + btVector3(0,0,0.01));
	connect(blocks, SIGNAL(obstaclesRemoved()), m_pTool, SLOT(resetPaths()));
	connect(this, SIGNAL(pathView(int)),m_pTool, SLOT(stepOnPath(int)));
	glView->setFocus(Qt::OtherFocusReason);
}
bool simControl::removeRover()
{
    if(sr2){
		m_pTool->disconnect();
		delete m_pTool;
		delete autoNav;
        delete sr2;
		m_pTool = 0;
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
void simControl::showPathTool()
{
	if(m_pTool) m_pTool->show();
}
void simControl::showPathView(int dir)
{
	emit pathView(dir);
}

/////////////////////////////////////////
// Waypoint editing functions
/////////////
void simControl::setWaypointGroundHeight()				// sets the waypoint Z height based on height of terrain at wp location
{
	int i=0;
	while(i < waypointList.size()){
		// set the z position of the waypoint for visability
		waypointList[i].position.setZ(ground->terrainHeightAt(waypointList[i].position));
		i++;
	}
}
void simControl::editWaypoint(int index)				// if a waypoint is moved recalculate its Z position
{
	WayPoint wp = waypointList[index];
	wp.position.setZ(ground->terrainHeightAt(wp.position));
	waypointList.replace(index,wp);
}
void simControl::addWaypointAt(WayPoint wp, int index)
{
	wp.position.setZ(ground->terrainHeightAt(wp.position));
	if(index<0) waypointList << wp;
	else waypointList.insert(index,wp);
}
void simControl::addWaypointAt(int uuid, float x,float y, WPstate st, WPscience sc,int i)
{
	WayPoint wp;
	wp.uuid = uuid;
	wp.position.setX(x);
	wp.position.setY(y);
	wp.position.setZ(ground->terrainHeightAt(wp.position));
	wp.state = st;
	wp.science = sc;
	if(i<0)waypointList << wp;
	else waypointList.insert(i,wp);
}
void simControl::resetWaypointStates()					// reset all the waypoint states to NEW
{
	WayPoint wp;
	for(int i = 0; i < waypointList.size(); ++i)
	{
		wp = waypointList[i];
		wp.state = WPstateNew;
		waypointList.replace(i,wp);
	}
}