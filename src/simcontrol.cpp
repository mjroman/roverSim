#include "simcontrol.h"
#include "skydome.h"
#include "terrain.h"
#include "obstacles.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "pathPlan.h"
#include "tools/waypointtool.h"
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
wTool(NULL),
pTool(NULL),
glView(vw)
{
   	arena = physicsWorld::instance(); // get the physics world object
	
	//ground = new terrain(QString(":/textures/src/textures/defaultTerrain.png"), glView);
	ground = new terrain("NULL", glView);
	
	blocks = new obstacles(ground,glView);
	
	sky = new skydome(glView);
	
	wTool = new waypointTool(ground, glView->parentWidget());
	glView->setWaypointList(wTool->getList());					// set the waypoint list to be drawn
	
	// add a few test waypoints
	addWaypoint(657,50.0,50.0);
	addWaypoint(658,1.0,14.0);
	
	connect(ground,SIGNAL(newTerrain()),blocks,SLOT(eliminate()));
	connect(ground,SIGNAL(newTerrain()),this,SLOT(removeRover()));
	connect(ground,SIGNAL(newTerrain()),this,SLOT(setAllWaypointHeights()));
	
	arena->startSimTimer();
}

simControl::~simControl()
{
	this->removeRover();
	
	if(wTool) delete wTool;
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
	
	wTool->resetStates();
	
	start.setZ(ground->terrainHeightAt(btVector3(1,1,0)));
	sr2->placeRobotAt(start);
	
	autoNav = new autoCode(sr2, wTool->getList(), parent);
	pTool = new pathTool(sr2, blocks, glView);
	pTool->setGoalPoint(autoNav->getCurrentWaypoint().position + btVector3(0,0,0.01));
	connect(blocks, SIGNAL(obstaclesRemoved()), pTool, SLOT(resetPaths()));
	connect(this, SIGNAL(pathView(int)),pTool, SLOT(stepOnPath(int)));
	glView->setFocus(Qt::OtherFocusReason);
}
bool simControl::removeRover()
{
    if(sr2){
		pTool->disconnect();
		delete pTool;
		delete autoNav;
        delete sr2;
		pTool = 0;
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
	if(pTool) pTool->show();
}
void simControl::showPathView(int dir)
{
	emit pathView(dir);
}

/////////////////////////////////////////
// Waypoint access functions
/////////////
void simControl::addWaypoint(int uuid, float x,float y, WPstate st, WPscience sc)
{
	WayPoint wp;
	wp.uuid = uuid;
	wp.position.setX(x);
	wp.position.setY(y);
	wp.position.setZ(ground->terrainHeightAt(wp.position));
	wp.state = st;
	wp.science = sc;
	wTool->addWaypoint(wp);
}
void simControl::showWaypointTool()
{
	wTool->show();
}
