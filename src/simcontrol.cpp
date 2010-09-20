#include "simcontrol.h"
#include "skydome.h"
#include "terrain.h"
#include "obstacles.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "pathPlan.h"
#include "utility/rngs.h"
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

simControl::simControl(QWidget* parent, simGLView* vw)
:
m_parent(parent),
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
	
	wTool = new waypointTool(ground, m_parent);
	glView->setWaypointList(wTool->getList());					// set the waypoint list to be drawn
	
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
// configure automation file method
/////////////
void simControl::runConfigFile()
{
	QSettings configFile(QDir::currentPath() + "/mission/config",QSettings::IniFormat);		// pullup the config file

	int count;
	btVector3 minSize, maxSize, worldsize;
	QVector2D yawrange;
	float step,cssize,efflimit,sprogress;
	
	long seed = configFile.value("Seed").toLongLong();	// set the random number seed
	PutSeed(seed);
	
	QString tfile = configFile.value("Terrain").toString();
	ground->openTerrain(tfile);									// load the terrain
	
	worldsize.setX(configFile.value("World_Size_X").toFloat());
	worldsize.setY(configFile.value("World_Size_Y").toFloat());
	worldsize.setZ(configFile.value("World_Size_Z").toFloat());
	ground->setTerrainSize(worldsize);							// set the size of the world, scale
	
	count = configFile.value("Obstacle_Count").toInt();
	minSize.setX(configFile.value("Obstacle_Min_X").toFloat());
	minSize.setY(configFile.value("Obstacle_Min_Y").toFloat());
	minSize.setZ(configFile.value("Obstacle_Min_Z").toFloat());
	maxSize.setX(configFile.value("Obstacle_Max_X").toFloat());
	maxSize.setY(configFile.value("Obstacle_Max_Y").toFloat());
	maxSize.setZ(configFile.value("Obstacle_Max_Z").toFloat());
	yawrange.setX(configFile.value("Obstacle_Yaw_Min").toFloat());
	yawrange.setY(configFile.value("Obstacle_Yaw_Max").toFloat());
	blocks->setParameters(count, minSize, maxSize, yawrange);	// set obstacle parameters
	blocks->generate();											// generate obstacles
	
	btVector3 roverstart(1,1,1);
	addWaypoint(1,50,50);										// add a waypoint for viewing
	newRover(roverstart);										// create a new rover
	
	step = configFile.value("Sensor_Step").toFloat();
	cssize = configFile.value("Sensor_Cspace").toFloat();
	efflimit = configFile.value("Path_Eff_Limit").toFloat();
	sprogress = configFile.value("Path_Spin_Progress").toFloat();
	
	QStringList rangeList = configFile.value("Sensor_Ranges").toString().split(",");
	for(int i=0; i<rangeList.size(); i++)
	{
		float range = rangeList[i].toFloat();
		pTool->addPath(range,step,cssize,efflimit,sprogress);				// create the paths with the parameters
	}
	
	m_iterations = configFile.value("Iterations").toInt();					// set the number of iterations to perform
	m_pathSizeMin = configFile.value("Path_Size_Min").toFloat();
	m_pathSizeMax = configFile.value("Path_Size_Max").toFloat();

	QString trialname = configFile.value("Trial_Name").toString();			// get the name of the current trial
	QDir triallocation(QDir::currentPath() + "/mission/" + trialname);		// create a folder of the trial name in the mission dir
	pTool->setTrialname(triallocation.absolutePath() + "/" + trialname);	// set the base name of the XML save file

	glView->printText(pTool->getTrialname());
	
	connect(pTool, SIGNAL(pathsFinished()), this, SLOT(runIteration()));
	connect(this, SIGNAL(genPaths()), pTool, SLOT(on_buttonGenAll_clicked()));
	
	runIteration();
}

void simControl::runIteration()
{
	if(m_iterations < 0){
		disconnect(pTool, SIGNAL(pathFinished()), this, SLOT(runIteration()));
		return;
	}
	if(blocks->areObstaclesActive()){										// wait until obstacles are at rest
		QTimer::singleShot(1000, this, SLOT(runIteration()));				// wait a second and call this method again
		return;
	}
	glView->printText(QString("iterations %1").arg(m_iterations));
	
	btVector3 tempStart;
	tempStart.setX(Randomn()*ground->terrainSize().x());					// calculate a random start position
	tempStart.setY(Randomn()*ground->terrainSize().y());		
	tempStart.setZ(ground->terrainHeightAt(tempStart));			
																			// make sure the robot starts outside of obstacles
	sr2->placeRobotAt(tempStart);
	arena->simulateStep();
	
	btVector3 tempGoal;
	float d;							
	do{																		// randomize the start and goal points
		tempGoal.setX(Randomn()*ground->terrainSize().x());					// calculate a random goal position
		tempGoal.setY(Randomn()*ground->terrainSize().y());
		d = tempStart.distance(tempGoal);
	}while(d < m_pathSizeMin || d > m_pathSizeMax);							// the straight line distance is within range
	
	tempGoal.setZ(ground->terrainHeightAt(tempGoal));						// set the Z height of the goal point
	pTool->setGoalPoint(tempGoal);
	wTool->moveWaypoint(0, tempGoal);
	glView->updateGL();
	
	m_iterations -= 1;
	emit genPaths();
}

/////////////////////////////////////////
// Rover generation functions, includes AutoNavigation and PathPlanning Tools
/////////////
void simControl::newRover(btVector3 start)
{
	if(!sr2) sr2 = new SR2rover(glView);
	
	wTool->resetStates();
	
	start.setZ(ground->terrainHeightAt(btVector3(1,1,0)));
	sr2->placeRobotAt(start);
	
	autoNav = new autoCode(sr2, wTool->getList(), m_parent);
	pTool = new pathTool(sr2, blocks, glView);
	pTool->setGoalPoint(autoNav->getCurrentWaypoint().position + btVector3(0,0,0.01));
	connect(wTool, SIGNAL(currentWaypoint(int)), autoNav, SLOT(setCurrentWaypointIndex(int)));
	connect(blocks, SIGNAL(obstaclesRemoved()), pTool, SLOT(resetPaths()));
	connect(this, SIGNAL(pathView(int)), pTool, SLOT(stepOnPath(int)));
	glView->setFocus(Qt::OtherFocusReason);
	emit roverState(true);
}
bool simControl::removeRover()
{
	emit roverState(false);
    if(sr2){
		autoNav->disconnect();
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
