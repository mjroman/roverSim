#include "simcontrol.h"
#include "skydome.h"
#include "terrain.h"
#include "obstacles.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "pathPlan.h"
#include "cSpace.h"
#include "utility/rngs.h"
#include "tools/waypointtool.h"
#include "tools/pathtool.h"
#include "simGLView.h"
#include <QFile>

// Use if Bullet Frameworks are added
#include <BulletCollision/btBulletCollisionCommon.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>

#define LAYOUTHEADER	"RoverSim Obstacle Layout File"

simControl::simControl(MainGUI* parent, simGLView* glView)
:
m_parent(parent),
sky(NULL),
sr2(NULL),
autoNav(NULL),
wTool(NULL),
pTool(NULL),
m_view(glView)
{
   	arena = physicsWorld::instance(); // get the physics world object
	
	//ground = new terrain(QString(":/textures/src/textures/defaultTerrain.png"), m_view);
	ground = new terrain("NULL", m_view);
	
	blocks = new obstacles(ground,m_view);
	
	sky = new skydome(m_view);
	
	wTool = new waypointTool(ground, m_parent);
	m_view->setWaypointList(wTool->getList());					// set the waypoint list to be drawn
	
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
		m_parent->printText("Resumed Simulation");
	}
	else{
		arena->stopSimTimer();
		m_parent->printText("Simulation Paused");
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
	
	long seed = configFile.value("Seed").toLongLong();
	
	QString trialname(configFile.value("Trial_Name").toString());			// Create a directory of the tiral including the random seed
	QString trialDir(trialname + "_" + QString::number(seed));
	
	if(m_trialLocation.exists(QDir::currentPath() + "/mission/" + trialDir)){
		seed = QTime::currentTime().msec();									// set the seed to the current time in milliseconds
		trialDir = QString(trialname + "_%1").arg(seed);
	}
	m_trialLocation.mkdir(QDir::currentPath() + "/mission/" + trialDir);	// create a folder of the trial name in the mission dir
	m_trialLocation.cd(QDir::currentPath() + "/mission/" + trialDir);		// move to the new location
	
	PutSeed(seed);															// set the random number seed
	
	QString tfile = configFile.value("Terrain").toString();
	ground->openTerrain(tfile);											// load the terrain
	
	btVector3 worldsize;
	worldsize.setX(configFile.value("World_Size_X").toFloat());
	worldsize.setY(configFile.value("World_Size_Y").toFloat());
	worldsize.setZ(configFile.value("World_Size_Z").toFloat());
	ground->setTerrainSize(worldsize);										// set the size of the world, scale
	
	QString countString = configFile.value("Obstacle_Count").toString();
 	if(countString.contains(":",Qt::CaseInsensitive)){
		QStringList densityRange = countString.split(":");
		if(densityRange.size() < 2){
			m_parent->printText("Config file - Incorrect Density Format\n");
			return;
		}
		for(int i=densityRange[0].toInt(); i <= densityRange[2].toInt(); i += densityRange[1].toInt())
			m_densityFields << i;
	}
	else{
		QStringList densityList = countString.split(",");
		for(int i=0;i<densityList.size();i++)
			m_densityFields << densityList[i].toFloat();
	}
	
	if(m_densityFields.isEmpty())
		m_parent->printText("Config file - Incorrect Density Format\n");
	else
		runConfigWorld();
}

void simControl::runConfigWorld()
{
	QSettings configFile(QDir::currentPath() + "/mission/config",QSettings::IniFormat);		// pullup the config file
	
	float coverage;
	btVector3 minSize, maxSize;
	QVector2D yawrange;
	float step,cssize,efflimit,sprogress;
	int draw;
	bool visibility;
	int count = m_densityFields.takeFirst();
	long seed;
	GetSeed(&seed);
	
	QString trialname(configFile.value("Trial_Name").toString());			// Create a directory of the tiral including the density count
	QString trialDir(trialname + "_" + QString::number(count));
	m_trialLocation.mkdir(trialDir);
		
	minSize.setX(configFile.value("Obstacle_Min_X").toFloat());
	minSize.setY(configFile.value("Obstacle_Min_Y").toFloat());
	minSize.setZ(configFile.value("Obstacle_Min_Z").toFloat());
	maxSize.setX(configFile.value("Obstacle_Max_X").toFloat());
	maxSize.setY(configFile.value("Obstacle_Max_Y").toFloat());
	maxSize.setZ(configFile.value("Obstacle_Max_Z").toFloat());
	yawrange.setX(configFile.value("Obstacle_Yaw_Min").toFloat());
	yawrange.setY(configFile.value("Obstacle_Yaw_Max").toFloat());
	blocks->setParameters(count, minSize, maxSize, yawrange);				// set obstacle parameters
	blocks->generate(count);												// generate obstacles

	coverage = blocks->getMeanCoverage()/(ground->terrainSize().x()*ground->terrainSize().y());
	
	step = configFile.value("Sensor_Step").toFloat();
	cssize = configFile.value("Sensor_Cspace").toFloat();
	visibility = configFile.value("Path_Visibility").toBool();
	efflimit = configFile.value("Path_Eff_Limit").toFloat()/100;
	sprogress = configFile.value("Path_Spin_Progress").toFloat();
	draw = configFile.value("Path_Drawing").toBool();
	m_iterations = configFile.value("Iterations").toInt();						// set the number of iterations to perform
	m_pathSizeMin = configFile.value("Path_Size_Min").toFloat();
	m_pathSizeMax = configFile.value("Path_Size_Max").toFloat();
	
	if(!draw) m_view->stopDrawing(); 
	
	btVector3 roverstart(1,1,1);
	wTool->removeWaypoints();												// remove all old waypoints so user doesn't get confused
	addWaypoint(1,50,50);													// add a waypoint for viewing
	newRover(roverstart);													// create a new rover
	
	QStringList rangeList = configFile.value("Sensor_Ranges").toString().split(",");
	for(int i=0; i<rangeList.size(); i++)
	{
		float range = rangeList[i].toFloat();
		pTool->addPath(range,step,cssize,efflimit,sprogress,draw,0);		// create the paths with the parameters
		if(visibility && range != 0)
			pTool->addPath(range,step,cssize,efflimit,sprogress,draw,1);	// create a path with the same range but limit the visibility
	}
	
/////////////////////////////////////////
// Statistic CSV file header
/////////////
	m_statsFile = new QFile(m_trialLocation.absolutePath() + "/" + trialDir + "/stats.csv");
	if (m_statsFile->open(QIODevice::WriteOnly | QIODevice::Text)){				// if a statistics file can be opened
		QTextStream statsStream(m_statsFile);									// create a stream
		statsStream << "Obstacle #," << count << ",";
		statsStream << "Iterations," << m_iterations << ",";
		statsStream << "Seed," << seed << ",";
		statsStream << "Area Covered," << coverage << "\n";
		
		statsStream << "Step," << step << ",";
		statsStream << "CSpace," << cssize << ",";
		statsStream << "EffLimit," << efflimit << ",";
		statsStream << "SpinProgress," << sprogress << "\n";
		
		statsStream << "Range,Path Len,Strait Len,Comp Eff,Pure Eff,Time ms,State,Path File,Visibility\n";
		pTool->setStatisticsDevice(m_statsFile);
	}
///////////////////////////////

	pTool->setTrialname(m_trialLocation.absolutePath() + "/" + trialDir + "/" + trialname);	// set the base name of the XML save file
	
	connect(pTool, SIGNAL(pathsFinished()), this, SLOT(runIteration()));
	
	runIteration();
}

void simControl::runIteration()
{
	if(m_iterations <= 0){
		m_statsFile->close();												// close the stats file
		disconnect(pTool, SIGNAL(pathFinished()), this, SLOT(runIteration()));
		
		if(m_densityFields.isEmpty())										
		{																	// if there are no more densities exit the program
			m_parent->close();
			return;
		}
		else
		{
			runConfigWorld();												// else configure the new world
			return;
		}
	}
	
	if(blocks->areObstaclesActive()){										// wait until obstacles are at rest
		QTimer::singleShot(1000, this, SLOT(runIteration()));				// wait a second and call this method again
		return;
	}
	
	btVector3 tempStart;
	cSpace *space = new cSpace(tempStart,0,0.8,blocks);						// generate C-Space
	do{
		tempStart.setX(Randomn()*ground->terrainSize().x());				// calculate a random start position
		tempStart.setY(Randomn()*ground->terrainSize().y());		
		tempStart.setZ(ground->terrainHeightAt(tempStart));
	}while(space->isPointInsideCSpace(tempStart));							// make sure the robot starts outside of obstacles
	
	sr2->placeRobotAt(tempStart);
	arena->simulateStep();
	pTool->setStartPoint(tempStart + btVector3(0,0,0.01));
	
	btVector3 tempGoal;
	float d;							
	do{																		// randomize the start and goal points
		tempGoal.setX(Randomn()*ground->terrainSize().x());					// calculate a random goal position
		tempGoal.setY(Randomn()*ground->terrainSize().y());
		d = tempStart.distance(tempGoal);
	}while(d < m_pathSizeMin || d > m_pathSizeMax ||						// the straight line distance is within range
			space->isPointInsideCSpace(tempGoal));							// and the goal is outside the C-Space
	
	delete space;
	
	tempGoal.setZ(ground->terrainHeightAt(tempGoal));						// set the Z height of the goal point
	pTool->setGoalPoint(tempGoal + btVector3(0,0,0.01));
	wTool->moveWaypoint(0, tempGoal);
	
	m_parent->printText(QString("iterations %1").arg(m_iterations));
	m_iterations -= 1;
	
	pTool->generateAndSave();
}

/////////////////////////////////////////
// Rover generation functions, includes AutoNavigation and PathPlanning Tools
/////////////
void simControl::newRover(btVector3 start)
{
	if(!sr2) sr2 = new SR2rover(m_view);
	
	wTool->resetStates();
	
	start.setZ(ground->terrainHeightAt(btVector3(1,1,0)));
	sr2->placeRobotAt(start);
	
	autoNav = new autoCode(sr2, wTool->getList(), m_parent);
	pTool = new pathTool(sr2, blocks, m_parent,m_view);
	pTool->setStartPoint(start + btVector3(0,0,0.01));
	pTool->setGoalPoint(autoNav->getCurrentWaypoint().position + btVector3(0,0,0.01));
	connect(wTool, SIGNAL(currentWaypoint(int)), autoNav, SLOT(setCurrentWaypointIndex(int)));
	connect(blocks, SIGNAL(obstaclesRemoved()), pTool, SLOT(resetPaths()));
	connect(this, SIGNAL(pathView(int)), pTool, SLOT(stepOnPath(int)));
	m_view->setFocus(Qt::OtherFocusReason);
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
