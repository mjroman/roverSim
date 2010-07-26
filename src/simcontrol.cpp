#include "simcontrol.h"
#include "skydome.h"
#include "terrain.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "cSpace.h"
#include "pathPlan.h"
#include "simGLView.h"
#include "utility/rngs.h"


// Use if Bullet Frameworks are added
#include <BulletCollision/btBulletCollisionCommon.h>
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btAlignedObjectArray.h>

#define LAYOUTHEADER	"RoverSim Obstacle Layout File"

simControl::simControl(simGLView* vw)
	:	ground(NULL),sky(NULL),sr2(NULL),autoNav(NULL),configSpace(NULL),path(NULL),glView(vw),
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
	
	//ground = new terrain(QString(":/textures/src/textures/defaultTerrain.png"), glView);
	ground = new terrain(NULL, glView);
	this->generateObstacles();
	
	sky = new skydome(glView);
	
	// add a few test waypoints
	addWaypointAt(657,50.0,50.0);
	addWaypointAt(658,1.0,14.0);
	
	glView->setWaypointList(&waypointList);
	
	simTimer = new QTimer(this);
    connect(simTimer, SIGNAL(timeout()), this, SLOT(stepSim()));
    this->startSimTimer(10);

	connect(glView, SIGNAL(pickingVector(btVector3,btVector3)), this, SLOT(pickObstacle(btVector3,btVector3)));
	connect(glView, SIGNAL(movingVector(btVector3,btVector3)), this, SLOT(moveObstacle(btVector3,btVector3)));
	connect(glView, SIGNAL(dropPicked()), this, SLOT(dropObstacle()));
	connect(glView, SIGNAL(spinPicked(float)), this, SLOT(spinObstacle(float)));
	connect(glView, SIGNAL(loftPicked(float)), this, SLOT(loftObstacle(float)));
}

simControl::~simControl()
{
	simTimer->stop();
	delete simTimer;
	if(path) delete path;
	if(configSpace) delete configSpace;
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
	if(path) delete path;
	path = 0;
	if(configSpace) delete configSpace;
	configSpace = 0;
	
	ground->openTerrain(filename);
	glView->printText("Terrain Loaded: " + filename);
	// restore waypoint Z height
	this->setWaypointGroundHeight();
}
void simControl::flattenGround()
{
	this->removeRover();
	ground->terrainFlatten();
	
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
	float dpht = m_dropHeight;
	
	this->removeObstacles();
	
    if(m_obstCount == 0) return;
	//qDebug("generating new obstacles %d",m_obstCount);
    for(i=0;i<m_obstCount;i++)
    {
		tempPlace.setX(Randomn()*50);//arena->worldSize().x());
		tempPlace.setY(Randomn()*50);//arena->worldSize().y());
		tempPlace.setZ(0);
		// keep all obstacles away from rover start position
        if(tempPlace.x() < 5 && tempPlace.y() < 5){ tempPlace.setX(5);} 
        //tempPlace.setZ(ground->maxHeight() + m_dropHeight);
		tempPlace.setZ(ground->terrainHeightAt(tempPlace) + dpht);
		
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
		dpht += tempSize.z();
        arena->placeObstacleShapeAt(tempPlace,alphaYaw,mass);
    }
}

void simControl::saveObstacles(QString filename)
{
	QFile obstFile(filename); 
	if (!obstFile.open(QIODevice::WriteOnly | QIODevice::Text))								// open file
		return;

	QFileInfo obstInfo(filename);
	QTextStream os(&obstFile);
	arena->idle();																			// pause the simulation so nothing moves while saving
	
	btAlignedObjectArray<btCollisionObject*>* obstArray = arena->getObstacleObjectArray();	// get the obstacle array
	
	os << LAYOUTHEADER << "\n";
	os << "terrain: " << ground->terrainFilename() << "\n";
	os << "quantity: " << obstArray->size() << "\n";										// write the number of obstacles
	
	for(int i=0; i<obstArray->size(); i++)
	{
		btRigidBody* body = btRigidBody::upcast(obstArray->at(i));
		os << "obstacle:\n";
		os << "shape " << body->getCollisionShape()->getShapeType() << "\n";		// write obstacle shape type
		os << "mass " << qSetRealNumberPrecision(10) << (1/body->getInvMass()) << "\n"; 							// write obstacle mass
		const btBoxShape* boxShape = static_cast<const btBoxShape*>(body->getCollisionShape());
		btVector3 size = boxShape->getHalfExtentsWithMargin();
		os << "size " << size.x() << " " << size.y() << " " << size.z() << "\n";	// write obstacle size
		btVector3 pos = body->getWorldTransform().getOrigin();
		os << "position " << pos.x() << " " << pos.y() << " " << pos.z() << "\n";	// write obstacle position
		btMatrix3x3 mx = body->getWorldTransform().getBasis();
		os << "rotation\n";
		for(int j=0;j<3;j++)														// write obstacle transformation basis vectors
		{
			btVector3 row = mx.getRow(j);
			os << row.x() << " " << row.y() << " " << row.z() << "\n";
		}
		os << "\n";
	}
	obstFile.close();	// flush data and close file
	glView->printText("Obstacle layout: " + obstInfo.baseName() + " saved");
	arena->toggleIdle();	// resume the simulation
}

void simControl::loadObstacles(QString filename)
{
	QFile obstFile(filename);
	QFileInfo obstInfo(filename);
	if(!obstFile.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
		
	QTextStream is(&obstFile);
	QString header = is.readLine(30);	// read in the first line to varify that it is the type of file interested in
	if(header.contains(LAYOUTHEADER))	// make sure the header matches the file
	{	
		int count;
		QString		tName;
		float 		tempMass;
		btVector3   tempPlace,tempSize;
		btMatrix3x3	tempBasis;
		btTransform	tempTrans;
	 	
		this->removeObstacles();
		
		if(!this->fileParser(&is,"terrain:",&tName))		// read in the terrain filename
		{
			glView->printText("Obstacle layout: " + obstInfo.baseName() + " - corrupt");
			obstFile.close();
			return;
		}
		
		if(tName != ground->terrainFilename()){			// if the terrain filename is different than what is loaded
			int ret = QMessageBox::question(glView,		// ask the user if it should be loaded
						"Load New Terrain?",
			 			"The obstacle layout was built on terrain file:\n" + tName + "\nThis is different from what is currently loaded.\n\n" + 
						"Do you want to load the new terrain?",
						QMessageBox::Yes | QMessageBox::No,
						QMessageBox::Yes);
													
			if(ret == QMessageBox::Yes)
				this->openNewGround(tName);				// open the new ground
		}
		
		if(!this->fileParser(&is,"quantity:",&count))	// read in the quantity of obstacles to be created
		{	
			glView->printText("Obstacle layout: " + obstInfo.baseName() + " - corrupt");
			obstFile.close();
			return;
		}
		if(count == 0){
			obstFile.close();
			return;
		}
		
		int i=0;
		while(this->fileParser(&is,"mass",&tempMass) &&
			  this->fileParser(&is,"size",&tempSize) &&
			  this->fileParser(&is,"position",&tempPlace) &&
			  this->fileParser(&is,"rotation",&tempBasis))
		{		
			tempTrans.setIdentity();
			tempTrans.setOrigin(tempPlace);
			tempTrans.setBasis(tempBasis);
			arena->createObstacleShape(BOX_SHAPE_PROXYTYPE,tempSize);	// create the collision shape
			arena->placeObstacleShapeAt(tempTrans,tempMass);			// create the dynamic rigid body and add it to the world
			i++;
		}
		m_obstCount = i;
		glView->printText("Obstacle layout: " + obstInfo.baseName() + " - loaded");
	}
	
	obstFile.close();
}

bool simControl::fileParser(QTextStream* stream, QString word, void* value)
{
	QString header;
	do{
		*stream >> header;
		if(stream->status() == QTextStream::ReadPastEnd) return false;
	}while(header != word);
	
	if(word == "terrain:"){
		QString name = stream->readLine();
		*(QString*)value = name.simplified();
	}
	else if(word == "quantity:" || word == "shape")
		*stream >> *(int*)value;
	else if(word == "mass")
		*stream >> *(float*)value;
	else if(word == "size" || word == "position"){
		btVector3 vect;
		for(int i=0;i<3;i++) *stream >> vect.m_floats[i];
		*(btVector3*)value = vect;
	}
	else if(word == "rotation"){
		float v[9];
		for(int i=0;i<9;i++) *stream >> v[i];
		btMatrix3x3 mx(v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]);
		*(btMatrix3x3*)value = mx;
	}
	else return false;
	
	if(stream->status() == QTextStream::ReadPastEnd) return false;
	return true;
}

void simControl::removeObstacles()
{
	if(path) delete path;
	path = 0;
	if(configSpace) delete configSpace;
	configSpace = 0;
	
	arena->deleteObstacleGroup();
}

/////////////////////////////////////////
// Obstacle pick and placement functions
/////////////
void simControl::pickObstacle(btVector3 camPos,btVector3 mousePos)
{
	// this will be a connection from glview
	// which sets the camera position and mouse world position for ray testing
	
	// test the ray and the object
	btCollisionWorld::ClosestRayResultCallback rayCBto(camPos,mousePos);
	arena->getDynamicsWorld()->rayTest(camPos,mousePos,rayCBto);
	if(rayCBto.hasHit() && arena->isObstacle(rayCBto.m_collisionObject)) // only pick up obstacle objects
	{
			// turn on mouse tracking for glView
			glView->setMouseTracking(true);
			// move the obstacle up above terrain
			m_pickingObject.rigidbody = btRigidBody::upcast(rayCBto.m_collisionObject);
			m_pickingObject.rigidbody->forceActivationState(WANTS_DEACTIVATION);
			m_pickingObject.rotAxis.setValue(0,0,1);
			m_pickingObject.dropHeight = 5.0;
			m_pickingObject.hitPoint = rayCBto.m_hitPointWorld;
			glView->setPickObject(&m_pickingObject);
			
			btTransform trans = m_pickingObject.rigidbody->getWorldTransform();
			trans.setIdentity();
			trans.setOrigin(m_pickingObject.hitPoint + btVector3(0,0,m_pickingObject.dropHeight));
			
			btDefaultMotionState* bodyMotionState = (btDefaultMotionState*)m_pickingObject.rigidbody->getMotionState();
			bodyMotionState->m_startWorldTrans = trans;
			bodyMotionState->m_graphicsWorldTrans = trans;
			m_pickingObject.rigidbody->setCenterOfMassTransform(trans);
			m_pickingObject.rigidbody->setInterpolationWorldTransform(trans);
			//rigidbody->activate();
			//rigidbody->applyCentralImpulse(btVector3(0,0,50));
	}
}

void simControl::moveObstacle(btVector3 camPos,btVector3 mousePos)
{
	// when the mouse is moved, move the obstacle
	notMeRayResultCallback rayCBto(camPos,mousePos,m_pickingObject.rigidbody);
	arena->getDynamicsWorld()->rayTest(camPos,mousePos,rayCBto);
	if(rayCBto.hasHit()){
		// use the world hitpoint to position the object
		m_pickingObject.rigidbody->setActivationState(WANTS_DEACTIVATION);
		btTransform trans = m_pickingObject.rigidbody->getWorldTransform();
		m_pickingObject.hitPoint = rayCBto.m_hitPointWorld;
		trans.setOrigin(m_pickingObject.hitPoint + btVector3(0,0,m_pickingObject.dropHeight));
		
		btDefaultMotionState* pickingMS = (btDefaultMotionState*)m_pickingObject.rigidbody->getMotionState();
		pickingMS->m_startWorldTrans = trans;
		pickingMS->m_graphicsWorldTrans = trans;
		m_pickingObject.rigidbody->setCenterOfMassTransform(trans);
		m_pickingObject.rigidbody->setInterpolationWorldTransform(trans);
	}
}

void simControl::spinObstacle(float spin)
{
	// when scrolling change the yaw of the obstacle
	btTransform trans = m_pickingObject.rigidbody->getWorldTransform();
	btQuaternion rot(m_pickingObject.rotAxis,DEGTORAD(spin*0.125));
	trans.setRotation(trans.getRotation() * rot);
	
	btDefaultMotionState* pickingMS = (btDefaultMotionState*)m_pickingObject.rigidbody->getMotionState();
	pickingMS->m_startWorldTrans = trans;
	pickingMS->m_graphicsWorldTrans = trans;
	m_pickingObject.rigidbody->setCenterOfMassTransform(trans);
	m_pickingObject.rigidbody->setInterpolationWorldTransform(trans);
}

void simControl::loftObstacle(float loft)
{
	// when scrolling change the yaw of the obstacle
	btTransform trans = m_pickingObject.rigidbody->getWorldTransform();
	m_pickingObject.dropHeight += loft*0.01;
	if(m_pickingObject.dropHeight < 0) m_pickingObject.dropHeight = 0;
	trans.setOrigin(m_pickingObject.hitPoint + btVector3(0,0,m_pickingObject.dropHeight));
	
	btDefaultMotionState* pickingMS = (btDefaultMotionState*)m_pickingObject.rigidbody->getMotionState();
	pickingMS->m_startWorldTrans = trans;
	pickingMS->m_graphicsWorldTrans = trans;
	m_pickingObject.rigidbody->setCenterOfMassTransform(trans);
	m_pickingObject.rigidbody->setInterpolationWorldTransform(trans);
}

void simControl::dropObstacle()
{
	// when the mouse is clicked again drop the obstacle
	// turn off mouse tracking for glView
	glView->setMouseTracking(false);
	m_pickingObject.rigidbody->activate();
	glView->setPickObject(NULL);
	m_pickingObject.rigidbody = NULL;
}
void simControl::orientObstacle()
{
	// change the atitude or UP vector of the obstacle
	if(m_pickingObject.rotAxis.x()) m_pickingObject.rotAxis.setValue(0,1,0);
	else if(m_pickingObject.rotAxis.y()) m_pickingObject.rotAxis.setValue(0,0,1);
	else m_pickingObject.rotAxis.setValue(1,0,0);
}

/////////////////////////////////////////
// Path plan generation functions
/////////////
void simControl::generatePath()
{
	btVector3 start,end;
	if(configSpace) delete configSpace;
	if(path) delete path;
	
	if(sr2 && waypointList.size() > 0){
		start = sr2->position;
		start.setZ(ground->terrainHeightAt(sr2->position));
		start += btVector3(0,0,0.01);
		end = autoNav->getCurrentWaypoint().position;
		end += btVector3(0,0,0.01);
		configSpace = new cSpace(start,30,glView);
		path = new pathPlan(start, end, configSpace, glView);
		//path->setColor(btVector3(1,0,0));
	}
	else{
		start.setValue(1,1,0);
		end.setValue(40,40,0);
		start.setZ(ground->terrainHeightAt(start));
		end.setZ(ground->terrainHeightAt(end));
		configSpace = new cSpace(start,0,glView);
		path = new pathPlan(start, end, configSpace, glView);
	}
}

void simControl::testPath()
{
	if(path) path->togglePathPoint();
}

/////////////////////////////////////////
// Rover generation functions
/////////////
void simControl::newRover(QWidget* parent)
{
	if(autoNav) delete autoNav;
	if(!ground) return;
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
		// set the z position of the waypoint for visability
		waypointList[i].position.setZ(ground->terrainHeightAt(waypointList[i].position));
		i++;
	}
}
// if a waypoint is moved recalculate its Z position
void simControl::editWaypoint(int index)
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