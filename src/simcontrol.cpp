#include "simcontrol.h"
#include "skydome.h"
#include "terrain.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "cSpace.h"
#include "pathPlan.h"
#include "tools/pathtool.h"
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
	:	ground(NULL),sky(NULL),sr2(NULL),autoNav(NULL),m_pTool(NULL),glView(vw),
m_obstType(0),
m_obstCount(50),
m_dropHeight(5),
m_minObstYaw(0),
m_maxObstYaw(45),
m_obstDensity(5)
{
	path = 0;
	m_CS = 0;
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
	if(path) delete path;
	if(m_CS) delete m_CS;
	simTimer->stop();
	delete simTimer;
	this->removeRover();
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
	if(arena->isIdle())
		glView->printText("Simulation Paused");
	else
		glView->printText("Resumed Simulation");
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
	QFileInfo terrainInfo(filename);
	glView->printText("Terrain Loaded: " + terrainInfo.baseName());
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

void simControl::removeObstacles()
{
	arena->deleteObstacleGroup();
	emit obstaclesRemoved();
}

// XML file creation functions
void simControl::saveObstaclesXML(QString filename)
{
	QFile obstFile(filename); 
	if (!obstFile.open(QIODevice::WriteOnly))												// open file
		return;
	
	QFileInfo obstInfo(filename);
	arena->idle();																			// pause the simulation so nothing moves while saving
	btAlignedObjectArray<btCollisionObject*>* obstArray = arena->getObstacleObjectArray();	// get the obstacle array
	
	QDomDocument xmlDoc( "roverSimDoc" );
	QDomElement root = xmlDoc.createElement( "obstacleLayout" );							// create a root element
	xmlDoc.appendChild(root);
	QString docInfo = "This XML document represents an obstacle layout for the RoverSim application";
	root.appendChild(xmlDoc.createComment(docInfo));
	
	root.setAttribute( "terrain", ground->terrainFilename());								// add the terrain filename and obstacle quantity as attributes
	root.setAttribute( "quantity", QString::number(obstArray->size()));
	
	for(int i=0; i<obstArray->size(); i++)
	{
		btRigidBody* body = btRigidBody::upcast(obstArray->at(i));
		root.appendChild(rigidBodyToNode(xmlDoc,body));										// add each rigid body obstacle as children
	}
	
	QTextStream stream(&obstFile);
	stream << xmlDoc.toString();															// write the XML text to the file
	
	obstFile.close();																		// flush data and close file
	glView->printText("Obstacle layout saved: " + obstInfo.baseName());
	arena->toggleIdle();																	// resume the simulation
}

QDomElement simControl::vectorToNode(QDomDocument &doc, const btVector3 v)
{
	QDomElement vector = doc.createElement( "vector" );
	
	vector.setAttribute( "X", QString::number(v.x(),'g',12));
	vector.setAttribute( "Y", QString::number(v.y(),'g',12));
	vector.setAttribute( "Z", QString::number(v.z(),'g',12));
	return vector;
}
QDomElement simControl::basisToNode(QDomDocument &doc, const btMatrix3x3 mx)
{
	QDomElement matrix = doc.createElement( "matrix" );
	
	for(int i=0;i<3;i++)
		matrix.appendChild(vectorToNode(doc, mx.getRow(i)));
	return matrix;
}
QDomElement simControl::transformToNode(QDomDocument &doc, const btTransform t)
{
	QDomElement trans = doc.createElement( "transform" );
	
	QDomElement origin = doc.createElement( "origin" );
	origin.appendChild(vectorToNode(doc, t.getOrigin()));
	trans.appendChild(origin);
	
	QDomElement rotate = doc.createElement( "rotation" );
	rotate.appendChild(basisToNode(doc, t.getBasis()));
	trans.appendChild(rotate);
	return trans;
}
QDomElement simControl::rigidBodyToNode(QDomDocument &doc, const btRigidBody* body)
{
	QDomElement rigidBody = doc.createElement( "rigidBody" );
	
	rigidBody.setAttribute( "shape", QString::number(body->getCollisionShape()->getShapeType()));
	rigidBody.setAttribute( "mass", QString::number(1.0/body->getInvMass(),'g',10));
	
	QDomElement size = doc.createElement( "size" );
	const btBoxShape* boxShape = static_cast<const btBoxShape*>(body->getCollisionShape());
	size.appendChild(vectorToNode(doc, boxShape->getHalfExtentsWithMargin()));
	
	rigidBody.appendChild(size);
	rigidBody.appendChild(transformToNode(doc, body->getWorldTransform()));
	return rigidBody;
}

void simControl::loadObstaclesXML(QString filename)
{
	QDomDocument xmlDoc( "roverSimDoc" );
	QFile obstFile(filename);
	QFileInfo obstInfo(filename);
	if(!obstFile.open(QIODevice::ReadOnly))
		return;
	
	if(!xmlDoc.setContent(&obstFile)){			// set the content of the file to an XML document
		obstFile.close();
		return;
	}
	obstFile.close();
	
	QDomElement root = xmlDoc.documentElement();
	if(root.tagName() != "obstacleLayout"){
		glView->printText("File is not an obstacle layout: " + obstInfo.baseName());
	}
	
	this->removeObstacles();
	
	//int count = root.attribute( "quantity", "0").toInt();
	QString	tName = root.attribute( "terrain", "NULL");
	
	if(tName != ground->terrainFilename()){			// if the terrain filename is different than what is loaded
		int ret = QMessageBox::question(glView,		// ask the user if it should be loaded
					"Load New Terrain?",
		 			"The obstacle layout was built on terrain file: " + tName + "\n\nThis is different from what is currently loaded.\n\n" + 
					"Do you want to load the new terrain?",
					QMessageBox::Yes | QMessageBox::No,
					QMessageBox::Yes);
												
		if(ret == QMessageBox::Yes)
			this->openNewGround(tName);				// open the new ground
	}
	
	QDomNode obst = root.firstChild();
	while(!obst.isNull()){
		QDomElement e = obst.toElement();
		
		if( !e.isNull() && e.tagName() == "rigidBody" ) elementToRigidBody(e);

		obst = obst.nextSibling();
	}
	m_obstCount = arena->getObstacleObjectArray()->size();
	glView->printText("Obstacle layout loaded: " + obstInfo.baseName());
	glView->printText(QString("Count = %1").arg(m_obstCount));
}

btVector3 simControl::elementToVector(QDomElement element)
{
	btVector3 vect;
	
	vect.setX(element.attribute("X").toFloat());
	vect.setY(element.attribute("Y").toFloat());
	vect.setZ(element.attribute("Z").toFloat());
	return vect;
}
btMatrix3x3 simControl::elementToMatrix(QDomElement element)
{
	btVector3 vx,vy,vz;
	QDomElement vect = element.firstChildElement("vector");
	vx = elementToVector(vect);
	vect = vect.nextSiblingElement("vector");
	vy = elementToVector(vect);
	vect = vect.nextSiblingElement("vector");
	vz = elementToVector(vect);
	
	btMatrix3x3 mx(vx.x(),vx.y(),vx.z(),vy.x(),vy.y(),vy.z(),vz.x(),vz.y(),vz.z());
	return mx;
}
btTransform simControl::elementToTransform(QDomElement element)
{
	btTransform trans;
	
	trans.setIdentity();
	QDomElement origin = element.firstChildElement("origin");
	trans.setOrigin(elementToVector(origin.firstChildElement("vector")));
	QDomElement rotate = element.firstChildElement("rotation");
	trans.setBasis(elementToMatrix(rotate.firstChildElement("matrix")));
	
	return trans;
}
void simControl::elementToRigidBody(QDomElement element)
{
	float 		tempMass;
	int			tempShape;
	btVector3   tempSize;
	btTransform	tempTrans;
	
	tempMass = element.attribute("mass", "1").toFloat();
	tempShape = element.attribute("shape","0").toInt();
	
	QDomElement size = element.firstChildElement("size");
	tempSize = elementToVector(size.firstChildElement("vector"));
	tempTrans = elementToTransform(element.firstChildElement("transform"));
	
	arena->createObstacleShape(BOX_SHAPE_PROXYTYPE,tempSize);	// create the collision shape
	arena->placeObstacleShapeAt(tempTrans,tempMass);			// create the dynamic rigid body and add it to the world
}

// old file creation functions
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
void simControl::testCspace()
{
	if(sr2){
		btVector3 start = sr2->position;
		start.setZ(ground->terrainHeightAt(sr2->position));
		start += btVector3(0,0,0.01);
		if(m_CS) delete m_CS;
		m_CS = new cSpace(start,0,glView);
		m_CS->drawCspace(true);
	}
}

void simControl::testPath()
{
	if(path) delete path;
	path = 0;
	if(sr2){
		path = new pathPlan(glView);
		path->goForGoal(sr2->position - btVector3(0,0,0.34),autoNav->getCurrentWaypoint().position + btVector3(0,0,0.01));
	}
}

/////////////////////////////////////////
// Rover generation functions
/////////////
void simControl::newRover(QWidget* parent, btVector3 start)
{
	if(!ground) return;
	if(!sr2) sr2 = new SR2rover(glView);
	
	this->setWaypointGroundHeight();
	this->resetWaypointStates();
	
	start.setZ(ground->terrainHeightAt(btVector3(1,1,0)));
	sr2->placeRobotAt(start);
	autoNav = new autoCode(sr2, &waypointList, parent);
	m_pTool = new pathTool(sr2, glView, parent);
	m_pTool->setGoalPoint(autoNav->getCurrentWaypoint().position + btVector3(0,0,0.01));
	connect(this, SIGNAL(obstaclesRemoved()), m_pTool, SLOT(resetPaths()));
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
void simControl::showPathView(int dir)
{
	emit pathView(dir);
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