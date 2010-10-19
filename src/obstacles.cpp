#include "obstacles.h"
#include "terrain.h"
#include "utility/glshapes.h"
#include "utility/rngs.h"
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btConeShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btDefaultMotionState.h>
#include "utility/SimDomElement.h"

#define OBSTSHAPEGROUP	"ObstacleShape" // settings group key value

userObstacleDialog::userObstacleDialog(QWidget *parent)
: QDialog(parent)
{
	setWindowTitle("User Obstacle Creation");
	
	// main input layout
	QVBoxLayout *inputLayout = new QVBoxLayout;
	sizeLabel = new QLabel("Obstacle Size");								// size label
	inputLayout->addWidget(sizeLabel);
	
	QGridLayout *sizeGridLayout = new QGridLayout;							// size grid
	QLabel* xlabel = new QLabel("X");
	sizeGridLayout->addWidget(xlabel,0,0);
	xSizeLineEdit = new QLineEdit(this);
	xSizeLineEdit->setText(QString::number(1));
	xSizeLineEdit->setAlignment(Qt::AlignHCenter);
	xSizeLineEdit->setMaximumWidth(100);
	sizeGridLayout->addWidget(xSizeLineEdit,0,1);
	
	QLabel* ylabel = new QLabel("Y");
	sizeGridLayout->addWidget(ylabel,1,0);
	ySizeLineEdit = new QLineEdit(this);
	ySizeLineEdit->setText(QString::number(1));
	ySizeLineEdit->setAlignment(Qt::AlignHCenter);
	ySizeLineEdit->setMaximumWidth(100);
	sizeGridLayout->addWidget(ySizeLineEdit,1,1);
	
	QLabel* zlabel = new QLabel("Z");
	sizeGridLayout->addWidget(zlabel,2,0);
	zSizeLineEdit = new QLineEdit(this);
	zSizeLineEdit->setText(QString::number(1));
	zSizeLineEdit->setAlignment(Qt::AlignHCenter);
	zSizeLineEdit->setMaximumWidth(100);
	sizeGridLayout->addWidget(zSizeLineEdit,2,1);
	inputLayout->addLayout(sizeGridLayout);
	
	QHBoxLayout *yawLayout = new QHBoxLayout;
	yawLabel = new QLabel("Yaw");
	yawLayout->addWidget(yawLabel);
	yawLineEdit = new QLineEdit(this);
	yawLineEdit->setText(QString::number(0));
	yawLineEdit->setAlignment(Qt::AlignHCenter);
	yawLineEdit->setMaximumWidth(100);
	yawLayout->addWidget(yawLineEdit);
	inputLayout->addLayout(yawLayout);
	
	QHBoxLayout *buttonLayout = new QHBoxLayout;
	doneButton = new QPushButton("Accept");
	doneButton->setMinimumWidth(80);
	doneButton->setStyleSheet("QPushButton:default{background: green; border: 2px solid darkgreen; border-radius:10; color: white;} QPushButton:pressed{background: red; border: 2px solid darkred; border-radius:10;}");
	doneButton->setDefault(true);
	buttonLayout->addWidget(doneButton);
	
	cancelButton = new QPushButton("Cancel");
	cancelButton->setMinimumWidth(80);
	cancelButton->setStyleSheet("QPushButton:enabled{background: yellow; border: 2px solid white; border-radius:10; color: black;} QPushButton:pressed{background: white; border-color: yellow;}");
	buttonLayout->addWidget(cancelButton);
	inputLayout->addLayout(buttonLayout);
	
	connect(doneButton,SIGNAL(clicked()),this,SLOT(acceptData()));
	connect(cancelButton,SIGNAL(clicked()),this,SLOT(reject()));
	
	this->setLayout(inputLayout);
}

void userObstacleDialog::acceptData()
{
	userObstSize.setX(fabs(xSizeLineEdit->text().toFloat()));
	userObstSize.setY(fabs(ySizeLineEdit->text().toFloat()));
	userObstSize.setZ(fabs(zSizeLineEdit->text().toFloat()));
	userObstYaw = yawLineEdit->text().toFloat();
	this->accept();
}

obstacles::obstacles(terrain* gnd, simGLView* glView)
:
simGLObject(glView),
ground(gnd),
m_saved(false)
{
	arena = physicsWorld::instance(); // get the physics world object
	oTool = new obstacleTool(m_view->parentWidget());
	
	connect(oTool, SIGNAL(regenerateObstacles(int)), this, SLOT(generate(int)));
	connect(m_view, SIGNAL(pickingVector(btVector3,btVector3)), this, SLOT(pickObstacle(btVector3,btVector3)));
	connect(m_view, SIGNAL(movingVector(btVector3,btVector3)), this, SLOT(moveObstacle(btVector3,btVector3)));
	connect(m_view, SIGNAL(dropPicked()), this, SLOT(dropObstacle()));
	connect(m_view, SIGNAL(spinPicked(float)), this, SLOT(spinObstacle(float)));
	connect(m_view, SIGNAL(loftPicked(float)), this, SLOT(loftObstacle(float)));
}

obstacles::~obstacles()
{
	this->eliminate();
	delete oTool;
}

/////////////////////////////////////////
// Obstacle parameter settings
/////////////
void obstacles::setParameters(int count, btVector3 min, btVector3 max, QVector2D yaw)
{
	oTool->setCount(count);
	oTool->setMinSize(min);
	oTool->setMaxSize(max);
	oTool->setYawRange(yaw.x(),yaw.y());
}

/////////////////////////////////////////
// Obstacle generation functions
/////////////
void obstacles::eliminate(int num)
{
	m_view->stopDrawing();
	arena->stopSimTimer();
	
	if(num <= 0 || num >= m_obstacleObjects.size())
		num = m_obstacleObjects.size();
		
	for(int i=num-1; i>=0; i--){
		arena->getDynamicsWorld()->removeCollisionObject(m_obstacleObjects[i]);
		m_obstacleObjects.removeAt(i);
		delete m_obstacleShapes[i];
		m_obstacleShapes.removeAt(i);
	}
	
	arena->resetWorld();
	m_view->startDrawing();
	emit obstaclesRemoved();
}

void obstacles::generate(int num)
{
	int i;
	int numObstDiff;
	
	if(!ground) {
		m_view->printText("Open new Terrain to generate obstacles");
		return;
	}
	
	m_meanArea = 0;
	m_dropHeight = oTool->dropHeight();
	
    if(num == 0) return;

	numObstDiff = num - m_obstacleObjects.size();

	if(numObstDiff == 0){												// create a completely new set of random obstacles
		this->eliminate();
		for(i=0;i<num;i++)
	   		singleRandomObstacle();
	}
	else if(numObstDiff > 0)											// add more random obstacles
	{	
		for(i=0;i<numObstDiff;i++)
	   		singleRandomObstacle();
	}
 	else																// remove some random obstacles
	{	
		eliminate(fabs(numObstDiff));
	}

	m_saved = false;
	emit obstaclesRegenerated();
}

void obstacles::randomize()
{
	generate(oTool->obstacleCount());
}

void obstacles::userObstacle()
{
	userObstacleDialog uoDialog(m_view->parentWidget());
	if(uoDialog.exec() == QDialog::Rejected)
		return;
	
	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(50,50,oTool->dropHeight() + (uoDialog.userObstSize.z()/2)));
	trans.setRotation(btQuaternion(0,0,DEGTORAD(uoDialog.userObstYaw)));
	
	singleObstacle(0.5*uoDialog.userObstSize,trans);
}

void obstacles::singleRandomObstacle()
{
	btVector3   		tempPlace,tempSize;
    float   			alphaYaw;
	btTransform 		startTrans;
	
	tempPlace.setX(Randomn()*ground->terrainSize().x());				// calculate a random position for the obstacle
	tempPlace.setY(Randomn()*ground->terrainSize().y());
	tempPlace.setZ(0);
		// keep all obstacles away from rover start position
	if(tempPlace.x() < 5 && tempPlace.y() < 5){ tempPlace.setX(5);} 

	tempPlace.setZ(ground->terrainHeightAt(tempPlace) + m_dropHeight);

	alphaYaw = (oTool->minOYaw() + Randomn()*(oTool->maxOYaw() - oTool->minOYaw()));

	startTrans.setIdentity();
	startTrans.setOrigin(tempPlace);
	startTrans.setRotation(btQuaternion(0,0,DEGTORAD(alphaYaw)));					// set the starting position of the obstacle

	tempSize.setX(oTool->minOLength() + Randomn()*(oTool->maxOLength() - oTool->minOLength()));
	tempSize.setY(oTool->minOWidth() + Randomn()*(oTool->maxOWidth() - oTool->minOWidth()));
	tempSize.setZ(oTool->minOHeight() + Randomn()*(oTool->maxOHeight() - oTool->minOHeight()));

	singleObstacle(tempSize,startTrans);
}

void obstacles::singleObstacle(btVector3 size, btTransform startTrans)
{
	btCollisionShape* oShape = NULL;
	float mass, volume;
	
	m_meanArea += 2*(size.x()*size.y() + size.x()*size.z() + size.y()*size.z())/3;       
	
	oShape = createObstacleShape(oTool->obstacleType(),size,volume);
	
	mass = oTool->density() * (volume);
	m_dropHeight += size.z();									// add the height of the obstacle to the drop height, keeps things from exploding
	
	createObstacleObject(mass,oShape,startTrans);				// create the new obstacle and add it to the world
}

btCollisionShape* obstacles::createObstacleShape(int shapeType, btVector3& lwh,float& vol)
{
	btCollisionShape* cShape;
	switch(shapeType)
    {														// don't forget lwh is just a half size dimension
        case 0:
			cShape = new btBoxShape(lwh);
            // volume = L x W x H
            vol = 8 * lwh.x() * lwh.y() * lwh.z();
            break;
        case 1:
            cShape = new btSphereShape(lwh.x());
            // volume  = 4/3 x PI x r^3
            vol = 4 * PI * ((lwh.x() * lwh.x() * lwh.x()) / 3);
            break;
        case 2:
            cShape = new btConeShapeZ(lwh.y(),lwh.x());
            // volume = 1/3 x PI x r^2 x H
            vol = (PI * lwh.x() * lwh.x() * lwh.y()) / 3;
            break;
        case 3:
            lwh.setZ(lwh.y());
            cShape = new btCylinderShapeX(lwh);
            // volume = PI x r^2 x H
            vol = PI * lwh.y() * lwh.y() *lwh.x();
            break;
        default:
            cShape = new btBoxShape(lwh);
            vol = 8 * lwh.x() * lwh.y() * lwh.z();
            break;
    }
	return cShape;
}

btRigidBody* obstacles::createObstacleObject(float mass, btCollisionShape* cShape, btTransform trans)
{
	btScalar massval(mass);													// set the mass of the box
	
	btVector3 bodyInertia(0,0,0);											// set inertia of body and calculate the CG
	if(massval != 0.f) cShape->calculateLocalInertia(massval,bodyInertia);
	
	// using motionstate provides interpolation and only synchronizes 'active' objects
	btDefaultMotionState* bodyMotionState = new btDefaultMotionState(trans);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,bodyMotionState,cShape,bodyInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setDamping(0.5,0.75);
	
	m_obstacleShapes << cShape;
	m_obstacleObjects << static_cast<btCollisionObject*>(body);
	
	arena->getDynamicsWorld()->addRigidBody(body);							// add the body to the world
	return body;
}

bool obstacles::areObstaclesActive()
{
	for(int i=0; i<m_obstacleObjects.size(); i++)
		if(m_obstacleObjects[i]->isActive()) return true;

	return false;
}

/////////////////////////////////////////
// XML file Creation functions
/////////////
void obstacles::saveLayout(QString filename)
{
	if(filename == NULL){
		filename = QFileDialog::getSaveFileName(m_view->parentWidget(),"Save Obstacle Layout", QDir::homePath());	// open a Save File dialog and select location and filename
		if(filename == NULL) return;															// if cancel is pressed dont do anything
	}

	if(!filename.endsWith(".xml")) filename.append(".xml");
	
	QFile obstFile(filename); 
	if (!obstFile.open(QIODevice::WriteOnly))												// open file
		return;
		
	m_layoutName = filename;
	
	QFileInfo obstInfo(m_layoutName);
	arena->stopSimTimer();																	// pause the simulation so nothing moves while saving
	
	QDomDocument xmlDoc( "roverSimDoc" );
	QDomElement root = xmlDoc.createElement( "obstacleLayout" );							// create a root element
	xmlDoc.appendChild(root);
	QString docInfo = "This XML document represents an obstacle layout for the RoverSim application";
	root.appendChild(xmlDoc.createComment(docInfo));
	
	root.setAttribute( "terrain", ground->terrainFilename());								// add the terrain filename and size as attributes
	root.setAttribute( "size", QString("%1,%2,%3").arg(ground->terrainSize().x()).arg(ground->terrainSize().y()).arg(ground->terrainSize().z()));
	root.setAttribute( "quantity", QString::number(m_obstacleObjects.size()));				// add the number of obstacles in the layout
	
	for(int i=0; i<m_obstacleObjects.size(); i++){
		btRigidBody* body = btRigidBody::upcast(m_obstacleObjects[i]);
		root.appendChild(SimDomElement::rigidBodyToNode(xmlDoc,body));						// add each rigid body obstacle as children
	}
	
	QTextStream stream(&obstFile);
	xmlDoc.save(stream,5);																	// write the XML text to the file
	m_saved = true;
	
	obstFile.close();																		// flush data and close file
	m_view->printText("Obstacle layout saved: " + obstInfo.baseName());
	arena->startSimTimer();																	// resume the simulation
}

void obstacles::loadLayout(QString filename)
{
	if(filename == NULL){
		filename = QFileDialog::getOpenFileName(m_view->parentWidget(),"Open Obstacle Layout", QDir::homePath());
		if(filename == NULL) return;					// cancel is pressed on the file dialog
	}
	
	m_layoutName = filename;
	
	QDomDocument xmlDoc( "roverSimDoc" );
	QFile obstFile(m_layoutName);
	QFileInfo obstInfo(m_layoutName);
	if(!obstFile.open(QIODevice::ReadOnly))
		return;
	
	if(!xmlDoc.setContent(&obstFile)){			// set the content of the file to an XML document
		obstFile.close();
		return;
	}
	obstFile.close();
	
	QDomElement root = xmlDoc.documentElement();
	if(root.tagName() != "obstacleLayout")
	{
		m_view->printText("File is not an obstacle layout: " + obstInfo.baseName());
		return;
	}
	
	this->eliminate();								// remove all obstacles
	
	QString	tName = root.attribute( "terrain", "NULL");
	
	if(tName != ground->terrainFilename()){			// if the terrain filename is different than what is loaded
		int ret = QMessageBox::question(m_view,		// ask the user if it should be loaded
					"Load New Terrain?",
		 			"The obstacle layout was built on terrain file: " + tName + "\n\nThis is different from what is currently loaded.\n\n" + 
					"Do you want to load the new terrain?",
					QMessageBox::Yes | QMessageBox::No,
					QMessageBox::Yes);
												
		if(ret == QMessageBox::Yes){
			m_saved = true;
			ground->openTerrain(tName);				// open the new ground
		}
		else m_saved = false;
	}
	
	btVector3 size;														// set the terrain to the proper size
	QStringList ss = root.attribute( "size","100,100,5").split(",");	// returns the size of the world attribute or sets the default to "100,100,5"
	size.setX(ss[0].toFloat());
	size.setY(ss[1].toFloat());
	size.setZ(ss[2].toFloat());
	ground->setTerrainSize(size);
	
	QDomNode obst = root.firstChild();
	while(!obst.isNull()){
		QDomElement e = obst.toElement();
		
		if( !e.isNull() && e.tagName() == "rigidBody" ) elementToObstacle(e);

		obst = obst.nextSibling();
	}
	
	m_view->printText("Obstacle layout loaded: " + obstInfo.baseName());
	m_view->printText(QString("Count = %1").arg(m_obstacleObjects.size()));
}

void obstacles::elementToObstacle(QDomElement element)
{
	float		tempVol;
	float 		tempMass;
	int			tempShape;
	btVector3   tempSize;
	btTransform	tempTrans;
	
	tempMass = element.attribute("mass", "1").toFloat();
	tempShape = element.attribute("shape","0").toInt();
	
	QDomElement size = element.firstChildElement("size");
	tempSize = SimDomElement::elementToVector(size.firstChildElement("vector"));
	tempTrans = SimDomElement::elementToTransform(element.firstChildElement("transform"));
	
	btCollisionShape* oShape = createObstacleShape(tempShape,tempSize,tempVol);		// create the collision shape
	createObstacleObject(tempMass,oShape,tempTrans);								// create the dynamic rigid body and add it to the world
}

 
/////////////////////////////////////////
// Obstacle pick and placement functions
/////////////
void obstacles::pickObstacle(btVector3 camPos,btVector3 mousePos)
{
	// this will be a connection from glview
	// which sets the camera position and mouse world position for ray testing
	
	// test the ray and the object
	btCollisionWorld::ClosestRayResultCallback rayCBto(camPos,mousePos);
	arena->getDynamicsWorld()->rayTest(camPos,mousePos,rayCBto);
	if(rayCBto.hasHit() && m_obstacleObjects.contains(rayCBto.m_collisionObject)) // only pick up obstacle objects
	{
			// turn on mouse tracking for glView
			m_view->setMouseTracking(true);
			// move the obstacle up above terrain
			m_pickingObject.rigidbody = btRigidBody::upcast(rayCBto.m_collisionObject);
			m_pickingObject.rigidbody->forceActivationState(WANTS_DEACTIVATION);
			m_pickingObject.rotAxis.setValue(0,0,1);
			m_pickingObject.dropHeight = 5.0;
			m_pickingObject.hitPoint = rayCBto.m_hitPointWorld;
			m_view->setPickObject(&m_pickingObject);
			
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
void obstacles::moveObstacle(btVector3 camPos,btVector3 mousePos)
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
void obstacles::spinObstacle(float spin)
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
void obstacles::loftObstacle(float loft)
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
void obstacles::dropObstacle()
{
	// when the mouse is clicked again drop the obstacle
	// turn off mouse tracking for glView
	m_view->setMouseTracking(false);
	m_pickingObject.rigidbody->activate();
	m_view->setPickObject(NULL);
	m_pickingObject.rigidbody = NULL;
	m_saved = false;
}
void obstacles::orientObstacle()
{
	// change the atitude or UP vector of the obstacle
	if(m_pickingObject.rotAxis.x()) m_pickingObject.rotAxis.setValue(0,1,0);
	else if(m_pickingObject.rotAxis.y()) m_pickingObject.rotAxis.setValue(0,0,1);
	else m_pickingObject.rotAxis.setValue(1,0,0);
}


/////////////////////////////////////////
// OpenGL drawing
/////////////
void obstacles::renderGLObject()
{
    btScalar	glm[16];
	
	//glColor3f(1.0f,0.0f,0.0f);
    //glColor3f(0.02f,0.52f,0.51f);	// tron blue
	//glColor3f(0.1f,0.0f,0.5f); // dark blue
	//glColor3f(0.7,0.0,0.7);	// dark purple
    //glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    //glMaterialfv(GL_FRONT, GL_EMISSION, obstacleEmission);
    for(int i=0; i < m_obstacleObjects.size(); i++){
        btRigidBody* body = btRigidBody::upcast(m_obstacleObjects[i]);
		
		if(body->getMotionState())
		{
			btDefaultMotionState* objMotionState = (btDefaultMotionState*)body->getMotionState();
			objMotionState->m_graphicsWorldTrans.getOpenGLMatrix(glm);
		}
		else
			body->getWorldTransform().getOpenGLMatrix(glm);

		if(body->isActive()) glColor3f(0.02f,0.52f,0.51f);	// tron blue
		else glColor3f(1.0f,0.0f,0.0f);	// red
		
        btCollisionShape* colisShape = body->getCollisionShape();
		
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
			default:
			break;
				/*
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
								break;*/
				
        }	
        glPopMatrix();
    }	
}