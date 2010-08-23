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

#define OBSTSHAPEGROUP	"ObstacleShape" // settings group key value

obstacles::obstacles(terrain* gnd, simGLView* glView)
:
simGLObject(glView),
ground(gnd)
{
	arena = physicsWorld::instance(); // get the physics world object
	oTool = new obstacleTool(m_view->parentWidget());
	
	connect(oTool, SIGNAL(regenerateObstacles()), this, SLOT(generate()));
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
// Obstacle generation functions
/////////////
void obstacles::eliminate()
{
	arena->setDraw(false);
	arena->idle();
	
	for(int i=0; i<m_obstacleObjects.size(); i++)
		arena->getDynamicsWorld()->removeCollisionObject(m_obstacleObjects[i]);
		
	for(int i=0; i<m_obstacleShapes.size(); i++)
		delete m_obstacleShapes[i];
	
	m_obstacleObjects.clear();
	m_obstacleShapes.clear();
	
	arena->resetBroadphaseSolver();
	arena->toggleIdle();
	arena->setDraw(true);
	emit obstaclesRemoved();
}

void obstacles::generate()
{
	int i;
    btVector3   		tempPlace,tempSize;
    float   			alphaYaw,mass,volume;
	btTransform 		startTrans;
	btCollisionShape*	oShape = NULL;
	float dpht = oTool->dropHeight();
	
	this->eliminate();
	if(!ground) {
		m_view->printText("Open new Terrain to generate obstacles");
		return;
	}
	
    if(oTool->obstacleCount() == 0) return;
	
    for(i=0;i<oTool->obstacleCount();i++)
    {
		tempPlace.setX(Randomn()*50);//arena->worldSize().x());							// calculate a random position for the obstacle
		tempPlace.setY(Randomn()*50);//arena->worldSize().y());
		tempPlace.setZ(0);
		// keep all obstacles away from rover start position
        if(tempPlace.x() < 5 && tempPlace.y() < 5){ tempPlace.setX(5);} 
        //tempPlace.setZ(ground->maxHeight() + m_dropHeight);
		tempPlace.setZ(ground->terrainHeightAt(tempPlace) + dpht);
		
		alphaYaw = (oTool->minOYaw() + Randomn()*(oTool->maxOYaw() - oTool->minOYaw()));
		
		startTrans.setIdentity();
		startTrans.setOrigin(tempPlace);
		startTrans.setRotation(btQuaternion(0,0,DEGTORAD(alphaYaw)));					// set the starting position of the obstacle

        tempSize.setX(oTool->minOLength() + Randomn()*(oTool->maxOLength() - oTool->minOLength()));
        tempSize.setY(oTool->minOWidth() + Randomn()*(oTool->maxOWidth() - oTool->minOWidth()));
        tempSize.setZ(oTool->minOHeight() + Randomn()*(oTool->maxOHeight() - oTool->minOHeight()));
        
		oShape = createObstacleShape(oTool->obstacleType(),tempSize,volume);
        
        mass = oTool->density() * (volume);
		dpht += tempSize.z();										// add the height of the obstacle to the drop height
		
		createObstacleObject(mass,oShape,startTrans);				// create the new obstacle and add it to the world	
    }
	emit obstaclesRegenerated();
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

/////////////////////////////////////////
// XML file Creation functions
/////////////
void obstacles::saveLayout()
{
	QString filename = QFileDialog::getSaveFileName(m_view->parentWidget(),"Save Obstacle Layout", "/Users");	// open a Save File dialog and select location and filename
	if(filename == NULL) return; 															// if cancel is pressed dont do anything

	if(!filename.endsWith(".xml")) filename.append(".xml");
	
	QFile obstFile(filename); 
	if (!obstFile.open(QIODevice::WriteOnly))												// open file
		return;
	
	QFileInfo obstInfo(filename);
	arena->idle();																			// pause the simulation so nothing moves while saving
	
	QDomDocument xmlDoc( "roverSimDoc" );
	QDomElement root = xmlDoc.createElement( "obstacleLayout" );							// create a root element
	xmlDoc.appendChild(root);
	QString docInfo = "This XML document represents an obstacle layout for the RoverSim application";
	root.appendChild(xmlDoc.createComment(docInfo));
	
	root.setAttribute( "terrain", ground->terrainFilename());								// add the terrain filename and obstacle quantity as attributes
	root.setAttribute( "quantity", QString::number(m_obstacleObjects.size()));				// add the number of obstacles in the layout
	
	for(int i=0; i<m_obstacleObjects.size(); i++){
		btRigidBody* body = btRigidBody::upcast(m_obstacleObjects[i]);
		root.appendChild(rigidBodyToNode(xmlDoc,body));										// add each rigid body obstacle as children
	}
	
	QTextStream stream(&obstFile);
	stream << xmlDoc.toString();															// write the XML text to the file
	
	obstFile.close();																		// flush data and close file
	m_view->printText("Obstacle layout saved: " + obstInfo.baseName());
	arena->toggleIdle();																	// resume the simulation
}

QDomElement obstacles::vectorToNode(QDomDocument &doc, const btVector3 v)
{
	QDomElement vector = doc.createElement( "vector" );
	
	vector.setAttribute( "X", QString::number(v.x(),'g',12));
	vector.setAttribute( "Y", QString::number(v.y(),'g',12));
	vector.setAttribute( "Z", QString::number(v.z(),'g',12));
	return vector;
}
QDomElement obstacles::basisToNode(QDomDocument &doc, const btMatrix3x3 mx)
{
	QDomElement matrix = doc.createElement( "matrix" );
	
	for(int i=0;i<3;i++)
		matrix.appendChild(vectorToNode(doc, mx.getRow(i)));
	return matrix;
}
QDomElement obstacles::transformToNode(QDomDocument &doc, const btTransform t)
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
QDomElement obstacles::rigidBodyToNode(QDomDocument &doc, const btRigidBody* body)
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

void obstacles::loadLayout()
{
	QString filename = QFileDialog::getOpenFileName(m_view->parentWidget(),"Open Obstacle Layout", "/Users");
	if(filename == NULL) return;
	
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
		m_view->printText("File is not an obstacle layout: " + obstInfo.baseName());
	}
	
	this->eliminate();							// remove all obstacles
	
	//int count = root.attribute( "quantity", "0").toInt();
	QString	tName = root.attribute( "terrain", "NULL");
	
	if(tName != ground->terrainFilename()){			// if the terrain filename is different than what is loaded
		int ret = QMessageBox::question(m_view,		// ask the user if it should be loaded
					"Load New Terrain?",
		 			"The obstacle layout was built on terrain file: " + tName + "\n\nThis is different from what is currently loaded.\n\n" + 
					"Do you want to load the new terrain?",
					QMessageBox::Yes | QMessageBox::No,
					QMessageBox::Yes);
												
		if(ret == QMessageBox::Yes)
			ground->openTerrain(tName);				// open the new ground
	}
	
	QDomNode obst = root.firstChild();
	while(!obst.isNull()){
		QDomElement e = obst.toElement();
		
		if( !e.isNull() && e.tagName() == "rigidBody" ) elementToRigidBody(e);

		obst = obst.nextSibling();
	}
	
	m_view->printText("Obstacle layout loaded: " + obstInfo.baseName());
	m_view->printText(QString("Count = %1").arg(m_obstacleObjects.size()));
	
	//labelTerrainFilename->setText(SController->getGround()->terrainShortname());		// update the terrain name incase it changes
}

btVector3 obstacles::elementToVector(QDomElement element)
{
	btVector3 vect;
	
	vect.setX(element.attribute("X").toFloat());
	vect.setY(element.attribute("Y").toFloat());
	vect.setZ(element.attribute("Z").toFloat());
	return vect;
}
btMatrix3x3 obstacles::elementToMatrix(QDomElement element)
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
btTransform obstacles::elementToTransform(QDomElement element)
{
	btTransform trans;
	
	trans.setIdentity();
	QDomElement origin = element.firstChildElement("origin");
	trans.setOrigin(elementToVector(origin.firstChildElement("vector")));
	QDomElement rotate = element.firstChildElement("rotation");
	trans.setBasis(elementToMatrix(rotate.firstChildElement("matrix")));
	
	return trans;
}
void obstacles::elementToRigidBody(QDomElement element)
{
	float		tempVol;
	float 		tempMass;
	int			tempShape;
	btVector3   tempSize;
	btTransform	tempTrans;
	
	tempMass = element.attribute("mass", "1").toFloat();
	tempShape = element.attribute("shape","0").toInt();
	
	QDomElement size = element.firstChildElement("size");
	tempSize = elementToVector(size.firstChildElement("vector"));
	tempTrans = elementToTransform(element.firstChildElement("transform"));
	
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