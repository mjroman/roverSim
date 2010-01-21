#include "maincontroller.h"
#include "camera.h"
#include "utility/rngs.h"
#include "rover.h"
#include "terrain.h"
#include "skydome.h"

// Use if Bullet Frameworks are added
#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

MainController::MainController(QWidget *parent)
:
QMainWindow(parent),
m_oTool(this),
m_simTool(this),
m_tTool(this)
{
    qDebug("pre-UIsetup");
    setupUi(this);
    move(240,22);
    resize(1200,700);
	
    m_oTool.close();
    m_simTool.close();
    m_tTool.close();

    // menu bar connections
    connect(actionRandomize_Obstacles, SIGNAL(triggered()), this, SLOT(generateObstacles()));
    connect(actionRemove_Obstacles, SIGNAL(triggered()), this, SLOT(removeAllObstacles()));
    connect(actionObstacle_Parameters, SIGNAL(triggered()), &m_oTool, SLOT(raise()));
    connect(actionOpen_Terrain, SIGNAL(triggered()), this, SLOT(openTerrainFile()));
    connect(actionSave_Terrain, SIGNAL(triggered()), this, SLOT(saveTerrainFile()));
    connect(actionFlatten_Terrain, SIGNAL(triggered()), this, SLOT(flattenTerrain()));
    connect(actionSim_Timing, SIGNAL(triggered()), &m_simTool, SLOT(show()));
    connect(actionTerrain_Parameters, SIGNAL(triggered()), &m_tTool, SLOT(raise()));

    // tool bar Simulation timing
    connect(&m_simTool, SIGNAL(paramUpdate()), this, SLOT(stepTimevals()));
    // tool bar obstacle generation button signal
    connect(&m_oTool, SIGNAL(regenerateObstacles()), this, SLOT(generateObstacles()));
    // tool bar terrain gravity update
    connect(&m_tTool, SIGNAL(gravityUpdate()), this, SLOT(simGravity()));
    // tool bar terrain scale update
    connect(&m_tTool, SIGNAL(scaleUpdate()), this, SLOT(rescaleTerrain()));

    connect(glView, SIGNAL(refreshView()), this, SLOT(updateGUI()));

    sr2 = 0;
    ground = 0;
    sky = 0;

    labelTerrainFilename->setText(QString(":/textures/src/textures/defaultTerrain.png"));
    arena = physicsWorld::instance(); // get the physics world object

    sky = new skydome(glView);
    ground = new terrain(labelTerrainFilename->text(), glView);
    m_tTool.setScale(ground->terrainScale());
    this->generateObstacles();

    simTimer = new QTimer(this);
    connect(simTimer, SIGNAL(timeout()), this, SLOT(stepSimulation()));
    startSimTimer(10);
}

MainController::~MainController()
{
    qDebug("deleting UI");
    delete simTimer;
}

bool MainController::removeRover()
{
    if(sr2){
        delete sr2;
        sr2 = 0;
        return true;
    }
    return false;
}

void MainController::closeEvent(QCloseEvent *event)
{
    this->removeRover();
    delete sky;
    delete ground;
    event->accept();
}

void MainController::startSimTimer(int msec)
{
    simTimer->start(msec);
}

void MainController::stopSimTimer()
{
    simTimer->stop();
}

void MainController::stepSimulation()
{
    if(sr2) sr2->updateRover();
    arena->simulatStep();
}

void MainController::stepTimevals()
{
    arena->simTimeStep = m_simTool.getTimeStep();
    arena->simFixedTimeStep = m_simTool.getFixedTimeStep();
    arena->simSubSteps = m_simTool.getSubSteps();
}

void MainController::simGravity()
{
    arena->setGravity(m_tTool.gravity());
}

void MainController::generateObstacles()
{
    btVector3   tempPlace,tempSize;
    float   alphaYaw,volume;
    int i;
	
    arena->deleteGroup(OBSTACLE_GROUP);
	
    if(m_oTool.obstacleCount() == 0) return;
    for(i=0;i<m_oTool.obstacleCount();i++)
    {
        tempPlace.setX(Randomn()*arena->worldSize().x());
        tempPlace.setY(Randomn()*arena->worldSize().y());
        if(tempPlace.x() < 5 && tempPlace.y() < 5){ tempPlace.setX(5);} // keep all obstacles away from rover start position
        tempPlace.setZ(ground->maxHeight() + m_oTool.dropHeight());
		
        tempSize.setX(m_oTool.minOLength() + Randomn()*(m_oTool.maxOLength() - m_oTool.minOLength()));
        tempSize.setY(m_oTool.minOWidth() + Randomn()*(m_oTool.maxOWidth() - m_oTool.minOWidth()));
        tempSize.setZ(m_oTool.minOHeight() + Randomn()*(m_oTool.maxOHeight() - m_oTool.minOHeight()));
        alphaYaw = (m_oTool.minOYaw() + Randomn()*(m_oTool.maxOYaw() - m_oTool.minOYaw()));
		
        switch(m_oTool.obstacleType())
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
        float mass = m_oTool.density() * (volume);
		
        arena->placeObstacleShapeAt(tempPlace,alphaYaw,mass);
    }
}

void MainController::removeAllObstacles()
{
    m_oTool.setObstacleCount(0);
    arena->deleteGroup(OBSTACLE_GROUP);
}

void MainController::openTerrainFile()
{
    // open an Open File dialog to look for a BMP image to represent a height map
    QString terrainFilename = QFileDialog::getOpenFileName(this,tr("Open Terrain"), "/Users",tr("Image File (*.png)"));

    if(terrainFilename == NULL) return; // if cancel is pressed dont do anything

    labelTerrainFilename->setText(terrainFilename);

    this->removeRover();
    if(ground) delete ground;
    ground = new terrain(labelTerrainFilename->text(), glView);
    m_tTool.setScale(ground->terrainScale());
    this->generateObstacles();    // fill the world with obstacles
}

void MainController::flattenTerrain()
{
    labelTerrainFilename->setText("NULL");
    this->removeRover();
    if(ground) delete ground;
    ground = new terrain(NULL, glView);
    this->generateObstacles();    // fill the world with obstacles
}

void MainController::saveTerrainFile()
{
    QString terrainFilename = QFileDialog::getSaveFileName(this,tr("Save Terrain PNG"), "/Users",tr("Image File (*.png)"));

    if(terrainFilename == NULL) return; // if cancel is pressed dont do anything

    if(!terrainFilename.endsWith(".png")) terrainFilename.append(".png");

    labelTerrainFilename->setText(terrainFilename);
    ground->saveTerrain(terrainFilename);
}

void MainController::rescaleTerrain()
{
    bool bot = this->removeRover();

    ground->terrainRescale(m_tTool.scale());
    arena->setGravity(m_tTool.gravity());

    if(bot){
        sr2 = new rover(glView);
        sr2->placeRoverAt(btVector3(1,1,ground->terrainHeightAt(btVector3(1,1,0)) + 3));
    }
    this->generateObstacles();    // fill the world with obstacles
}

/////////////////////////////////////////
// user input
/////////////
void MainController::keyPressEvent(QKeyEvent *event)
{
    switch(event->key()){
        case ' ':
        {
            arena->toggleIdle();
            return;
        }
        case 'v':
        case 'V':
        {
            glView->getCamera()->cameraToggleView();
            if(glView->getCamera()->cameraView == RoverView) glView->setViewAngle(0.5);
            else glView->setViewAngle(1.0);
            return;
        }
        case '[':
        {
            ground->terrainRaise(glView->getCamera()->cameraDirection(),m_tTool.increment(),m_tTool.diameter());
            return;
        }
        case ']':
        {
            ground->terrainLower(glView->getCamera()->cameraDirection(),m_tTool.increment(),m_tTool.diameter());
            return;
        }
        case 'n':
        case 'N':
        {
            this->removeRover();
            sr2 = new rover(glView);
            sr2->placeRoverAt(btVector3(1,1,ground->terrainHeightAt(btVector3(1,1,0))));
            return;
        }
        case '-':
            glView->setViewAngle(glView->getViewAngle()-0.1);
            return;
        case '=':
            glView->setViewAngle(glView->getViewAngle()+0.1);
            return;
        default:
            break;
    }

    if(sr2){
        switch(event->key()){
        case 'b':
        case 'B':
            {
                sr2->resetRover();
                break;
            }
        case 'f':
        case 'F':
            {
                sr2->m_bodyParts[0]->applyCentralImpulse(btVector3(sin(sr2->heading),cos(sr2->heading),10));
                break;
            }
        case 'l':
        case 'L':
            {
                sr2->toggleSensors();
                break;
            }
        case Qt::Key_Up:
            {
                if(event->modifiers() & Qt::ShiftModifier){
                    if(sr2->tiltAngle >= 45) break;
                    sr2->tiltAngle += 0.5;
                }
                else{
                    sr2->incRightSpeed(1);
                    sr2->incLeftSpeed(1);
                }
                break;
            }
        case Qt::Key_Down:
            {
                if(event->modifiers() & Qt::ShiftModifier){
                    if(sr2->tiltAngle <= -45) break;
                    sr2->tiltAngle -= 0.5;
                }
                else{
                    sr2->incRightSpeed(-0.5);
                    sr2->incLeftSpeed(-0.5);
                }
                break;
            }
        case Qt::Key_Right:
            {
                if(event->modifiers() & Qt::ShiftModifier) sr2->panAngle -= 0.5;
                else{
                    //m_oldSpeed = (sr2->leftSpeed>sr2->rightSpeed)? sr2->leftSpeed:sr2->rightSpeed;
                    sr2->incRightSpeed(-0.1);
                    sr2->incLeftSpeed(0.1);
                }
                break;
            }
        case Qt::Key_Left:
            {
                if(event->modifiers() & Qt::ShiftModifier) sr2->panAngle += 0.5;
                else{
                    //m_oldSpeed = (sr2->leftSpeed>sr2->rightSpeed)? sr2->leftSpeed:sr2->rightSpeed;
                    sr2->incRightSpeed(0.1);
                    sr2->incLeftSpeed(-0.1);
                }
                break;
            }

        default:
            break;
        }
    }
}

void MainController::keyReleaseEvent(QKeyEvent *event)
{
    if(!sr2) return;
    switch(event->key()){
        case Qt::Key_Up:
        {
            //sr2->stopRover();
            break;
        }
        case Qt::Key_Down:
        {
            sr2->stopRover();	
            break;
        }
        case Qt::Key_Left:
        {
            float avgSpeed = (sr2->rightSpeed + sr2->leftSpeed) / 2;
            sr2->setLeftSpeed(avgSpeed);
            sr2->setRightSpeed(avgSpeed);
            break;
        }
        case Qt::Key_Right:
        {
            float avgSpeed = (sr2->rightSpeed + sr2->leftSpeed) / 2;
            sr2->setRightSpeed(avgSpeed);
            sr2->setLeftSpeed(avgSpeed);
            break;
        }
    }
}

/////////////////////////////////////////
// GUI labels
/////////////
void MainController::updateGUI()
{
    if(sr2){
        labelRoverPosition->setText(QString("(%1 ,%2)").arg(sr2->position.x(),0,'f',2).arg(sr2->position.y(),0,'f',2));
        labelRoverSpeed->setText(QString("(%1 ,%2)").arg(sr2->leftSpeed,0,'f',2).arg(sr2->rightSpeed,0,'f',2));
        labelRoverHeading->setText(QString().setNum(RADTODEG(sr2->heading),'f',2));
        labelRoverPitch->setText(QString().setNum(RADTODEG(sr2->pitch),'f',1));
        labelRoverRoll->setText(QString().setNum(RADTODEG(sr2->roll),'f',1));
        labelLeftEncoder->setText(QString().setNum(sr2->leftEncoder()));
        labelRightEncoder->setText(QString().setNum(sr2->rightEncoder()));
    }

    btVector3 cameraParam = glView->getCamera()->cameraPitchYawZoom();
    labelCameraYaw->setText(QString().setNum(cameraParam.x(),'f',1));
    labelCameraPitch->setText(QString().setNum(cameraParam.y(),'f',1));
    labelCameraDist->setText(QString().setNum(cameraParam.z(),'f',1));
    labelCameraZoom->setText(QString().setNum(glView->getViewAngle(),'f',1));
    labelCameraCrosshair->setText(QString("(%1 ,%2)").arg(glView->getCamera()->cameraDirection().x(),0,'f',1)
                                  .arg(glView->getCamera()->cameraDirection().y(),0,'f',1));

}
