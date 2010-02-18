#include "maincontroller.h"
#include "camera.h"
#include "terrain.h"
#include "rover.h"


MainController::MainController(QWidget *parent)
:
QMainWindow(parent),
m_oTool(this),
m_simTool(this),
m_tTool(this)
{
    qDebug("UI setup");
    setupUi(this);
    move(240,22);
    resize(1200,700);
	
    m_oTool.close();
    m_simTool.close();
    m_tTool.close();

	SController = new simControl(glView);
	
    // menu bar connections
// world menu
    connect(actionSim_Timing, SIGNAL(triggered()), &m_simTool, SLOT(show()));
// obstacles menu
    connect(actionRandomize_Obstacles, SIGNAL(triggered()), SController, SLOT(generateObstacles()));
    connect(actionRemove_Obstacles, SIGNAL(triggered()), SController, SLOT(removeObstacles()));
    connect(actionObstacle_Parameters, SIGNAL(triggered()), &m_oTool, SLOT(raise()));
// terrain menu
    connect(actionOpen_Terrain, SIGNAL(triggered()), this, SLOT(openGround()));
    connect(actionSave_Terrain, SIGNAL(triggered()), this, SLOT(saveGround()));
    connect(actionFlatten_Terrain, SIGNAL(triggered()), SController, SLOT(flattenGround()));
   	connect(actionTerrain_Parameters, SIGNAL(triggered()), &m_tTool, SLOT(raise()));

    // tool bar Simulation timing
    connect(&m_simTool, SIGNAL(paramUpdate()), this, SLOT(stepTimevals()));
    // tool bar obstacle generation button signal
    connect(&m_oTool, SIGNAL(regenerateObstacles()), this, SLOT(generateObstacles()));
    // tool bar terrain gravity update
    connect(&m_tTool, SIGNAL(gravityUpdate()), this, SLOT(simGravity()));
    // tool bar terrain scale update
    connect(&m_tTool, SIGNAL(scaleUpdate()), this, SLOT(rescaleGround()));

    connect(glView, SIGNAL(refreshView()), this, SLOT(updateGUI()));
	
    labelTerrainFilename->setText(SController->getGround()->terrainFilename());
    
    m_tTool.setScale(SController->getGround()->terrainScale());
}

MainController::~MainController()
{
    qDebug("deleting UI");
}

void MainController::closeEvent(QCloseEvent *event)
{
    delete SController;
    event->accept();
}

void MainController::stepTimevals()
{
    SController->stepTimevals(m_simTool.getTimeStep(),m_simTool.getFixedTimeStep(),m_simTool.getSubSteps());
}

void MainController::simGravity()
{
    SController->setGravity(m_tTool.gravity());
}

void MainController::openGround()
{
	// open an Open File dialog to look for a BMP image to represent a height map
    QString filename = QFileDialog::getOpenFileName(this,tr("Open Terrain"), tr("/Users"),tr("Image File (*.png)"));
	if(filename == NULL) return; // if cancel is pressed dont do anything
	SController->openNewGround(filename);
}

void MainController::saveGround()
{
	QString filename = QFileDialog::getSaveFileName(this,tr("Save Terrain PNG"), tr("/Users"),tr("Image File (*.png)"));
	if(filename == NULL) return; // if cancel is pressed dont do anything

    if(!filename.endsWith(".png")) filename.append(".png");
	SController->getGround()->saveTerrain(filename);
}

void MainController::rescaleGround()
{
    SController->rescaleGround(m_tTool.scale());
    SController->setGravity(m_tTool.gravity());
}

void MainController::generateObstacles()
{
	SController->m_obstType = m_oTool.obstacleType();
	SController->m_obstCount = m_oTool.obstacleCount();
	SController->m_dropHeight = m_oTool.dropHeight();
	SController->m_minObstSize = btVector3(m_oTool.minOLength(),m_oTool.minOWidth(),m_oTool.minOHeight());
	SController->m_maxObstSize = btVector3(m_oTool.maxOLength(),m_oTool.maxOWidth(),m_oTool.maxOHeight());
	SController->m_minObstYaw = m_oTool.minOYaw();
	SController->m_maxObstYaw = m_oTool.maxOYaw();
	SController->m_obstDensity = m_oTool.density();
	SController->generateObstacles();
}

void MainController::removeAllObstacles()
{
    m_oTool.setObstacleCount(0);
	SController->removeObstacles();
}
/////////////////////////////////////////
// user input
/////////////
void MainController::keyPressEvent(QKeyEvent *event)
{
    switch(event->key()){
        case ' ':
        {
			SController->pauseSim();
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
			SController->getGround()->terrainRaise(glView->getCamera()->cameraDirection(),m_tTool.increment(),m_tTool.diameter());
            return;
        }
        case ']':
        {
            SController->getGround()->terrainLower(glView->getCamera()->cameraDirection(),m_tTool.increment(),m_tTool.diameter());
            return;
        }
        case 'n':
        case 'N':
        {
			SController->removeRover();
			SController->newRover(glView);
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

	rover *sr2;
	sr2 = SController->getRover();
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
                //sr2->m_bodyParts[0]->applyCentralImpulse(btVector3(sin(sr2->heading),cos(sr2->heading),10));
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
	rover *sr2;
	sr2 = SController->getRover();
 
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
	rover *sr2;
	sr2 = SController->getRover();
	// GUI SR2 properties
    if(sr2){
        labelRoverPosition->setText(QString("(%1 ,%2)").arg(sr2->position.x(),0,'f',2).arg(sr2->position.y(),0,'f',2));
        labelRoverSpeed->setText(QString("(%1 ,%2)").arg(sr2->leftSpeed,0,'f',2).arg(sr2->rightSpeed,0,'f',2));
        labelRoverHeading->setText(QString().setNum(RADTODEG(sr2->heading),'f',2));
        labelRoverPitch->setText(QString().setNum(RADTODEG(sr2->pitch),'f',1));
        labelRoverRoll->setText(QString().setNum(RADTODEG(sr2->roll),'f',1));
        labelLeftEncoder->setText(QString().setNum(sr2->leftEncoder()));
        labelRightEncoder->setText(QString().setNum(sr2->rightEncoder()));
    }

// GUI camera properties
    btVector3 cameraParam = glView->getCamera()->cameraPitchYawZoom();
	labelCameraView->setText(glView->getCamera()->cameraViewName());
    labelCameraYaw->setText(QString().setNum(cameraParam.x(),'f',1));
    labelCameraPitch->setText(QString().setNum(cameraParam.y(),'f',1));
    labelCameraDist->setText(QString().setNum(cameraParam.z(),'f',1));
    labelCameraZoom->setText(QString().setNum(glView->getViewAngle(),'f',1));
    labelCameraCrosshair->setText(QString("(%1 ,%2)").arg(glView->getCamera()->cameraDirection().x(),0,'f',1)
                                  .arg(glView->getCamera()->cameraDirection().y(),0,'f',1));
							
	labelTerrainFilename->setText(SController->getGround()->terrainFilename());
	m_tTool.setScale(SController->getGround()->terrainScale());
}

/////////////////////////////////////////
// network functions
/////////////
