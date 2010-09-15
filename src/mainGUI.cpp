#include "mainGUI.h"
#include "camera.h"
#include "terrain.h"
#include "obstacles.h"
#include "sr2rover.h"
#include "autoCode.h"
#include "utility/definitions.h"
#include "tools/simtool.h"

#define GUIGROUP	"GUI_windows" // settings group key value

MainGUI::MainGUI(QWidget *parent)
:
QMainWindow(parent),
m_tTool(this),
m_wTool(this)
{
    setupUi(this);

	setWindowTitle("Rover Simulator");
	
	QCoreApplication::setOrganizationName("OUengineering");
	QCoreApplication::setOrganizationDomain("i-borg.engr.ou.edu");
	QCoreApplication::setApplicationName("Rover_Sim");

	QDir temp(QCoreApplication::applicationDirPath());
	temp.cdUp();
	QDir::setCurrent(temp.path());										// set the current directory of the application
	temp.cd("../..");
	
	textConsole->append(temp.absolutePath());
	if(QFile::exists(temp.absolutePath() + "/mission/config.txt")){
		textConsole->append("found config");							// check for config file
	}
	
	
	QSettings settings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim");
	if(!QFile::exists(settings.fileName()) || !settings.contains("MainWindowGeom")){
		move(240,22);
	    resize(1200,700);
		settings.setValue("MainWindowGeom", saveGeometry());
	}
	else
		this->restoreGeometry(settings.value("MainWindowGeom").toByteArray());

    m_tTool.hide();
	m_wTool.hide();

	SController = new simControl(glView);
	
    // menu bar connections
// view menu
	connect(actionFullScreen, SIGNAL(triggered()), this, SLOT(screenSize()));

// world menu
    connect(actionSim_Timing, SIGNAL(triggered()), this, SLOT(showSimTiming()));

// rover menu
	connect(actionNew_Rover, SIGNAL(triggered()), this, SLOT(newRover()));
	connect(actionShow_Waypoint_Editor, SIGNAL(triggered()), this, SLOT(waypointSetup()));
	connect(actionShow_Rover_Info, SIGNAL(triggered()), SController, SLOT(showNavTool()));
	connect(actionShow_Path_Info, SIGNAL(triggered()), SController, SLOT(showPathTool()));
	actionShow_Rover_Info->setEnabled(false);
	actionShow_Path_Info->setEnabled(false);

// setup the rover view menu
	connect(actionFree_View, SIGNAL(triggered()), this, SLOT(cameraFreeView()));
	connect(actionRover_Center, SIGNAL(triggered()), this, SLOT(cameraRoverCenter()));
	connect(actionRover_Follow, SIGNAL(triggered()), this, SLOT(cameraRoverFollow()));
	connect(actionRover_View, SIGNAL(triggered()), this, SLOT(cameraRoverView()));
	connect(actionRover_PanCam, SIGNAL(triggered()), this, SLOT(cameraRoverPanCam()));
	menuRoverView->setEnabled(false);

// obstacles menu
    connect(actionRandomize_Obstacles, SIGNAL(triggered()), SController->getBlocks(), SLOT(generate()));
	connect(actionRemove_Obstacles, SIGNAL(triggered()), SController->getBlocks(), SLOT(eliminate()));
	connect(actionSave_ObstacleLayout, SIGNAL(triggered()), SController->getBlocks(), SLOT(saveLayout()));
	connect(actionLoad_ObstacleLayout, SIGNAL(triggered()), SController->getBlocks(), SLOT(loadLayout()));
    connect(actionObstacle_Parameters, SIGNAL(triggered()), this, SLOT(showObstacleTool()));

// terrain menu
    connect(actionOpen_Terrain, SIGNAL(triggered()), SController->getGround(), SLOT(openTerrain()));
    connect(actionSave_Terrain, SIGNAL(triggered()), SController->getGround(), SLOT(saveTerrain()));
    connect(actionFlatten_Terrain, SIGNAL(triggered()), SController->getGround(), SLOT(flattenTerrain()));
   	connect(actionTerrain_Parameters, SIGNAL(triggered()), this, SLOT(showTerrainTool()));
	connect(SController->getGround(), SIGNAL(newTerrain()), this, SLOT(terrainChanged()));
	
    // tool bar terrain scale update
    connect(&m_tTool, SIGNAL(scaleUpdate(btVector3)), SController->getGround(), SLOT(rescaleTerrain(btVector3)));
	// tool bar add waypoint update
	connect(&m_wTool, SIGNAL(addedWP(WayPoint,int)), SController, SLOT(addWaypointAt(WayPoint,int)));
	connect(&m_wTool, SIGNAL(editedWP(int)), SController, SLOT(editWaypoint(int)));
	connect(&m_wTool, SIGNAL(resetWP()), SController, SLOT(resetWaypointStates()));
// Text Console
	connect(glView, SIGNAL(outputText(QString)), textConsole, SLOT(append(QString)));
	
// server connections
	//connect(&m_tcpServer, SIGNAL(newConnection()),this, SLOT(serverAcceptConnect()));
	m_tcpSocket = NULL;
	//this->serverStart();
	
    connect(glView, SIGNAL(refreshView()), this, SLOT(updateGUI()));
	
	this->terrainChanged();

	//glView->setFocus(Qt::OtherFocusReason);
	glView->setFocusPolicy(Qt::StrongFocus);
	
	this->helpText();
}

MainGUI::~MainGUI()
{
}

void MainGUI::closeEvent(QCloseEvent *event)
{
	QSettings settings(QSettings::IniFormat,QSettings::UserScope,"OUengineering","Rover_Sim");
	settings.setValue("MainWindowGeom", saveGeometry());
    delete SController;
    event->accept();
}

/////////////////////////////////////////
// VIEW items
/////////////
void MainGUI::screenSize()
{
	if(this->isFullScreen()) {
		this->showNormal();
		move(240,22);
		resize(1200,700);
		actionFullScreen->setText("Fullscreen Mode");
	}
	else{
		this->showFullScreen();
		actionFullScreen->setText("Exit Fullscreen");
	}
}

void MainGUI::helpText()
{
	textConsole->append("Command-Mouse :\t Rotate view");
	textConsole->append("Command-R :\t Randomize Obstacles");
	textConsole->append("Command-N :\t Drop/Remove a Rover");
	textConsole->append("Arrow Keys :\t \t Drive the Rover");
	textConsole->append("Space :\t\t Pause Physics Motion");
	textConsole->append("V :\t\t Cycles through views with Rover");
	textConsole->append("B :\t\t Resets the Rover");
	textConsole->append("L :\t\t Toggles Sensor Graphics");
}
/////////////////////////////////////////
// GUI tool views
/////////////
void MainGUI::showSimTiming()
{
	simtool sDialog(this);
	if(sDialog.exec() == QDialog::Accepted) SController->stepTimevals(sDialog.step,sDialog.fixedStep,sDialog.subStep);
}
void MainGUI::showTerrainTool()
{
	SController->getBlocks()->hideTool();

	if(m_tTool.isVisible())
		m_tTool.hide();
	else
		m_tTool.show();
}
void MainGUI::showObstacleTool()
{
	m_tTool.hide();
	
	SController->getBlocks()->showTool();
}

/////////////////////////////////////////
// Terrain editing functions
/////////////
void MainGUI::terrainChanged()
{
	labelTerrainFilename->setText(SController->getGround()->terrainShortname());
	menuRoverView->setEnabled(false);
}

/////////////////////////////////////////
// Rover creation and destruction
/////////////
void MainGUI::newRover()
{
	bool state = (SController->getRover() == 0) ? true : false;			// if there is no rover then create a new one
	
	if(state){
		SController->newRover(this);
		actionNew_Rover->setText("Remove Rover");
	}
	else{
		SController->removeRover();
		actionNew_Rover->setText("New Rover");
	}
		
	menuRoverView->setEnabled(state);
	actionShow_Rover_Info->setEnabled(state);
	actionShow_Path_Info->setEnabled(state);
}

void MainGUI::waypointSetup()
{
	m_wTool.raiseWaypointEditor(SController->getWaypointList());
}

void MainGUI::cameraFreeView(){glView->getCamera()->cameraFreeView(); glView->setViewAngle(1.0);}
void MainGUI::cameraRoverCenter(){glView->getCamera()->cameraRoverCenter(); glView->setViewAngle(1.0);}
void MainGUI::cameraRoverFollow(){glView->getCamera()->cameraRoverFollow(); glView->setViewAngle(1.0);}
void MainGUI::cameraRoverView(){glView->getCamera()->cameraRoverView(); glView->setViewAngle(0.5);}
void MainGUI::cameraRoverPanCam(){glView->getCamera()->cameraRoverPanCam(); glView->setViewAngle(1.0);}

/////////////////////////////////////////
// Keyboard and Mouse user input
/////////////
void MainGUI::keyPressEvent(QKeyEvent *event)
{
	if(!glView->hasFocus()) return;
    switch(event->key()){
        case ' ':
        {
			SController->pauseSim();
            return;
        }
		case 'A':
		{
			SController->getBlocks()->orientObstacle();
			return;
		}
		case 'C':
		{
			return;
		}
		case 'F':
		{
			glView->toggleFog();
			return;
		}
		case 'P':
		{
			glView->toggleDrawing();
			return;
		}
		case 'S':
		{
			return;
		}
        case 'V':
        {
            glView->getCamera()->cameraToggleView();
            if(glView->getCamera()->cameraView == RoverView) glView->setViewAngle(0.5);
            else glView->setViewAngle(1.0);
            return;
        }
		case 'X':
		{
			SController->showPathView(1);
			return;
		}
		case 'Z':
		{
			SController->showPathView(-1);
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
        case '-':
            glView->setViewAngle(glView->getViewAngle()-0.1);
            return;
        case '=':
            glView->setViewAngle(glView->getViewAngle()+0.1);
            return;
        default:
            break;
    }

	SR2rover *sr2;
	sr2 = SController->getRover();
    if(sr2){
        switch(event->key()){
		case 'A':
			{
				//SController->getAutoNav()->toggleAutonomous();
				break;
			}
        case 'B':
            {
                sr2->resetRobot();
                break;
            }
		case 'C':
			{
				SController->getAutoNav()->quickObstacleCheck();
				break;
			}
        case 'D':
            {
                //sr2->m_robotObjects[0]->applyCentralImpulse(btVector3(sin(sr2->heading),cos(sr2->heading),10));
                break;
            }
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
                    sr2->incRightSpeed(-REMOTETURNSENSITIVITY);
                    sr2->incLeftSpeed(REMOTETURNSENSITIVITY);
                }
                break;
            }
        case Qt::Key_Left:
            {
                if(event->modifiers() & Qt::ShiftModifier) sr2->panAngle += 0.5;
                else{
                    //m_oldSpeed = (sr2->leftSpeed>sr2->rightSpeed)? sr2->leftSpeed:sr2->rightSpeed;
                    sr2->incRightSpeed(REMOTETURNSENSITIVITY);
                    sr2->incLeftSpeed(-REMOTETURNSENSITIVITY);
                }
                break;
            }

        default:
            break;
        }
    }
}

void MainGUI::keyReleaseEvent(QKeyEvent *event)
{
	if(glView->hasFocus()){
		SR2rover *sr2;
		sr2 = SController->getRover();

		if(!sr2) return;	
		switch(event->key()){
			case Qt::Key_Up:
			{
			//sr2->stopRobot();
				break;
			}
			case Qt::Key_Down:
			{
				sr2->stopRobot();	
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
}


/////////////////////////////////////////
// GUI labels
/////////////
void MainGUI::updateGUI()
{
	SR2rover *sr2;
	sr2 = SController->getRover();
	// GUI SR2 properties
    if(sr2){
        labelRoverPosition->setText(QString("(%1 ,%2 ,%3)").arg(sr2->position.x(),0,'f',2).arg(sr2->position.y(),0,'f',2).arg(sr2->position.z(),0,'f',2));
        labelRoverSpeed->setText(QString("(%1 ,%2)").arg(sr2->leftSpeed,0,'f',2).arg(sr2->rightSpeed,0,'f',2));
		labelDiffAngle->setText(QString().setNum(RADTODEG(sr2->differentialAngle),'f',1));
		labelRoverHeading->setText(QString().setNum(RADTODEG(sr2->heading),'f',2));
        labelRoverPitch->setText(QString().setNum(RADTODEG(sr2->pitch),'f',1));
        labelRoverRoll->setText(QString().setNum(RADTODEG(sr2->roll),'f',1));
        labelLeftEncoder->setText(QString().setNum(sr2->leftEncoder()));
        labelRightEncoder->setText(QString().setNum(sr2->rightEncoder()));
		labelRoverOdometer->setText(QString("%1m").arg(sr2->odometer,0,'f',2));
    }

//	labelDebug->setText(QString("random seed = %1\n").arg(seed));
	
// GUI camera properties
    btVector3 cameraParam = glView->getCamera()->cameraPitchYawZoom();
	labelCameraView->setText(glView->getCamera()->cameraViewName());
    labelCameraYaw->setText(QString().setNum(cameraParam.x(),'f',1));
    labelCameraPitch->setText(QString().setNum(cameraParam.y(),'f',1));
    labelCameraDist->setText(QString().setNum(cameraParam.z(),'f',1));
    labelCameraZoom->setText(QString().setNum(glView->getViewAngle(),'f',1));
    labelCameraCrosshair->setText(QString("(%1 ,%2)").arg(glView->getCamera()->cameraDirection().x(),0,'f',1)
                                  .arg(glView->getCamera()->cameraDirection().y(),0,'f',1));
}

/////////////////////////////////////////
// Save the World
/////////////

/////////////////////////////////////////
// network functions
/////////////
void MainGUI::serverStart()
{
	if (!m_tcpServer.isListening() && !m_tcpServer.listen(QHostAddress::Any,50657)) 
	{
		QMessageBox::StandardButton ret = QMessageBox::critical(this,
			tr("Server error"),
			tr("Unable to start loopback connection: %1.")
			.arg(m_tcpServer.errorString()),
			QMessageBox::Retry
			| QMessageBox::Cancel);
		if (ret == QMessageBox::Cancel)
			return;
	}
	textConsole->append("Server waiting at:"+
						QHostAddress(QHostAddress::LocalHost).toString()+
						"\tPort:"+
						QString::number(m_tcpServer.serverPort()));
}
void MainGUI::serverAcceptConnect()
{
	m_tcpSocket = m_tcpServer.nextPendingConnection();
    connect(m_tcpSocket, SIGNAL(readyRead()),this, SLOT(serverUpdate()));
    connect(m_tcpSocket, SIGNAL(disconnected()), m_tcpSocket, SLOT(deleteLater()));
	connect(m_tcpSocket, SIGNAL(disconnected()), this, SLOT(serverDisconnect()));
	m_tcpServer.close();
	m_blockSize = 0;
	inStream.setDevice(m_tcpSocket);
	inStream.setVersion(QDataStream::Qt_4_0);
	textConsole->append(QString("Connection:")+m_tcpSocket->peerAddress().toString());
}
void MainGUI::serverDisconnect()
{
	inStream.setDevice(0);
	m_tcpSocket->disconnect();
	textConsole->append("client disconnected");
	this->serverStart();
}
void MainGUI::serverUpdate()
{
	if (m_blockSize == 0) {
        if (m_tcpSocket->bytesAvailable() < (int)sizeof(quint16)) return;
        inStream >> m_blockSize;
    }

    if (m_tcpSocket->bytesAvailable() < m_blockSize) return;

	quint8 command;
	bool state;
	quint8 parameter;
	inStream >> command;
	
	if(command == STRING)
	{
		QString textString;
		inStream >> textString;
		textConsole->append(textString);
	}
	else
	{
		inStream >> state >> parameter;
		textConsole->append(QString("%1 cmd %2 state %3 parm").arg(command).arg(state).arg(parameter));

		char *data = NULL;
		m_blockSize = m_tcpSocket->bytesAvailable();
		if(m_blockSize > 0){
			data = new char[m_blockSize];
			inStream.readRawData(data, m_blockSize);
		}

		switch((serverCommand) command){
			case ROBOT:
			//if(state) SController->setRoverData(parameter,data);
			//else SController->getRoverData(parameter,data);
			break;
			case OBSTACLES:
				//if(state) SController->setObstacleData(parameter,data);
			break;
			case TERRAIN:
			break;
			case SIMULATION:
			break;
			default:
			break;
		}
		if(data) delete [] data;
	}
	m_blockSize = 0;
}
