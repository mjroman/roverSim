#ifndef MainGUI_H
#define MainGUI_H

#include <QtGui/QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QDataStream>
#include "simcontrol.h"
#include "ui_mainGUI.h"
#include "tools/obstacletool.h"
#include "tools/simtool.h"
#include "tools/terraintool.h"

class simControl;

// the following enum is used under the server update switch
enum serverCommand {
		ROBOT,
		OBSTACLES,
		TERRAIN,
		SIMULATION,
		STRING
};

class MainGUI : public QMainWindow, private Ui::MainGUI
{
    Q_OBJECT
	
private:
	QTcpServer		m_tcpServer;
	QTcpSocket		*m_tcpSocket;
	quint16			m_blockSize;
	QDataStream		inStream;
	QDataStream		outStream;
	
	simControl		*SController;
	
    obstacleTool    m_oTool;
    simtool         m_simTool;
    terrainTool     m_tTool;

	void serverStart();

public:
    MainGUI(QWidget *parent = 0);
    ~MainGUI();

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

public slots:
	void stepTimevals();
	void simGravity();
	void openGround();
	void saveGround();
	void rescaleGround();
	void flattenGround();
	void generateObstacles();
	void removeAllObstacles();
	void newRover();
	void waypointInfo();
	void cameraFreeView();
	void cameraRoverCenter();
	void cameraRoverFollow();
	void cameraRoverView();
	void cameraRoverPanCam();
	
	void serverAcceptConnect();
	void serverDisconnect();
	void serverUpdate();

    void closeEvent(QCloseEvent *event);
    void updateGUI();
};

#endif // MainGUI_H
