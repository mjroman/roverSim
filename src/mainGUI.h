#ifndef MainGUI_H
#define MainGUI_H

#include <QtGui/QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QDataStream>
#include "simcontrol.h"
#include "ui_mainGUI.h"
#include "tools/terraintool.h"
#include "tools/waypointtool.h"

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
	
    terrainTool     m_tTool;
	waypointTool	m_wTool;

	void serverStart();
	//void mousePressEvent(QMouseEvent *event);

public:
    MainGUI(QWidget *parent = 0);
    ~MainGUI();

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

public slots:
	void showSimTiming();
	void showTerrainTool();
	void showObstacleTool();
	
	void terrainChanged();
	
	void newRover();
	void waypointSetup();
	
	void cameraFreeView();
	void cameraRoverCenter();
	void cameraRoverFollow();
	void cameraRoverView();
	void cameraRoverPanCam();
	
	void serverAcceptConnect();
	void serverDisconnect();
	void serverUpdate();

	void screenSize();
    void closeEvent(QCloseEvent *event);
	void updateGUI();
};

#endif // MainGUI_H
