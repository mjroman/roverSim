#ifndef MainGUI_H
#define MainGUI_H

#include <QtGui/QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include "simcontrol.h"
#include "ui_mainGUI.h"
#include "tools/obstacletool.h"
#include "tools/simtool.h"
#include "tools/terraintool.h"

class simControl;

class MainGUI : public QMainWindow, private Ui::MainGUI
{
    Q_OBJECT
	
private:
	QTcpServer		m_TCPserver;
	QTcpSocket		*m_serverConnection;
	simControl		*SController;
	
    obstacleTool    m_oTool;
    simtool         m_simTool;
    terrainTool     m_tTool;

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
	void cameraFreeView();
	void cameraRoverCenter();
	void cameraRoverFollow();
	void cameraRoverView();
	void cameraRoverPanCam();
	
	void serverStart();
	void serverAcceptConnect();
	void serverUpdate();
	void serverError(QAbstractSocket::SocketError socketError);

    void closeEvent(QCloseEvent *event);
    void updateGUI();
};

#endif // MainGUI_H
