#ifndef MainGUI_H
#define MainGUI_H

#include <QtGui/QMainWindow>
#include <QTcpServer>
#include <QTcpSocket>
#include <QDataStream>
#include "ui_mainGUI.h"

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

		public:
			MainGUI(QWidget *parent = 0);
			~MainGUI();

			void printText(QString s);
			void keyPressEvent(QKeyEvent *event);
			void keyReleaseEvent(QKeyEvent *event);

		public slots:
			void showSimTiming();
			void showTerrainTool();
			void showObstacleTool();
			void showAutomatorTool();

			void terrainChanged();

			void newRover();
			void roverMenuState(bool state);
			
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
		signals:
			void executeMission();

		private:
			QTcpServer		m_tcpServer;
			QTcpSocket		*m_tcpSocket;
			quint16			m_blockSize;
			QDataStream		inStream;
			QDataStream		outStream;

			simControl		*SController;

			void helpText();
			void serverStart();
			void initGUISettings();
		//void mousePressEvent(QMouseEvent *event);
		};

#endif // MainGUI_H
