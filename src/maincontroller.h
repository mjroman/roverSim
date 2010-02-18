#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <QtGui/QMainWindow>
#include "simcontrol.h"
#include "ui_maincontroller.h"
#include "tools/obstacletool.h"
#include "tools/simtool.h"
#include "tools/terraintool.h"

class simControl;

class MainController : public QMainWindow, private Ui::MainController
{
    Q_OBJECT
	
private:
	simControl		*SController;
	
    obstacleTool    m_oTool;
    simtool         m_simTool;
    terrainTool     m_tTool;

public:
    MainController(QWidget *parent = 0);
    ~MainController();

    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

public slots:
	void stepTimevals();
	void simGravity();
	void openGround();
	void saveGround();
	void rescaleGround();
	void generateObstacles();
	void removeAllObstacles();

    void closeEvent(QCloseEvent *event);
    void updateGUI();
};

#endif // MAINCONTROLLER_H
