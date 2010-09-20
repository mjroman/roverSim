#ifndef SIMCONTROL_H
#define	SIMCONTROL_H

#include <QTimer>
#include "physicsWorld.h"
#include "utility/structures.h"

class terrain;
class obstacles;
class SR2rover;
class skydome;
class simGLView;
class autoCode;
class waypointTool;
class pathTool;
class pathPlan;


class simControl : public QObject
{
	Q_OBJECT

	public:
		simControl(QWidget* parent, simGLView* vw=0);
		~simControl();

		// simulation control functions	
		void stepTimevals(float tStep,float fixedtStep,int subSteps);	
		void pauseSim();
		void setGravity(btVector3 g);

		// terrain control functions
		terrain* getGround() { return ground; }

		// obstacles setting functions
		obstacles* getBlocks() { return blocks; }

		// rover control functions
		SR2rover* getRover() { return sr2; }

		// autonomous rover control object
		autoCode* getAutoNav() { return autoNav; }

		// waypoint functions
		void addWaypoint(int uuid, float x,float y, WPstate st=WPstateNew, WPscience sc=WPscienceNone);

	public slots:
		void runConfigFile();
		void runIteration();
		void newRover(btVector3 start = btVector3(1,1,0));
		bool removeRover();
		
		void showWaypointTool();
		void showNavTool();
		void showPathTool();
		void showPathView(int dir);

	signals:
		void pathView(int);
		void genPaths();
		void roverState(bool);

	private:
		QWidget			*m_parent;
		physicsWorld    *arena;
		terrain         *ground;
		obstacles		*blocks;
		skydome         *sky;
		SR2rover		*sr2;
		autoCode		*autoNav;
		waypointTool	*wTool;
		pathTool		*pTool;
		double          delTime;
		simGLView		*glView;
		
		int 			m_iterations;
		float			m_pathSizeMin,m_pathSizeMax;
	};
#endif  //SIMCONTROL_H