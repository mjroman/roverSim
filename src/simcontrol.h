#ifndef SIMCONTROL_H
#define	SIMCONTROL_H

#include <QTimer>
#include <QDir>
#include "mainGUI.h"
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
class QFile;


class simControl : public QObject
{
	Q_OBJECT

	public:
		simControl(MainGUI* parent, simGLView* vw=0);
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
		void runConfigWorld();
		void runIteration();
		void newRover(btVector3 start = btVector3(1,1,0));
		bool removeRover();
		
		void showWaypointTool();
		void showNavTool();
		void showPathTool();
		void showPathView(int dir);

	signals:
		void pathView(int);
		void roverState(bool);

	private:
		MainGUI			*m_parent;
		physicsWorld    *arena;
		terrain         *ground;
		obstacles		*blocks;
		skydome         *sky;
		SR2rover		*sr2;
		autoCode		*autoNav;
		waypointTool	*wTool;
		pathTool		*pTool;
		double          delTime;
		simGLView		*m_view;
		
		QList<float>	m_densityFields;
		int 			m_iterations;
		float			m_pathSizeMin,m_pathSizeMax;
		QFile			*m_statsFile;
		QDir			m_trialLocation;
	};
#endif  //SIMCONTROL_H