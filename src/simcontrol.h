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
class pathTool;
class pathPlan;


class simControl : public QObject
{
    Q_OBJECT
	private:
		physicsWorld    *arena;
		terrain         *ground;
		obstacles		*blocks;
		skydome         *sky;
		SR2rover		*sr2;
		autoCode		*autoNav;
		pathTool		*m_pTool;
	    double          delTime;
		simGLView		*glView;
		QList<WayPoint>	waypointList;
		
	public:
		simControl(simGLView* vw=0);
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
		QList<WayPoint>* getWaypointList() { return &waypointList; }
		void addWaypointAt(int uuid, float x,float y, WPstate st=WPstateNew, WPscience sc=WPscienceNone, int i = -1);
		
	public slots:
		void setWaypointGroundHeight();
		void addWaypointAt(WayPoint wp, int index);
		void editWaypoint(int index);
		void resetWaypointStates();
		
		void newRover(QWidget* parent, btVector3 start = btVector3(1,1,0));
		bool removeRover();
		
		void showNavTool();
		void showPathTool();
		void showPathView(int dir);
		
	signals:
		void pathView(int);
};
#endif  //SIMCONTROL_H