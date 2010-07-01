#ifndef SIMCONTROL_H
#define	SIMCONTROL_H

#include <QTimer>
#include "physicsWorld.h"
#include "utility/structures.h"

class terrain;
class SR2rover;
class skydome;
class simGLView;
class autoCode;
class pathPlan;


class simControl : public QObject
{
    Q_OBJECT
	private:
		physicsWorld    *arena;
		terrain         *ground;
		skydome         *sky;
		SR2rover		*sr2;
		autoCode		*autoNav;
		pathPlan		*path;
		QTimer          *simTimer;
	    double          delTime;
		simGLView		*glView;
		QList<WayPoint>	waypointList;
		pickValue		m_pickingObject;
		
	public:
		// obstacle members
		int				m_obstType;
		int				m_obstCount;
		float			m_dropHeight;
		btVector3		m_minObstSize;
		btVector3		m_maxObstSize;
		float			m_minObstYaw;
		float			m_maxObstYaw;
		float			m_obstDensity;
		
		simControl(simGLView* vw=0);
		~simControl();
		
		// simulation control functions
		void setGravity(btVector3 g);
		void startSimTimer(int msec);
		void stopSimTimer();
		
		void stepTimevals(float tStep,float fixedtStep,int subSteps);
		void pauseSim();
	
		// obstacles setting functions
		//void setObstacleData(int param, char *data);
	
		// terrain control functions
		terrain* getGround() { return ground; }
		void rescaleGround(btVector3 scale);
		
		// rover control functions
		SR2rover* getRover() { return sr2; }
		int	parameterRover(quint8 p);
		bool removeRover();
		
		// autonomous rover control object
		autoCode* getAutoNav() { return autoNav; }
		
		// waypoint functions
		QList<WayPoint>* getWaypointList() { return &waypointList; }
		void addWaypointAt(int uuid, float x,float y, WPstate st=WPstateNew, WPscience sc=WPscienceNone, int i = -1);
		
	public slots:
	// obstacle control functions
		void generateObstacles();
		void removeObstacles();
		void pickObstacle(btVector3 camPos,btVector3 mousePos);
		void moveObstacle(btVector3 camPos,btVector3 mousePos);
		void dropObstacle();
		void spinObstacle(float spin);
		void orientObstacle();
		void stepSim();
		
		void openNewGround(QString filename);
		void flattenGround();
		void setWaypointGroundHeight();
		void addWaypointAt(WayPoint wp, int index);
		void editWaypoint(int index);
		void resetWaypointStates();
		void generatePath();
		
		void newRover(QWidget* parent);
		void showNavTool();
};
#endif  //SIMCONTROL_H