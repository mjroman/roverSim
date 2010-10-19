#ifndef PATHPLAN_H
#define PATHPLAN_H

#include <QList>
#include "simglobject.h"
#include "utility/structures.h"

class btCollisionObject;
class cSpace;
class obstacles;

typedef enum _PathDisplay
{
	PP_DEBUG = 0,
	PP_CROWFLY,
	PP_CURRENTSEARCH,
	PP_RANGEFAN,
	PP_BASELINE,
	PP_LIGHTTRAIL,
	PP_BUILDPATH
} PathDisplay;

class cCallback
{
    public:
        virtual void Execute() const =0;
};

template <class objectInst>
class ACallback : public cCallback
{
    public:
		typedef void (objectInst::*functCall)();
		
        ACallback(objectInst  *instPointer, functCall  functPointer)
        {
            inst   = instPointer;
            method 	= functPointer;
        }
        
        virtual void Execute() const 
        {
        	if(method)(inst->*method)();
        }

        void SetCallback (objectInst  *instPointer, functCall  functPointer)
        {
            inst   = instPointer;
            method 	= functPointer;
        }
	private:
        objectInst  *inst;
        functCall  	method;
};

class pathPlan : public simGLObject
{
	Q_OBJECT
private:
	obstacles*									m_blocks;
	cSpace*										m_CS;				// the Configuration Space the path is calculated in
	
	QList<rankPoint>							m_pointPath;		// global point containter, used while searching for a path
	goalPath									m_GP;				// the shortest Goal Path
	QList<rankPoint>							m_trailPath;		// holds the step path for limited range sensor paths
	
	QColor										m_color;
	QTime										m_time;
	float										m_range;			// holds the sensor range or distance, if 0 then infinite range
	float										m_margin;			// the size of obstacle growth for C-Space creation 
	float										m_step;				// the distance traveled on the path inbetween limited range path readings
	bool										m_visibilityType;	// bool holding state of path planning based on visible nodes only
	
	rankPoint									m_startPoint;		// start point of the path
	rankPoint									m_midPoint;
	rankPoint									m_goalPoint;		// calculate a path to this point
	
	float										m_goalDistance;		// straight line distance to the goal from a midpoint
	float										m_straightDistance;	// straight line distance from the start of the path to the goal
	btCollisionObject*							m_goalOccluded;		// hold the object the goal is inside if the goal is occluded
	float										m_progressLimit;	// holds the maximum distance of progress before quitting path search
	float										m_efficiencyLimit;	// minimum efficiency exit limit for searching a path to the goal
	
	QList<minimaPoint>							m_minimaList;		// a list of all local minima locations and spin directions
	int											m_spinDirection;	// 0=no path limiting, 1=right side path limiting, -1=left side path limiting
	float										m_spinProgress;		// the distance driven after a local minima has been reached
	int											m_spinProgressBase;	// 0 = distance based progress 1 = step based progress
	
	PathState									m_state;			// holds the state of the search for a path to the goal
	
	bool										m_drawSwitch;
	QList< ACallback<pathPlan> >				m_drawingList;		// holds all drawing callbacks
	QList< ACallback<pathPlan>* >				m_displayList;		// holds a pointer to the objects that should be currently drawn
	
	// debugging
	int											m_linkViewIndex;	// if C key is pressed this holds the view point on the shortest path
	QList<rankPoint>							contactPoints;		// holds all visible points at current linkViewIndex on the shortest path
	QList<btVector3> 							hitPoints;

	void generateCspace();
	bool isGoalInRange();
	PathState cycleToGoal();
	bool AStarSearch();
	
	goalPath reconstructPath(rankPoint here, QList<rankPoint> list);
	bool clearLocalMinima(QList<rankPoint>& list,float& dist);
	btCollisionObject* isRayBlocked(rankPoint from,rankPoint to, btVector3* point = NULL);
	void getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right);
	QList<rankPoint> quickSortFScoreLessthan(QList<rankPoint> list);
	QList<rankPoint> getVisablePointsFrom(rankPoint here);

	void drawDebugPath();
	void drawCrowFlyLine();
	void drawCurrentSearchPath();
	void drawRangeFan();
	void drawPathBuildLine();
	void drawPathBaseLine();
	void drawLightTrail();
	
public:
	pathPlan(obstacles *obs, simGLView* glView = NULL);
	~pathPlan();
	
	void goForGoal(btVector3 start, btVector3 end);
	void reset();
	
	btVector3 getStartPoint() { return m_startPoint.point; }
	btVector3 getGoalPoint() { return m_goalPoint.point; }
	float getGoalDistance() { return m_goalDistance; }
	const goalPath* getShortestPath() const { return &m_GP; }
	float getShortestLength() { return m_GP.length; }
	
	const QColor getColor() const { QColor c = m_color; c.setAlphaF(1.0); return c; }
	const float getRange() const { return m_range; }
	const float getMargin() const { return m_margin; }
	const float getStep() const { return m_step; }
	const bool getVisibilityType() const { return m_visibilityType; }
	const float getEffLimit() const { return m_efficiencyLimit; }
	const float getSpinLimit() const { return m_spinProgress; }
	int	getSpinBase() { return m_spinProgressBase; }
	const int getState() const { return m_state; }
	
	void setColor(QColor color) { m_color = color; m_color.setAlphaF(0.45); }
	void setRange(float r) { m_range = fabs(r); }
	void setMargin(float m) { m_margin = fabs(m); }
	void setStep(float s) { m_step = s; }
	void setVisibilityType(bool v) { m_visibilityType = v; }
	void setEffLimit(float e) { m_efficiencyLimit = e; }
	void setSpinLimit(float s) { m_spinProgress = s; }
	void setSpinBase(int b) { m_spinProgressBase = b; }
	void setDrawSwitch(bool x) { m_drawSwitch = x; }
	void setShortestPath(goalPath gp) { m_GP = gp; displayPath(true);}
	void setState(int st) { m_state = (PathState)st; }
	
	bool  m_displayDebug;
	bool  m_displayCrowFly;
	bool  m_displayPath;
	bool  m_displayLightTrail;
	bool  m_displayCS;
	
public slots:	
	void displayDebug(bool x);
	void displayCrowFly(bool x);
	void displayCurrentSearch(bool x);
	void displayRangeFan(bool x);
	void displayPath(bool x);
	void displayLightTrail(bool x);
	void displayCspace(bool x);
	void displayBuildPath(bool x);
	
	void togglePathReset();
	void togglePathPoint(int dir);
	void renderGLObject();
};
#endif // PATHPLAN_H