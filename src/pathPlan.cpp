#include "pathPlan.h"
#include "cSpace.h"
#include "obstacles.h"
#include "utility/definitions.h"
#include "utility/glshapes.h"
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

#define SPACEMARGIN			0.65

pathPlan::pathPlan(obstacles *obs,simGLView* glView)
:
simGLObject(glView),
m_blocks(obs),
m_CS(NULL),
m_range(0),
m_margin(SPACEMARGIN),
m_step(0.25),
m_visibilityType(false),
m_goalOccluded(NULL),
m_efficiencyLimit(0.3),
m_spinProgress(6),
m_spinProgressBase(0),
m_state(PS_SEARCHING),
m_drawSwitch(true),
m_linkViewIndex(0)
{
	// create callbacks to all the drawing methods, these are added to/removed from the display list
	// make sure these remain in order to match with the enumeration
	ACallback<pathPlan> drawCB(this, &pathPlan::drawDebugPath);	m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawCrowFlyLine); 		m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawCurrentSearchPath);	m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawRangeFan);			m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawPathBaseLine);		m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawLightTrail);			m_drawingList << drawCB;
	drawCB.SetCallback(this,&pathPlan::drawPathBuildLine);		m_drawingList << drawCB;
	
	m_GP.length = 0;
	m_GP.time = 0;
	m_GP.efficiency = 0;
	
	displayBuildPath(false);
	displayPath(false);
	displayLightTrail(false);
	displayCrowFly(false);
	displayDebug(false);
	displayCspace(false);
}

pathPlan::~pathPlan()
{
	m_pointPath.clear();
	contactPoints.clear();
	hitPoints.clear();
	m_GP.points.clear();
	if(m_CS) delete m_CS;
	m_CS = 0;
}

void pathPlan::reset()
{
	togglePathReset();
	m_GP.points.clear();
	m_GP.length = 0;
	m_GP.time = 0;
	m_GP.efficiency = 0;
}

void pathPlan::goForGoal(btVector3 start, btVector3 end)
{
	memset(&m_startPoint,0,sizeof(rankPoint));
	memset(&m_midPoint,0,sizeof(rankPoint));
	memset(&m_goalPoint,0,sizeof(rankPoint));
	
	m_state = PS_SEARCHING;
	m_startPoint.point = start;
	m_goalPoint.point = end;
	m_straightDistance = start.distance(end);
	m_progressLimit = m_straightDistance / m_efficiencyLimit;
	m_spinDirection = 0;
	
	displayCurrentSearch(true);								// turn on drawing the yellow search lines
	displayBuildPath(true);
	
	this->generateCspace();									// create the C-Space to compute the path in
	
	if(m_view) m_view->overlayString(QString("Searching Range %1").arg(m_range));
	
	m_time.start();											// start time of path calculation
	
	if(m_range == 0) {
		this->AStarSearch();								// if infinite range find the shortest path
		if(m_GP.length == 0) m_state = PS_PATHNOTFOUND;
		else m_state = PS_COMPLETE;
	}
	else {
		m_state = this->cycleToGoal();						// step forward on path until the goal is in range
	
		m_GP.points = m_trailPath + m_GP.points;			// prepend the step trail point list
		m_trailPath.clear();
		m_GP.length = 0;
		for(int i=0; i<m_GP.points.size()-1; i++) 
			m_GP.length += m_GP.points[i].point.distance(m_GP.points[i+1].point);	// calculate the total length
		m_startPoint = m_GP.points[0];						// set the starting point for drawing
	}
	
	if(m_goalOccluded && m_state == PS_COMPLETE) 			// path is complete but the goal is occluded
		m_state = PS_GOALOCCLUDED;							// set the state
	
	m_GP.time = m_time.elapsed();							// get the elapsed time for the path generation
	
	if(m_CS) delete m_CS;									// delete the C Space to free up memory since it is not needed
	m_CS = 0;
	m_pointPath.clear();									// clear out the construction point path
	
	displayCurrentSearch(false);							// turn off search path drawing
	displayBuildPath(false);
	displayPath(true);
			
	if(m_GP.length != 0) m_GP.efficiency = m_straightDistance/m_GP.length;
}


/////////////////////////////////////////
// Generate a new Configuration Space to plan paths around
/////////////
void pathPlan::generateCspace()
{
	if(m_CS) delete m_CS;
	m_CS = new cSpace(m_startPoint.point,m_range,m_margin,m_blocks,m_view);		// create a new Configuration Space based on the start point
	if(m_view) m_CS->drawCspace(m_displayCS);
	m_goalOccluded = NULL;
	
	if(isGoalInRange()){
		QList<btCollisionObject*>* ghostList = m_CS->getGhostList();
		for(int i=0; i < ghostList->size(); i++)							// check if goal point is inside of a cspace object
		{
			if(m_CS->isPointInsideObject(m_goalPoint.point,ghostList->at(i))){
				m_goalOccluded = ghostList->at(i);
				break;
			}
		}
	}
	
	m_midPoint = m_startPoint;												// reset all parameters due to new Cspace
	m_pointPath.clear();
	m_pointPath << m_startPoint;
}

// returns true if the goal is within sensor range of the robot
bool pathPlan::isGoalInRange()
{
	m_goalDistance = m_startPoint.point.distance(m_goalPoint.point);		// calculate the distance to the goal from the start point
	if(m_range == 0) return true;
	if(m_range <= m_goalDistance) return false;
	else return true;
}

/////////////////////////////////////////
// Path creation
/////////////
PathState pathPlan::cycleToGoal()
{
	int i;
	float progress = 0;
	m_trailPath.clear();
	m_trailPath << m_startPoint;
	m_minimaList.clear();
	
	while( progress <= m_progressLimit )										// looping condition based on path efficiency
	{	
		this->AStarSearch();													// calculate the path to the goal
		
		if(m_GP.points.size() <= 1){ 											// if only 1 point is in the path list or it is empty then no path is found
			if(m_time.elapsed() > 300000)
				return PS_TIMEOUT;
			else
				return PS_PATHNOTFOUND;
		}
		else if( m_range > m_goalDistance ) 
			return PS_COMPLETE;													// a complete path has been found and the goal is in range
		
		i = 1;
		float step = m_step;
		btVector3 strideVect = m_GP.points[i].point - m_trailPath.last().point;	// get the vector to the next step
		
		while(step > strideVect.length()){										// while the step distance is longer than the distance to the next point
			m_trailPath << m_GP.points[i];										// add the current position to the trail path
			step -= strideVect.length();
			i++;
			strideVect = m_GP.points[i].point - m_trailPath.last().point;		// find the vector to the next point on the path
		}
	
		memset(&m_startPoint,0,sizeof(rankPoint));								// create a new start point
		m_startPoint.point = step * strideVect.normalized() + m_trailPath.last().point;	// compute the new point from the end of the trail
		
		// make sure the rover doesn't step into an object's C-Space
		///////////////////////////////////////////////////////////////////////////			
		btCollisionObject* object = 0;
		if(m_trailPath.last().object) object = m_trailPath.last().object; 		// check if the end of the trail is near a C-Space object
		else if(m_GP.points[i].object) object = m_GP.points[i].object;			// or the next point on the path in the direction headed

		if(object){
			btVector3 offset = m_startPoint.point - object->getWorldTransform().getOrigin();
			offset = (0.01 * offset.normalized()) * btVector3(1,1,0);			// move over the start point 1cm from the edge of the C-Space
			m_startPoint.point += offset;
			m_startPoint.object = object;
		}

		m_CS->movePointOutsideCSpace(m_startPoint.point);						// check if the new start point is inside the C-Space
		///////////////////////////////////////////////////////////////////////////
		
		m_trailPath << m_startPoint;
		
		progress += m_step;
		m_GP.length = 0;
		m_GP.points.clear();
		
		if( !clearLocalMinima(m_trailPath,progress) ){							// if the rover gets stuck in a local minima set the state
			if(m_spinDirection == 1)
				return PS_SWITCHBACKRIGHT;
			else if(m_spinDirection == -1)
				return PS_SWITCHBACKLEFT;
			else
				return PS_LOCALMINIMA;
		}
		
		this->generateCspace();													// create new C-Space and calculate new goal distance
	}
	return PS_NOPROGRESS;														// current path length is not making progress
}


/////////////////////////////////////////
// A* search, This is where it all happens
/////////////
bool pathPlan::AStarSearch()
{
	int i,j;
	bool newNode,oldNode;
	float g_score;
	goalPath v_GP;
	rankPoint tPoint;
	QList<rankPoint> prospectPoints;									// visible nodes from current location
	QList<rankPoint> openSet;											// contains all possible nodes to go to
	QList<rankPoint> closedSet;											// contains all nodes already visited
	
	m_midPoint = m_startPoint;
	m_midPoint.parentIndex = -1;
	m_midPoint.gScore = 0;
	m_midPoint.hScore = m_midPoint.point.distance(m_goalPoint.point);
	m_midPoint.fScore = m_midPoint.gScore + m_midPoint.hScore;
	openSet << m_midPoint;
	
	while(!openSet.isEmpty())
	{
		if(m_time.elapsed() > 300000) return false; 			// 5 minute limit and no path to the goal found exit
		
		m_midPoint = openSet.takeFirst(); 						// remove node in openSet with the lowest fScore
		
		if(m_drawSwitch && m_view)								// draw the path while searching
		{
			v_GP = reconstructPath(m_midPoint,closedSet);
			m_pointPath.clear();
			m_pointPath = v_GP.points;
			m_pointPath.pop_back();
			m_view->updateGL();
		}
		
		btCollisionObject *objBlock = this->isRayBlocked(m_midPoint,m_goalPoint);	// is the ray blocked?
		
		if(objBlock == NULL || 						// if the goal is clear
		objBlock == m_goalOccluded) 				// or the blocking object is the occluded goal object
		{
			m_GP = reconstructPath(m_midPoint,closedSet);
			return true;
		}
		
		closedSet << m_midPoint;							// add current node to already visited set
		prospectPoints.clear();
		prospectPoints = getVisablePointsFrom(m_midPoint);	// get all visible nodes
		
		for(i=0; i<prospectPoints.size(); i++)				// loop through all visible prospect nodes
		{	
			oldNode = false;
			for( j = 0; j < closedSet.size(); j++){
				if(prospectPoints[i].object == closedSet[j].object && prospectPoints[i].corner == closedSet[j].corner){
					oldNode = true;
					break;
				}
			}
			if(oldNode) continue;						// skip points that have already been visited
			
			g_score = m_midPoint.gScore + m_midPoint.point.distance(prospectPoints[i].point);
			
			newNode = true;
			for( j = 0; j < openSet.size(); j++){		// check if the prospective node is new or has a lower g score
				if(prospectPoints[i].object == openSet[j].object && prospectPoints[i].corner == openSet[j].corner){
					newNode = false;
					if(g_score < openSet[j].gScore){
						openSet[j].parentIndex = closedSet.size()-1;
						openSet[j].gScore = g_score;
						openSet[j].fScore = openSet[j].gScore + openSet[j].hScore;
					}
					break;
				}
			}
			
			if(newNode){								// add new nodes to the open set
				prospectPoints[i].parentIndex = closedSet.size()-1;
				prospectPoints[i].gScore = g_score;
				prospectPoints[i].hScore = m_goalPoint.point.distance(prospectPoints[i].point);
				prospectPoints[i].fScore = prospectPoints[i].gScore + prospectPoints[i].hScore;
				openSet << prospectPoints[i];
			}
		}
		
		openSet = quickSortFScoreLessthan(openSet);
		
		////////////////////////////////////////////
		// rover POV sensor visibility switch
		if(m_visibilityType && m_range != 0){
			m_GP = reconstructPath(openSet.first(),closedSet);	// build the path
			return true;
		}
	}
	return false;
}

/////////////////////////////////////////
// Path Debugging
/////////////
void pathPlan::togglePathReset()
{
	displayPath(true);
	displayBuildPath(false);
	displayDebug(false);
	if(m_CS) delete m_CS;
	m_CS = 0;
	hitPoints.clear();
	contactPoints.clear();
}
// this is a debugging function used to see what paths are available from a given point on the path
// calling this function repeatedly cycles throught the shortest path
void pathPlan::togglePathPoint(int dir)
{
	if(m_GP.points.isEmpty()) return;

	if(m_CS) delete m_CS;
	m_CS = 0;
	hitPoints.clear();
	contactPoints.clear();
	m_spinDirection = 0;
	
	m_linkViewIndex += dir;													// increment the index
	if(m_linkViewIndex >= m_GP.points.size()) m_linkViewIndex = 0;			// make sure the index is in bounds
	else if(m_linkViewIndex < 0) m_linkViewIndex = m_GP.points.size()-1;

	if(!m_displayDebug) displayDebug(true);									// turn on debug drawing

	rankPoint here = m_GP.points[m_linkViewIndex];							// get the point to view from using the index
	here.object = 0;

	m_view->getCamera()->cameraSetDirection(here.point); 					// set the camera view to the path point
	m_CS = new cSpace(here.point,m_range,m_margin,m_blocks,m_view);			// create a new Configuration Space based on the start point
	if(m_view) m_CS->drawCspace(true);
	
	if(m_CS->isPointInsideCSpace(here.point))	
		m_view->overlayString("inside");
	
	contactPoints = getVisablePointsFrom(here);								// gather all objects extreme vertices
	
	if(!contactPoints.isEmpty()) hitPoints << contactPoints[0].point;		// show the most likely path choice
	contactPoints.prepend(m_GP.points[m_linkViewIndex]);					// push the point the current view is from for drawing
	m_view->updateGL();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Path generation utility functions
/////////////
goalPath pathPlan::reconstructPath(rankPoint here, QList<rankPoint> list)
{
	goalPath gp;
	gp.length = here.gScore + here.point.distance(m_goalPoint.point);

	gp.points.prepend(m_goalPoint);
	while(here.parentIndex != -1){
		gp.points.prepend(here);
		here = list[here.parentIndex];
	}
	gp.points.prepend(m_startPoint);
	return gp;
}

// checks local minima under limited range sensor paths for switch back condition
// sets the global spin direction and keeps track of spin progress distance
bool pathPlan::clearLocalMinima(QList<rankPoint>& list, float& dist)
{
	int index = list.size()-1;
	static float spinDist = 0;

	if(m_startPoint.object == 0 || list.size() < 3) return true;	// make sure there is an object near by and the list is big enough
	
	if(spinDist > 0){ spinDist = 0; return true; }					// skip the next local minima check after spin progress to avoid switchback condition

	btVector3 preVect = list[index-1].point - list[index-2].point;	// use the previous position to compare
	btVector3 newVect = list[index].point - list[index-1].point;	// to the new position

	if(preVect.dot(newVect) < 0){									// A switchback is detected, dot is negative if the angle is greater than 90+
		minimaPoint mp;													// save the spot where it is detected
		mp.point = list[index-1].point;
		mp.threshold = 2*m_step;

		if(!m_minimaList.isEmpty() && mp.point.distance2(m_minimaList.last().point) < SQ(m_minimaList.last().threshold)){ 
			mp.progress = m_minimaList.last().progress * 2;							// increase the progress limit by twice the previous
			mp.spin = m_minimaList.last().spin * -1;								// flip the direction of spin if near the last local minima
		}
		else {
			if(m_spinProgressBase == 0) mp.progress = m_spinProgress;				// distance based progress
			else if(m_spinProgressBase == 1) mp.progress = m_spinProgress * m_step;	// step based progress
			
			// set initial spin direction around objects here
			//////////////////////////////////////////////////////////////////////////////////////////////////
			mp.spin = (newVect.cross(preVect).z() > 0)? -1 : 1;						// 1 = CW spin -1 = CCW	
			//////////////////////////////////////////////////////////////////////////////////////////////////
		}
		m_minimaList << mp;
		m_spinDirection = m_minimaList.last().spin;

		btVector3 obstDirection = m_goalPoint.point - m_startPoint.point;

		
		while( spinDist < mp.progress && dist <= m_progressLimit )	// circumnavigate the obstacle until it has traveled past the progress distance
		{								// or until the dot product between the (current rover location and obstacle) and (goal and obstacle) is positive in the direction of spin
			this->generateCspace();
			
			if( !m_CS->movePointAroundCSpace(m_startPoint.point,obstDirection,m_step,m_spinDirection) )
				return false;

			list << m_startPoint;
			spinDist += m_step;
			dist += m_step;
			
			if(m_drawSwitch && m_view)
				m_view->updateGL();
				
			//m_view->overlayString(QString("Progress distance %1 < %2").arg(spinDist).arg(mp.progress));
		}
		m_spinDirection = 0;
	}
	return true;
}

// checks the ray between from and to, if it is clear NULL is returned else returns the object blocking
btCollisionObject* pathPlan::isRayBlocked(rankPoint from,rankPoint to, btVector3* point)
{
	QList<rankPoint> hitList;
	QList<btCollisionObject*>* ghostList = m_CS->getGhostList();
	
	for(int i=0;i<ghostList->size();i++){
		int k;
		rankPoint rp;
		rp.object = ghostList->at(i);
		
		if(rp.object == to.object && rp.object == from.object) continue; // checking the edge of an object should be ignored
		
		QList<btVector3> shapePoints = m_CS->getTopShapePoints(rp.object);
		for(int j=0;j<shapePoints.size();j++){
			if(j == shapePoints.size()-1) k=0;
			else k=j+1;
			if(m_CS->segmentIntersection(from.point,to.point,shapePoints[j],shapePoints[k],&rp.point) > 0.){
				rp.fScore = from.point.distance2(rp.point);
				hitList << rp;
			}
		}
	}
	
	if(hitList.isEmpty()) return NULL; // no hits all clear
	
	hitList = this->quickSortFScoreLessthan(hitList); // sort all hit points from closest to farthest
	
	for(int i=0;i<hitList.size();i++){
		if(hitList[i].fScore > 0.0){
			if(point) *point = hitList[i].point;
			return hitList[i].object;
		}
	}
	return NULL;
}

// gets the extreme vertecies of the collision object with reference to the midPoint 
void pathPlan::getExtremes(btCollisionObject* obj, rankPoint pivotPoint, rankPoint* left, rankPoint* right)
{
	float leftMax = 0;
	float rightMax = 0;
	bool setFirst = true;
	btVector3 firstPoint;
	QList<btVector3> ptList;
	
	btTransform objTrans = obj->getWorldTransform();		// the transform of the object being intersected
	ptList = m_CS->getTopShapePoints(obj);					// get all the top points of the obj

	for(int i = 0; i < ptList.size(); ++i)					// cycle through all top points
	{
		if(pivotPoint.object == obj && pivotPoint.corner == i) continue;	// ignore the point if it is equal to the pivot point
		
		btVector3 vert = ptList[i]; 
		//vert.setZ(objTrans.getOrigin().z());								// flatten vectors to 2D
		vert.setZ(m_startPoint.point.z());									// set all extremes to start height
		
		if( setFirst ){ 
			firstPoint = vert; 
			(*left).point = vert;
			(*left).corner = i;
			(*left).object = obj;
			(*right).point = vert;
			(*right).corner = i;
			(*right).object = obj;
			setFirst = false;
			continue; 
		}
		
			// get the cross product to find the direction, left or right side of first point
		btVector3 xc = (firstPoint - pivotPoint.point).cross(vert - pivotPoint.point);
			// get the angle to find the extremes
		float angle = (firstPoint - pivotPoint.point).angle(vert - pivotPoint.point);

			// if cross product between obj center and vertex is + or UP then left point
		if(xc.z() > 0){
			if(angle > leftMax){
				leftMax = angle;
				(*left).point = vert;
				(*left).corner = i;
				(*left).object = obj;
			}
		}
			// else if - or DOWN then right point
		else{
			if(angle > rightMax){
				rightMax = angle;
				(*right).point = vert;
				(*right).corner = i;
				(*right).object = obj;
			}
		}
	}
}

// returns a list of all the visable extremes of C Space obstacles
QList<rankPoint> pathPlan::getVisablePointsFrom(rankPoint here)
{
	int i;
	rankPoint leftMost;
	rankPoint rightMost;
	QList<rankPoint> list;
	QList<btCollisionObject*>* ghostList = m_CS->getGhostList();

// gather all objects extreme vertices
	for(i = 0; i < ghostList->size(); ++i)										// get all points from individual objects
	{		
		bool lState,rState;
		lState = true;
		rState = true;
		this->getExtremes(ghostList->at(i),here,&leftMost,&rightMost);			// get the far left and right points around the obstacle
		
		if(ghostList->at(i)->getUserPointer())									// check both extremes to make sure they are not inside of a grouped object
		{
			cSpace::overlapGroup* gp = static_cast<cSpace::overlapGroup*>(ghostList->at(i)->getUserPointer());
			for(int j=0;j<gp->list.size();j++)
			{
				if( gp->list[j] == ghostList->at(i) ) continue;					// skip the object the extremes are from
				if( lState && m_CS->isPointInsideObject(leftMost.point,gp->list[j]) )	lState = false;
				if( rState && m_CS->isPointInsideObject(rightMost.point,gp->list[j]) )	rState = false;
				if( !lState && !rState ) break;
			}
		}
		
		// check for spin condition past obstacles and eliminate paths to the left or right depending on spin	
		if(lState) list << leftMost;					// keep all left extremes
		if(rState) list << rightMost;					// keep all right extremes
	}

	i=0;
	while(i < list.size()){
		if(this->isRayBlocked(here,list[i])) 							// remove all remaining points that are blocked
			list.removeAt(i);
		else
			i++;
	}
	return list;
}

// does a quick sort on the list, orders from lowest to highest fScore
QList<rankPoint> pathPlan::quickSortFScoreLessthan(QList<rankPoint> list)
{
	QList<rankPoint> less;
	QList<rankPoint> greater;
	
	if(list.size() <= 1) return list;
	rankPoint pivot = list.takeLast();
	for(int i = 0; i < list.size(); ++i)
	{
		if(list[i].fScore < pivot.fScore) less << list[i];
		else greater << list[i];
	}
	return (quickSortFScoreLessthan(less) << pivot << quickSortFScoreLessthan(greater));
}


/////////////////////////////////////////
// path display access functions
/////////////
void pathPlan::displayDebug(bool x)
{
	m_displayDebug = x;
	if(x) m_displayList.push_front( &m_drawingList[PP_DEBUG] );
	else m_displayList.removeAll( &m_drawingList[PP_DEBUG] );
	if(m_range) displayRangeFan(x);	
}
void pathPlan::displayCrowFly(bool x)
{
	m_displayCrowFly = x;
	if(x) m_displayList.push_front(&m_drawingList[PP_CROWFLY] );	// insert after the baseline of the path has been drawn
	else m_displayList.removeAll(&m_drawingList[PP_CROWFLY]);
}
void pathPlan::displayCurrentSearch(bool x)
{
	if(x) m_displayList.push_back(&m_drawingList[PP_CURRENTSEARCH]);
	else m_displayList.removeAll(&m_drawingList[PP_CURRENTSEARCH]);
}
void pathPlan::displayRangeFan(bool x)
{
	if(x) m_displayList.push_back(&m_drawingList[PP_RANGEFAN]);
	else m_displayList.removeAll(&m_drawingList[PP_RANGEFAN]);
}
void pathPlan::displayPath(bool x)
{
	m_displayPath = x;
	if(x) m_displayList.push_front( &m_drawingList[PP_BASELINE] );	// make sure the base line of the path is drawn first
	else m_displayList.removeAll(&m_drawingList[PP_BASELINE]);
}
void pathPlan::displayLightTrail(bool x)
{
	m_displayLightTrail = x;
	if(x) m_displayList.push_back( &m_drawingList[PP_LIGHTTRAIL] );	// insert after the baseline of the path has been drawn
	else m_displayList.removeAll(&m_drawingList[PP_LIGHTTRAIL]);
}
void pathPlan::displayCspace(bool x)
{
	m_displayCS = x;
	if(m_CS) m_CS->drawCspace(m_displayCS);
}
void pathPlan::displayBuildPath(bool x)
{
	if(x) m_displayList.push_front( &m_drawingList[PP_BUILDPATH] );
	else m_displayList.removeAll( &m_drawingList[PP_BUILDPATH] );
}

/////////////////////////////////////////
// Draw the path's
/////////////
void pathPlan::drawDebugPath()				// draws test points and lines
{
	int i;
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glNormal3f(1,1,1);
	glColor3f(0,1,0);
	for(i = 1; i < contactPoints.size(); ++i)
	{
		glVertex3fv(contactPoints[0].point.m_floats);
		glVertex3fv(contactPoints[i].point.m_floats);
	}
	glEnd();

	glPointSize(6.0);
	glBegin(GL_POINTS);
	glColor3f(0,0,1);
	for(i=0;i<hitPoints.size();i++)
		glVertex3fv(hitPoints[i].m_floats);
	glEnd();
}

void pathPlan::drawCrowFlyLine()			// draws a dark blue line from the start of the path to the goal, used for visual cues and user evaluation
{
	glBegin(GL_LINES);
	glLineWidth(1.0);
	glColor3f(0,0,1);
	glNormal3f(0,0,1);
	glVertex3fv(m_startPoint.point.m_floats);
	glVertex3fv(m_goalPoint.point.m_floats);
	glEnd();
}

void pathPlan::drawCurrentSearchPath()		// draws a yellow line indicating the current search path while looping
{
	// draws the yellow line on the base of the current search path
	glLineWidth(4.0);
	glBegin(GL_LINE_STRIP);
	glColor3f(1,1,0);
	glNormal3f(0,0,1);
	for(int i = 0; i < m_pointPath.size(); ++i)
	{
		glVertex3fv(m_pointPath[i].point.m_floats);
	}
	glEnd();
}

void pathPlan::drawRangeFan()
{
	btVector3 cc = m_CS->getCenterPoint() + btVector3(0,0,0.1);
	radarFan(cc.m_floats,m_range);
}

void pathPlan::drawPathBuildLine()
{
	int i;
	static bool x = true;
	if(x)glLineStipple(1,0xFF);
	else glLineStipple(1,0xFF00);
	x = !x;
	glLineWidth(6.0);
	glNormal3f(0,0,1);
	glColor3f(m_color.redF(),m_color.greenF(),m_color.blueF());
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINE_STRIP);										// draw the dotted line path
	for(i = 0; i < m_trailPath.size(); i++)
		glVertex3fv(m_trailPath[i].point.m_floats);
	for(i = 0; i < m_GP.points.size(); i++)
		glVertex3fv(m_GP.points[i].point.m_floats);
	glEnd();
	glDisable(GL_LINE_STIPPLE);
	
	glPointSize(8.0);
	glNormal3f(0,0,1);
	if(x)glColor3f(0,0,0);
	else glColor3f(1,1,1);
	glBegin(GL_POINTS);
	for(i = 0; i < m_minimaList.size(); i++)
		glVertex3fv(m_minimaList[i].point.m_floats);
	glEnd();
}

void pathPlan::drawPathBaseLine()			// draws the line on the base of the shortest path in m_color color
{
	glLineWidth(1.0);
	glNormal3f(0,0,1);
	glColor3f(m_color.redF(),m_color.greenF(),m_color.blueF());
	glBegin(GL_LINE_STRIP);
	for(int i = 0; i < m_GP.points.size(); i++)
	{
		glVertex3fv(m_GP.points[i].point.m_floats);
	}
	glEnd();
}

void pathPlan::drawLightTrail()				// draws the path with a verticle haze
{
 	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_QUAD_STRIP);
	glNormal3f(0,0,1);
	for(int i = 0; i < m_GP.points.size(); ++i)
	{	
		btVector3 t = m_GP.points[i].point;
		glColor4f(m_color.redF(),m_color.greenF(),m_color.blueF(),m_color.alphaF());
		glVertex3f(t.x(),t.y(),t.z());
		glColor4f(0,0,0,0);
		glVertex3f(t.x(),t.y(),t.z()+1.5);
	}
	glEnd();
	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);
}

void pathPlan::renderGLObject()
{
	for(int i = 0; i < m_displayList.size(); ++i)
	{
		m_displayList[i]->Execute();
	}
}