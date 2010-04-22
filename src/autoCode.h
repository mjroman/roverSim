#ifndef AUTOCODE_H
#define AUTOCODE_H

#include <QtGui>
#include "ui_autoCode.h"
#include "sr2rover.h"
#include "laserscanner.h"
#include "utility/definitions.h"

class autoCode : public QWidget, private Ui::autoCode
{
	Q_OBJECT
	public:
		autoCode(SR2rover *bot, QWidget *parent = 0);
		~autoCode();
		void goAutonomous();
		void stopAutonomous(RoverState rs);
		void toggleAutonomous();
		int getCurrentWaypoint() { return wpIndex; }

	public slots:
		void moveToWaypoint();
		void on_combo_wpSelect_activated(int index);
		void displayCurrentWaypoint();
		void quickObstacleCheck();
			
	protected:
		float	POINTTURNSPEED;
		float	POINTTURNANGLE;
		float	TURNMULTIPLIER;	
		float	CLOSEENOUGH;
		float 	TURNACCURACYLIMIT;
		int		ROVERCRUISESPEED;
		float 	BODYDIST;	
		float	PROFILEOBSTACLEMAX;	
		float 	MAXPITCH;
		float   MAXROLL;
		float   PITCHDOWNIGNOREDISTOBSTACLES;
		float   BODYTOOCLOSE;
		float 	PANELOBSTACLEMAX;
		float   TURNFACTOR;
		float 	GOPASTDISTANCE;
		float	PATHEFFICIENCY;

	private:
		SR2rover 			*sr2;
		bool				autoRunning;
		RoverState 			state;
		RoverError			error;
		int					wpIndex;
		WayPoint 			currentWaypoint;
		bool				currentWaypointDisplay;
		QMap<int, QString>	RSmap;
		QMap<int, QString>	REmap;
		QMap<int, QString>  WPmap;
		QMap<int, QString>	WSmap;
		float				expectedDistance;
		float				blockedDirection;
		btVector3			lastBlockedPosition;
		int					lastBlockedDirection;

		void callForHelp(RoverError errCode);
		float compassToCartRadians(float rad);
		float avgAngle(float rad1, float rad2);
		float compassWaypointHeading();
		float roverWaypointHeading();
		float distanceToWaypoint(int i);
		void driveToward(float relHead, float distTo);
		void getPanelHeights(float* heights);
		bool checkForObstacles(float distTo);
		void avoidingTurn();
		void roverStateKeyMapping();
		void roverErrorKeyMapping();
		void waypointStateKeyMapping();
		void waypointScienceKeyMapping();
		void setComboWaypointList();
		void updateGUI();
};

#endif //AUTOCODE_H