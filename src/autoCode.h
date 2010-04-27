#ifndef AUTOCODE_H
#define AUTOCODE_H

#include <QtGui>
#include <QSettings>
#include "ui_navigationtool.h"
#include "sr2rover.h"
#include "laserscanner.h"
#include "utility/definitions.h"

typedef struct _navParam {
	QString name;
	QVariant stuff;
}navParam;

class autoCode : public QWidget, private Ui::navigationtool
{
	Q_OBJECT
	public:
		autoCode(SR2rover *bot,QList<WayPoint> *list, QWidget* parent = 0);
		~autoCode();
		int getCurrentWaypoint() { return wpIndex; }
		
	public slots:
		void goAutonomous();
		void stopAutonomous(RoverState rs);
		void moveToWaypoint();
		void quickObstacleCheck();
		void on_buttonRunning_clicked(bool checked = false);
		void updateTool();
		void tableDataChange(int row, int column);
			
	protected:
		navParam 	MAXPITCH;
		navParam  	MAXROLL;
		navParam   	ROVERCRUISESPEED;
		navParam	POINTTURNSPEED;
		navParam	POINTTURNANGLE;
		navParam 	TURNACCURACYLIMIT;
		navParam	TURNMULTIPLIER;	
		navParam	TURNFACTOR;
		navParam 	GOPASTDISTANCE;
		navParam	CLOSEENOUGH;
		navParam 	BODYDIST;	
		navParam	BODYTOOCLOSE;
		navParam 	PANELOBSTACLEMAX;
		navParam	PROFILEOBSTACLEMAX;	
		navParam	PITCHDOWNIGNOREDISTOBSTACLES;
		navParam	PATHEFFICIENCY;

	private:
		SR2rover 			*sr2;
		QList<navParam*>	Plist;
		bool				running;
		RoverState 			state;
		RoverError			error;
		QList<WayPoint>		*WPlist;
		int					wpIndex;
		WayPoint 			currentWaypoint;
		float				wpRange;
		float				expectedDistance;
		float				blockedDirection;
		btVector3			lastBlockedPosition;
		int					lastBlockedDirection;
		QSettings 			simSettings;

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
		
		QMap<int, QString>	RSmap;	// rover state map
		QMap<int, QString>	REmap;	// rover error map

		void roverStateKeyMapping();
		void roverErrorKeyMapping();
		void parameterListInit();
		void tableSetup();
		void initSettingsNames();
		void initSettings();
};

#endif //AUTOCODE_H