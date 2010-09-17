#ifndef WAYPOINTTOOL_H
#define WAYPOINTTOOL_H

#include <QtGui>
#include "ui_waypointtool.h"
#include "../utility/structures.h"

class terrain;

class waypointTool : public QWidget, private Ui::waypointtool 
{
	Q_OBJECT
	public:
		waypointTool(terrain *t,QWidget *parent = 0);
		~waypointTool();
		
		QList<WayPoint>* getList() { return &wayList; }
		void addWaypoint(WayPoint);
		
	public slots:
		void show();
		void resetStates();
		void on_buttonSetCurrent_clicked();
		void refreshGUI(int index);
		void on_buttonAdd_clicked();
		void on_buttonDelete_clicked();
		void uuidEdited();
		void positionEdited();
		void on_comboScience_activated(int s);
		void on_comboState_activated(int s);
		void setHeights();
	signals:
		void currentWaypoint(int);

	private:
		terrain				*ground;
		QList<WayPoint>		wayList;
		int					currentIndex;
		bool				currentWaypointDisplay;
		
		QMap<int, QString>  WPmap;	// waypoint state map
		QMap<int, QString>	WSmap;	// waypoint science map
		void waypointScienceKeyMapping();
		void waypointStateKeyMapping();
		void setComboScienceList();
		void setComboStateList();
		void enableGUI(bool state);
};

#endif //WAYPOINTTOOL_H