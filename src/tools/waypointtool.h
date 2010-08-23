#ifndef WAYPOINTTOOL_H
#define WAYPOINTTOOL_H

#include <QtGui>
#include "ui_waypointtool.h"
#include "../utility/structures.h"

class waypointTool : public QWidget, private Ui::waypointtool 
{
	Q_OBJECT
	public:
		waypointTool(QWidget *parent = 0);
		~waypointTool();
		void updateComboWaypointList();

	public slots:
		void show();
		void on_buttonAdd_clicked();
		void on_buttonDelete_clicked();
		void on_comboWpSelect_activated(int index);
		void on_comboScience_activated(int index);
		void raiseWaypointEditor(QList<WayPoint>* list);
		void edited();
		void resetStates();

	signals:
		void addedWP(WayPoint wp,int index);
		void editedWP(int index);
		void resetWP();

	private:
		int					cIndex;
		bool				currentWaypointDisplay;
		QList<WayPoint>		*WPlist;
		
		void waypointScienceKeyMapping();
		void waypointStateKeyMapping();
		void setComboScienceList();
		QMap<int, QString>  WPmap;	// waypoint state map
		QMap<int, QString>	WSmap;	// waypoint science map
};

#endif //WAYPOINTTOOL_H