#ifndef WAYPOINTTOOL_H
#define WAYPOINTTOOL_H

#include <QtGui>
#include "ui_waypointtool.h"
#include "../utility/definitions.h"

class waypointTool : public QWidget, private Ui::waypointtool 
{
	Q_OBJECT
	public:
		waypointTool(QWidget *parent = 0);
		~waypointTool();
		void updateComboWaypointList();

	public slots:
		void on_buttonAdd_clicked();
		void on_buttonDelete_clicked();
		void on_comboWpSelect_activated(int index);
		void on_comboScience_activated(int index);
		void raiseWaypointEditor(QList<WayPoint>* list);
		void edited();

	signals:
		void addedWP();

	private:
		int					cIndex;
		QList<WayPoint>		*WPlist;
		QMap<int, QString>	WSmap;

		void waypointScienceKeyMapping();
		void setComboScienceList();
};

#endif //WAYPOINTTOOL_H