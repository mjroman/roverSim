#ifndef PATHTOOL_H
#define PATHTOOL_H

#include <QtGui>
#include "ui_pathtool.h"
#include "../pathPlan.h"
#include "../utility/structures.h"

class simGLView;
class robot;

class pathTool : public QWidget, private Ui::pathtool
{
	Q_OBJECT
	public:
		pathTool(robot *bot, simGLView* glView = NULL, QWidget* parent = 0);
		~pathTool();
		
		void tableSetup();
		void setGoalPoint(btVector3 goal) { goalPoint = goal; }
		
	public slots:
		void removePaths();
		void on_buttonAdd_clicked();
		void on_buttonDelete_clicked();
		void on_buttonGenerate_clicked();
		void updateTool();
		void tableDataChange(int row, int column);
		void tableDataEdit(int row, int column);
		
	private:
		robot				*rover;
		simGLView			*view;
		QList<goalPath>		pathPrototypeList;
		QList<pathPlan*>	pathList;
		btVector3			goalPoint;
		int					selectedPath;
};
#endif //PATHTOOL_H