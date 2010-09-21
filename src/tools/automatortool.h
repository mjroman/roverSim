#ifndef AUTOMATORTOOL_H
#define AUTOMATORTOOL_H

#include <QtGui>
#include "ui_automatortool.h"
#include <QSettings>

class automatorTool : public QDialog , private Ui::automatortool
{
	Q_OBJECT
	
	public:
		automatorTool(QWidget *parent = 0);
	
	public slots:
		void setTerrainFile();
		void saveData();
		void runData();
		
	private:
		QSettings		configFile;
		QFileInfo		terrainFileInfo;
		
		void acceptData();
		
};
#endif // AUTOMATORTOOL_H