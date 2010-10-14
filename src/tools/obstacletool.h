#ifndef OBSTACLETOOL_H
#define OBSTACLETOOL_H

#include <QtGui>
#include <QSettings>
#include "ui_obstacletool.h"
#include "utility/structures.h"

class obstacleTool : public QWidget, private Ui::obstacleTool 
{
	Q_OBJECT

	public:
		obstacleTool(QWidget *parent = 0);
		~obstacleTool();

		void setObstacleCount(int num){
			obstCount.stuff.setValue(num);
			SpinBoxObstCount->setValue(num);
		}
		
		void setCount(int c);
		void setMinSize(btVector3 m);
		void setMaxSize(btVector3 m);
		void setYawRange(float min,float max);
		
		int obstacleCount(){ return obstCount.stuff.toInt(); }
		int obstacleType(){ return obstShape.stuff.toInt(); }
		float minOLength(){ return obstMinLength.stuff.toFloat(); }
		float minOWidth(){ return obstMinWidth.stuff.toFloat(); }
		float minOHeight(){ return obstMinHeight.stuff.toFloat(); }
		float maxOLength(){ return obstMaxLength.stuff.toFloat(); }
		float maxOWidth(){ return obstMaxWidth.stuff.toFloat(); }
		float maxOHeight(){ return obstMaxHeight.stuff.toFloat(); }
		float minOYaw(){ return obstMinYaw.stuff.toFloat(); }
		float maxOYaw(){ return obstMaxYaw.stuff.toFloat(); }
		float density(){ return obstDensity.stuff.toFloat(); }
		float dropHeight(){ return obstDropHeight.stuff.toFloat(); }

	public slots:
		void show();
		void on_ButtonGenerate_clicked(bool checked=false);
		void updateLabels(int index);

	signals:
		void regenerateObstacles(int);
		
	protected:
		settingParam	obstShape;
		settingParam	obstCount;
		settingParam	obstMinLength,obstMinWidth,obstMinHeight;
		settingParam	obstMaxLength,obstMaxWidth,obstMaxHeight;
		settingParam	obstMinYaw,obstMaxYaw;
		settingParam	obstDensity;
		settingParam	obstDropHeight;
		
	private:
		QSettings 			obstSettings;
		
		void initSettingsNames();
		void defaultSettings();
		void getSettings();
		void setSettings();
};

#endif // OBSTACLETOOL_H
