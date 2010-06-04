#ifndef OBSTACLETOOL_H
#define OBSTACLETOOL_H

#include <QtGui>
#include <QSettings>
#include "ui_obstacletool.h"

typedef struct _obstParam {
	QString name;
	QVariant stuff;
}obstParam;

class obstacleTool : public QWidget, private Ui::obstacleTool {
	Q_OBJECT
	private:
		QSettings 			obstSettings;
		
		void initSettingsNames();
		void initSettings();
		void getSettings();
		void setSettings();
		
	protected:
		obstParam	obstShape;
		obstParam	obstCount;
		obstParam	obstMinLength,obstMinWidth,obstMinHeight;
		obstParam	obstMaxLength,obstMaxWidth,obstMaxHeight;
		obstParam	obstMinYaw,obstMaxYaw;
		obstParam	obstDensity;
		obstParam	obstDropHeight;
		
	public:
		obstacleTool(QWidget *parent = 0);
		~obstacleTool();

		void setObstacleCount(int num){
			obstCount.stuff.setValue(num);
			SpinBoxObstCount->setValue(num);
		}
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
		void raise();
		void on_ButtonGenerate_clicked(bool checked=false);
		void on_ButtonCancel_clicked();
		void updateLabels(int index);

	signals:
		void regenerateObstacles();
};

#endif // OBSTACLETOOL_H
