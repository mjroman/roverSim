#ifndef PATHTOOL_H
#define PATHTOOL_H

#include <QtGui>
#include <QComboBox>
#include <QDomNode>
#include <QTextStream>
#include "ui_pathtool.h"
#include "../utility/structures.h"

class simGLView;
class robot;
class obstacles;
class pathPlan;

class ColorListEditor : public QComboBox
{
	Q_OBJECT
	Q_PROPERTY(QColor color READ color WRITE setColor USER true)

	public:
		ColorListEditor(QWidget *widget = 0);

	public:
		QColor color() const;
		void setColor(QColor c);

	private:
		void populateList();
};

class pathEditDialog : public QDialog
{
	Q_OBJECT
	
	public:
		pathEditDialog(pathPlan *ph, QWidget *parent = 0);
		
		ColorListEditor *colorComboBox;
		QLineEdit 		*rangeLineEdit;
		QLineEdit		*growLineEdit;
		QLineEdit 		*stepLineEdit;
		QLineEdit		*efficiencyLineEdit;
		QLineEdit		*spinLineEdit;
		QComboBox		*spinBaseBox;
		QLineEdit 		*breadthLineEdit;
		QCheckBox 		*saveAllCheckBox;
		
		QGroupBox		*displayGroupBox;
		QCheckBox 		*baselineCheckBox;
		QCheckBox 		*lightTrailCheckBox;
		QCheckBox 		*crowFlyCheckBox;
		QCheckBox 		*saveDisplayCheckBox;
		QCheckBox		*cspaceDisplayCheckBox;
		
		QPushButton		*doneButton;
		QPushButton		*cancelButton;
		
	private:
		pathPlan		*path;
		QLabel 			*colorLabel;
		QLabel 			*rangeLabel;
		QLabel			*growLabel;
		QLabel 			*stepLabel;
		QLabel			*efficiencyLabel;
		QLabel			*spinLabel;
		QLabel 			*breadthLabel;
	
	public slots:
		void enableLines();
		void stepWarning();	
		void acceptData();
};

class pathTool : public QWidget, private Ui::pathtool
{
	Q_OBJECT
	public:
		pathTool(robot *bot, obstacles *obs, simGLView* glView = NULL);
		~pathTool();
		
		void tableSetup();
		void setGoalPoint(btVector3 goal) { goalPoint = goal; }
		void addToTable(pathPlan *path);
		void addPath(float range, float step, float csSize, float effLimit, float spinProgress);
		
	public slots:
		void show();
		void removePaths();
		void resetPaths();
		void on_buttonAdd_clicked();
		void on_buttonDelete_clicked();
		void on_buttonGenerate_clicked();
		void on_buttonGenAll_clicked();
		void processPath(int x);
		void updateTool();
		void updateCompEfficiency(float gLength);
		void setRowBackground(int row, QBrush stroke);
		void tableDataChange(int row, int column);
		void tableDataEdit(int row, int column);
		void stepOnPath(int dir);
		
	signals:
		void changeBackground(int,QBrush);
		void computePath(int);
	
	private:
		robot				*rover;
		obstacles			*blocks;
		simGLView			*view;
		QList<pathPlan*>	pathList;
		btVector3			goalPoint;
		int					m_selectedPath;
		bool				m_allPaths;
		QSound				m_foundSound;
		int					m_runCount;
		QString				m_filename;
		QFile				*m_file;
		QDomDocument		m_xmlDoc;
		QTextStream			m_xmlStream;
		
		bool initSaveFile();
};
#endif //PATHTOOL_H