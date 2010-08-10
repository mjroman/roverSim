#ifndef PATHTOOL_H
#define PATHTOOL_H

#include <QtGui>
#include <QComboBox>
#include "ui_pathtool.h"
#include "../pathPlan.h"
#include "../utility/structures.h"

class simGLView;
class robot;

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
		QLineEdit 		*stepLineEdit;
		QLineEdit 		*breadthLineEdit;
		QCheckBox 		*saveAllCheckBox;
		
		QGroupBox		*displayGroupBox;
		QCheckBox 		*baselineCheckBox;
		QCheckBox 		*lightTrailCheckBox;
		QCheckBox 		*crowFlyCheckBox;
		QCheckBox 		*saveDisplayCheckBox;
		
		QDialogButtonBox *buttonBox;
		QPushButton		*doneButton;
		QPushButton		*cancelButton;
		
	private:
		pathPlan		*path;
		QLabel 			*colorLabel;
		QLabel 			*rangeLabel;
		QLabel 			*stepLabel;
		QLabel 			*breadthLabel;
	
	public slots:	
		void acceptData();
};

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
		void resetPaths();
		void on_buttonAdd_clicked();
		void on_buttonDelete_clicked();
		void on_buttonGenerate_clicked();
		void updateTool();
		void tableDataChange(int row, int column);
		void tableDataEdit(int row, int column);
		
	private:
		robot				*rover;
		simGLView			*view;
		QList<pathPlan*>	pathList;
		btVector3			goalPoint;
		int					selectedPath;
};
#endif //PATHTOOL_H