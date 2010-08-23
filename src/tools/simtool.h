#ifndef SIMTOOL_H
#define SIMTOOL_H

#include <QtGui>

class simtool : public QDialog
{
    Q_OBJECT

private:
	QPushButton		*cancelButton;
    QPushButton 	*updateButton;

    QLineEdit   	*timeStepLine;
    QLineEdit   	*fixedStepLine;
    QLineEdit   	*subStepLine;

    QLabel      	*timeStepLabel;
    QLabel      	*fixedStepLabel;
    QLabel      	*subStepLabel;

public:
	float			step;
	float			fixedStep;
	int				subStep;
    simtool(QWidget *parent = 0);

public slots:
	void acceptData();
};

#endif // SIMTOOL_H
