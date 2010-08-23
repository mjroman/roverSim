#ifndef SIMTOOL_H
#define SIMTOOL_H

#include <QWidget>

class QPushButton;
class QLineEdit;
class QLabel;

class simtool : public QWidget
{
    Q_OBJECT

public:
    QPushButton *updateButton;
    QLineEdit   *timeStepLine;
    QLineEdit   *fixedStepLine;
    QLineEdit   *subStepLine;
    QLabel      *timeStepLabel;
    QLabel      *fixedStepLabel;
    QLabel      *subStepLabel;

    simtool(QWidget *parent = 0);
	~simtool();
	
    float getTimeStep();
    float getFixedTimeStep();
    int getSubSteps();

public slots:
    void closeEvent(QCloseEvent *event);

signals:
    void paramUpdate();

};

#endif // SIMTOOL_H
