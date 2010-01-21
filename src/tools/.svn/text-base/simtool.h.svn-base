#ifndef SIMTOOL_H
#define SIMTOOL_H

#include <QWidget>

QT_BEGIN_NAMESPACE
class QPushButton;
class QLineEdit;
class QLabel;
QT_END_NAMESPACE

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

    float getTimeStep();
    float getFixedTimeStep();
    int getSubSteps();

public slots:
    void closeEvent(QCloseEvent *event);

signals:
    void paramUpdate();

};

#endif // SIMTOOL_H
