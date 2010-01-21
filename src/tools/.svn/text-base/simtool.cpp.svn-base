#include <QtGui>

#include "simtool.h"
#include "physicsWorld.h"

simtool::simtool(QWidget *parent) : QWidget(parent)
{
    QWidget::setWindowFlags(Qt::Sheet);

    updateButton = new QPushButton(tr("&Update"));
    updateButton->setStyleSheet(QString("background:rgb(255, 0, 0); color:white; border:2px solid rgb(169, 2, 6); border-radius:10px;"));

    connect(updateButton, SIGNAL(clicked()), this, SLOT(close()));

    timeStepLine = new QLineEdit;
    fixedStepLine = new QLineEdit;
    subStepLine = new QLineEdit;

    physicsWorld *arena = physicsWorld::instance(); // get the physics world object

    timeStepLine->setText(QString::number(arena->simTimeStep));
    timeStepLine->setFixedWidth(60);
    fixedStepLine->setText(QString::number(arena->simFixedTimeStep));
    fixedStepLine->setFixedWidth(60);
    subStepLine->setText(QString::number(arena->simSubSteps));
    subStepLine->setFixedWidth(60);

    timeStepLabel = new QLabel(this);
    timeStepLabel->setText(QString("Time step:"));

    fixedStepLabel = new QLabel(this);
    fixedStepLabel->setText(QString("Fixed step:"));

    subStepLabel = new QLabel(this);
    subStepLabel->setText(QString("Sub step:"));

    QHBoxLayout *layoutH = new QHBoxLayout;
    layoutH->addWidget(timeStepLabel);
    layoutH->addWidget(timeStepLine);
    layoutH->addWidget(fixedStepLabel);
    layoutH->addWidget(fixedStepLine);
    layoutH->addWidget(subStepLabel);
    layoutH->addWidget(subStepLine);
    QVBoxLayout *layoutV = new QVBoxLayout;
    layoutV->addLayout(layoutH);
    layoutV->addWidget(updateButton);
    setLayout(layoutV);
}

void simtool::closeEvent(QCloseEvent *event)
{
    emit paramUpdate();
    event->accept();
}

float simtool::getTimeStep()
{
    return timeStepLine->text().toFloat();
}

float simtool::getFixedTimeStep()
{
    return fixedStepLine->text().toFloat();
}

int simtool::getSubSteps()
{
    return subStepLine->text().toInt();
}
