#include "pathtool.h"
#include "../robot.h"

pathTool::pathTool(robot *bot, simGLView* glView, QWidget* parent)
:
QWidget(parent),
rover(bot),
view(glView),
selectedPath(0)
{
	setupUi(this);
	move(20,400);
	setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Path Creation");
	
	tableSetup();
	show();
}

pathTool::~pathTool()
{
	qDebug("deleting pathTool");
	removePaths();
	pathPrototypeList.clear();
}

void pathTool::removePaths()
{
	if(pathList.isEmpty()) return;
	int i;
	for(i=0; i<pathList.size(); i++) delete pathList[i];
	pathList.clear();
}

void pathTool::tableSetup()
{
	pathTableWidget->setColumnCount(4);
	pathTableWidget->setRowCount(0);
	pathTableWidget->showGrid();
	pathTableWidget->setAlternatingRowColors(false);
	pathTableWidget->verticalHeader()->setResizeMode(QHeaderView::Fixed);
	pathTableWidget->horizontalHeader()->setResizeMode(QHeaderView::Fixed);

	QTableWidgetItem* item = new QTableWidgetItem(" ");
	pathTableWidget->setHorizontalHeaderItem(0,item);		// color
	pathTableWidget->setColumnWidth(0,28);
	item = new QTableWidgetItem("Range");
	pathTableWidget->setHorizontalHeaderItem(1,item);		// range
	pathTableWidget->setColumnWidth(1,45);
	item = new QTableWidgetItem("Length");
	pathTableWidget->setHorizontalHeaderItem(2,item);		// length
	pathTableWidget->setColumnWidth(2,80);
	item = new QTableWidgetItem("Time (s)");
	pathTableWidget->setHorizontalHeaderItem(3,item);		// time
	pathTableWidget->setColumnWidth(3,100);

	connect(pathTableWidget, SIGNAL(cellClicked(int,int)),this,SLOT(tableDataChange(int,int)));
	connect(pathTableWidget, SIGNAL(cellDoubleClicked(int,int)),this,SLOT(tableDataEdit(int,int)));
}

void pathTool::on_buttonAdd_clicked()
{
	if(pathPrototypeList.isEmpty()) buttonDelete->setEnabled(true);
	
	// execute a dialog here to create a new path
	
	goalPath newPath;
	newPath.length = 0;
	newPath.color = btVector3(0,1,0);
	newPath.color.m_floats[3] = 0.45;
	newPath.range = 5;
	newPath.step = 0.25;
	newPath.breadth = 0;
	newPath.saveOn = false;
	
	pathPrototypeList.insert(selectedPath, newPath);
	
	pathTableWidget->insertRow(selectedPath);
	QColor clr;
	clr.setRgbF(newPath.color.x(),newPath.color.y(),newPath.color.z(),newPath.color.w());
	QTableWidgetItem* color = new QTableWidgetItem();
	color->setData(Qt::DecorationRole,clr);
	color->setFlags(Qt::ItemIsEnabled);
	pathTableWidget->setItem(selectedPath,0,color);
	
	QTableWidgetItem* range = new QTableWidgetItem();
	range->setData(Qt::DisplayRole,newPath.range);
	QFont ft = range->font();
	ft.setPointSize(12);
	range->setFont(ft);
	range->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	//range->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selectedPath,1,range);
	
	QTableWidgetItem* length = new QTableWidgetItem("...");
	length->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	length->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selectedPath,2,length);
	
	QTableWidgetItem* stint = new QTableWidgetItem("...");
	stint->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	stint->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(selectedPath,3,stint);
}
void pathTool::on_buttonDelete_clicked()
{
	if(selectedPath >= pathPrototypeList.size()) selectedPath = pathPrototypeList.size()-1;
	pathPrototypeList.removeAt(selectedPath);
	
	if(!pathList.isEmpty()){
		delete pathList[selectedPath];
		pathList.removeAt(selectedPath);
	}
	if(pathPrototypeList.isEmpty()) buttonDelete->setEnabled(false);
	pathTableWidget->removeRow(selectedPath);
}
void pathTool::on_buttonGenerate_clicked()
{
	int i;
	pathPlan *path;
	
	for(i=0; i<pathList.size(); i++) delete pathList[i];		// delete all old paths before creating new ones
	pathList.clear();
	
	for(i=0; i<pathPrototypeList.size(); i++)
	{
		path = new pathPlan(rover->position,goalPoint,pathPrototypeList[i],view);	// create new paths based on prototype list
		pathList << path;
		updateTool();
	}
}

// update the data in the table, this is called after the paths are generated
void pathTool::updateTool()
{
	int i,j;
	for(i=0; i<pathList.size(); i++){
		goalPath* gp = pathList[i]->getShortestPath();
		for(j=0; j<pathTableWidget->columnCount(); j++){
			QTableWidgetItem* item = pathTableWidget->item(i,j);
			switch(j){
				case 0:
				{
					QColor clr;
					clr.setRgbF(gp->color.x(),gp->color.y(),gp->color.z(),gp->color.w());
					item->setData(Qt::DecorationRole,clr);
					break;
				}
				case 1:
				{
					item->setData(Qt::DisplayRole,gp->range);
					break;
				}
				case 2:
				{
					item->setData(Qt::DisplayRole,gp->length);
					break;
				}
				case 3:
				{
					item->setData(Qt::DisplayRole,QVariant(gp->time/1000.));
					break;
				}
				default:
				break;
			}
		}
	}
}

void pathTool::tableDataChange(int row, int column)
{
	selectedPath = row;
}

void pathTool::tableDataEdit(int row, int column)
{
	selectedPath = row;
	
	// bring up editing dialog box
}