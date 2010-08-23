#include "pathtool.h"
#include "../robot.h"
#include "../obstacles.h"
#include "../pathPlan.h"
#include <QSound>

/////////////////////////////////////////
// custom color combo box
/////////////
ColorListEditor::ColorListEditor(QWidget *widget) 
: QComboBox(widget)
{
	populateList();
}
QColor ColorListEditor::color() const
{
	return qVariantValue<QColor>(itemData(currentIndex(), Qt::DecorationRole));
}
void ColorListEditor::setColor(QColor c)
{
	setCurrentIndex(findData(c, int(Qt::DecorationRole)));
}
void ColorListEditor::populateList()
{
	QStringList colorNames = QColor::colorNames();

	for (int i = 0; i < colorNames.size(); ++i) {
		QColor color(colorNames[i]);

		insertItem(i, colorNames[i]);
		setItemData(i, color, Qt::DecorationRole);
	}
}

/////////////////////////////////////////
// Path editing custom dialog
/////////////
pathEditDialog::pathEditDialog(pathPlan *ph, QWidget *parent)
: QDialog(parent), path(ph)
{
	setWindowTitle("Set Path Parameters");
	
	// main input layout
	QGridLayout *inputLayout = new QGridLayout;
	colorLabel = new QLabel("Path Color");
	inputLayout->addWidget(colorLabel,0,0);
	colorComboBox = new ColorListEditor;
	colorComboBox->setColor(path->getColor());
	if(colorComboBox->currentIndex() < 0){
		int z = rand() % QColor::colorNames().size();
		colorComboBox->setCurrentIndex(z);
	}
	inputLayout->addWidget(colorComboBox,0,1);
	
  	rangeLabel = new QLabel("Sensor Range");
	inputLayout->addWidget(rangeLabel,1,0);
	rangeLineEdit = new QLineEdit(this);
	rangeLineEdit->setText(QString::number(path->getRange()));
	rangeLineEdit->setAlignment(Qt::AlignHCenter);
	this->setFocusProxy(rangeLineEdit);									// sets the range line edit as the focused item when the dialog is shown
	rangeLineEdit->selectAll();
	inputLayout->addWidget(rangeLineEdit,1,1);
	
    stepLabel = new QLabel("Step Distance");
	inputLayout->addWidget(stepLabel,2,0);
	stepLineEdit = new QLineEdit(this);
	stepLineEdit->setText(QString::number(path->getStep()));
	stepLineEdit->setAlignment(Qt::AlignHCenter);
	inputLayout->addWidget(stepLineEdit,2,1);
	
	breadthLabel = new QLabel("Search Breadth");
	inputLayout->addWidget(breadthLabel,3,0);
	breadthLineEdit = new QLineEdit(this);
	breadthLineEdit->setText(QString::number(path->getBreadth()));
	breadthLineEdit->setAlignment(Qt::AlignHCenter);
	inputLayout->addWidget(breadthLineEdit,3,1);
	
	saveAllCheckBox = new QCheckBox("Save All Paths");
	saveAllCheckBox->setChecked(ph->getSaveOn());
	inputLayout->addWidget(saveAllCheckBox,4,1);
	
	// Display group box setup
	QVBoxLayout *displayLayout = new QVBoxLayout;
	displayGroupBox = new QGroupBox("Path display items");
    
	baselineCheckBox = new QCheckBox("Baseline");
	displayLayout->addWidget(baselineCheckBox);
	
	lightTrailCheckBox = new QCheckBox("Light Trail");
	displayLayout->addWidget(lightTrailCheckBox);
	
	crowFlyCheckBox = new QCheckBox("Crow Fly Line");
	displayLayout->addWidget(crowFlyCheckBox);
	
	saveDisplayCheckBox = new QCheckBox("Saved Paths");
	displayLayout->addWidget(saveDisplayCheckBox);
	
	cspaceDisplayCheckBox = new QCheckBox("Config Space");
	displayLayout->addWidget(cspaceDisplayCheckBox);
	
	displayGroupBox->setLayout(displayLayout);

	// Dialong button layout
	QVBoxLayout *buttonBox = new QVBoxLayout;
	doneButton = new QPushButton("Accept");
	doneButton->setMinimumWidth(80);
	doneButton->setStyleSheet("QPushButton:default{background: green; border: 2px solid darkgreen; border-radius:10; color: white;} QPushButton:pressed{background: red; border: 2px solid darkred; border-radius:10;}");
	doneButton->setDefault(true);
	buttonBox->addWidget(doneButton);
	
	cancelButton = new QPushButton("Cancel");
	cancelButton->setMinimumWidth(80);
	cancelButton->setStyleSheet("QPushButton:enabled{background: yellow; border: 2px solid white; border-radius:10; color: black;} QPushButton:pressed{background: white; border-color: yellow;}");
	buttonBox->addWidget(cancelButton);
	
	connect(doneButton,SIGNAL(clicked()),this,SLOT(acceptData()));
	connect(cancelButton,SIGNAL(clicked()),this,SLOT(reject()));
	
	connect(baselineCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayPath(bool)));
	connect(lightTrailCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayLightTrail(bool)));
	connect(crowFlyCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayCrowFly(bool)));
	connect(saveDisplayCheckBox,SIGNAL(clicked(bool)),path,SLOT(displaySavedPaths(bool)));
	connect(cspaceDisplayCheckBox,SIGNAL(clicked(bool)),path,SLOT(displayCspace(bool)));
	
	baselineCheckBox->setChecked(path->m_displayPath);
	lightTrailCheckBox->setChecked(path->m_displayLightTrail);
	crowFlyCheckBox->setChecked(path->m_displayCrowFly);
	saveDisplayCheckBox->setChecked(path->m_displaySavedPaths);
	cspaceDisplayCheckBox->setChecked(path->m_displayCS);
	
	// Main window layout
	QHBoxLayout *mainLayout = new QHBoxLayout;
	mainLayout->addLayout(inputLayout);
	mainLayout->addWidget(displayGroupBox);
	mainLayout->addLayout(buttonBox);
	
	this->setLayout(mainLayout);
}

void pathEditDialog::acceptData()
{
	path->setColor(colorComboBox->color());
	path->setRange(rangeLineEdit->text().toFloat());
	path->setStep(stepLineEdit->text().toFloat());
	path->setBreadth(breadthLineEdit->text().toFloat());
	path->setSaveOn(saveAllCheckBox->isChecked());
	this->accept();
}

/////////////////////////////////////////
// Path creation main tool
/////////////
pathTool::pathTool(robot *bot, obstacles *obs, simGLView* glView)
:
QWidget(glView->parentWidget()),
rover(bot),
blocks(obs),
view(glView),
m_selectedPath(0),
m_foundSound("/Users/mattroman/Documents/code/roverSim/src/sounds/singleBeep2.wav")
{
	setupUi(this);
	move(20,540);
	resize(300,150);
	setWindowFlags(Qt::Window | Qt::WindowStaysOnTopHint);
	setWindowTitle("Path Creation");
	
	buttonDelete->setEnabled(false);
	tableSetup();
	
	connect(this,SIGNAL(changeBackground(int,QBrush)),this,SLOT(setRowBackground(int,QBrush)));
	connect(this,SIGNAL(computePaths(int)),this,SLOT(processPath(int)));
	connect(pathTableWidget, SIGNAL(cellClicked(int,int)),this,SLOT(tableDataChange(int,int)));
	connect(pathTableWidget, SIGNAL(cellDoubleClicked(int,int)),this,SLOT(tableDataEdit(int,int)));	
	show();
}

pathTool::~pathTool()
{
	removePaths();
}

void pathTool::show()
{
	QPropertyAnimation *anim = new QPropertyAnimation(this,"pos");
	anim->setDuration(1000);
	anim->setStartValue(QPoint(pos().x(),pos().y()-25));
	anim->setEndValue(pos());
	anim->setEasingCurve(QEasingCurve::OutElastic);
	anim->start();
	QWidget::show();
}

void pathTool::removePaths()
{
	if(pathList.isEmpty()) return;
	int i;
	for(i=0; i<pathList.size(); i++) delete pathList[i];
	pathList.clear();
	pathTableWidget->clearContents();
}

void pathTool::resetPaths()
{
	int i;
	for(i=0; i<pathList.size(); i++){
		pathList[i]->reset();									// delete all old paths before creating new ones
		emit changeBackground(i,QBrush(QColor(Qt::white)));
	}
}

void pathTool::tableSetup()
{
	this->setUpdatesEnabled(true);
	pathTableWidget->setColumnCount(5);
	pathTableWidget->setRowCount(0);
	pathTableWidget->showGrid();
	pathTableWidget->setAlternatingRowColors(false);
	pathTableWidget->verticalHeader()->setResizeMode(QHeaderView::Fixed);
	pathTableWidget->horizontalHeader()->setResizeMode(QHeaderView::Fixed);

	QTableWidgetItem* item = new QTableWidgetItem(" ");		// color
	pathTableWidget->setHorizontalHeaderItem(0,item);		
	pathTableWidget->setColumnWidth(0,28);
	item = new QTableWidgetItem("Range");					// range
	pathTableWidget->setHorizontalHeaderItem(1,item);		
	pathTableWidget->setColumnWidth(1,45);
	item = new QTableWidgetItem("Length");					// length
	pathTableWidget->setHorizontalHeaderItem(2,item);		
	pathTableWidget->setColumnWidth(2,80);
	item = new QTableWidgetItem("Time (s)");				// time
	pathTableWidget->setHorizontalHeaderItem(3,item);		
	pathTableWidget->setColumnWidth(3,100);
	item = new QTableWidgetItem("Efficiency");				// efficiency
	pathTableWidget->setHorizontalHeaderItem(4,item);		
	pathTableWidget->setColumnWidth(4,100);

	this->setMaximumWidth(393);
}

// when the [+] button is pressed
void pathTool::on_buttonAdd_clicked()
{
	// execute a dialog here to create a new path
	pathPlan *path = new pathPlan(blocks,view);
	pathEditDialog eDialog(path,this);
	if(eDialog.exec() == QDialog::Rejected){
		delete path;
		return;
	}
	
	if(pathList.isEmpty()) buttonDelete->setEnabled(true);				// enable the delete button if the list is empty
	
	pathList.insert(m_selectedPath, path);								// add the new path to the list
	
	pathTableWidget->insertRow(m_selectedPath);							// insert a new row into the table

	QTableWidgetItem* color = new QTableWidgetItem();					// add the color item
	color->setData(Qt::DecorationRole,path->getColor());
	color->setFlags(Qt::ItemIsEnabled);
	pathTableWidget->setItem(m_selectedPath,0,color);
	
	QTableWidgetItem* range = new QTableWidgetItem();					// add the range item
	QFont ft = range->font();
	if(path->getRange() == 0) {
		range->setData(Qt::DisplayRole,QString::fromWCharArray(L"\x221e"));
		ft.setPointSize(18);
	}
	else {
		range->setData(Qt::DisplayRole,path->getRange());
		ft.setPointSize(12);
	}
	range->setFont(ft);
	range->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	range->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(m_selectedPath,1,range);
	
	QTableWidgetItem* length = new QTableWidgetItem("...");				// add the length item
	length->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	length->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(m_selectedPath,2,length);
	
	QTableWidgetItem* stint = new QTableWidgetItem("...");				// add the time item
	stint->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	stint->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(m_selectedPath,3,stint);
	
	QTableWidgetItem* eff = new QTableWidgetItem("...");				// add the efficiency item
	eff->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
	eff->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
	pathTableWidget->setItem(m_selectedPath,4,eff);
}

// when the [-] button is pressed
void pathTool::on_buttonDelete_clicked()
{
	if(m_selectedPath >= pathList.size()) m_selectedPath = pathList.size()-1;
	delete pathList[m_selectedPath];
	pathList.removeAt(m_selectedPath);
	
	if(pathList.isEmpty()) buttonDelete->setEnabled(false);
	pathTableWidget->removeRow(m_selectedPath);
}

void pathTool::on_buttonGenerate_clicked()
{
	resetPaths();
	if(checkBoxSave->isChecked()){
		
	}
	emit changeBackground(0,QBrush(QColor("springgreen")));
	emit computePaths(0);
}

void pathTool::processPath(int x)
{
	qApp->processEvents();
	if(x >= pathList.size()){
		QSound::play("/Users/mattroman/Documents/code/roverSim/src/sounds/singleBell.wav");
		return;
	}
	
	if(x != 0) m_foundSound.play();

	pathList[x]->goForGoal(rover->position - btVector3(0,0,0.34),goalPoint);
	updateTool();
	
	if(pathList[x]->isLooping())
		emit changeBackground(x,QBrush(QColor(Qt::yellow)));
	else
		emit changeBackground(x,QBrush(QColor("lightsteelblue")));
	x++;
	emit changeBackground(x,QBrush(QColor("springgreen")));

	emit computePaths(x);
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
					item->setData(Qt::DecorationRole,pathList[i]->getColor());
					break;
				}
				case 1:
				{
					float r = pathList[i]->getRange();
					if(r == 0)	item->setData(Qt::DisplayRole,QString::fromWCharArray(L"\x221e"));
					else item->setData(Qt::DisplayRole,r);
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
				case 4:
				{
					item->setData(Qt::DisplayRole,QVariant(gp->efficiency));
					break;
				}
				default:
				break;
			}
		}
	}
}

void pathTool::setRowBackground(int row, QBrush stroke)
{
	if(row >= pathTableWidget->rowCount()) return;
	for(int i=1;i<pathTableWidget->columnCount();i++){
		QTableWidgetItem* item = pathTableWidget->item(row,i);
		item->setBackground(stroke);
	}
}

void pathTool::tableDataChange(int row, int column)
{
	pathTableWidget->selectRow(row);
	if(row == m_selectedPath) return;
	if(m_selectedPath < pathList.size())
		pathList[m_selectedPath]->togglePathReset();
	m_selectedPath = row;
}

void pathTool::tableDataEdit(int row, int column)
{
	m_selectedPath = row;
	
	// bring up editing dialog box
	pathEditDialog eDialog(pathList[row],this);
	eDialog.exec();
	updateTool();
}

void pathTool::stepOnPath(int dir)
{
	if(pathList.isEmpty()) return;
	if(m_selectedPath < pathList.size() && m_selectedPath >= 0)
		pathList[m_selectedPath]->togglePathPoint(dir);
}
