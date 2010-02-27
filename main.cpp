#include <QtGui/QApplication>
#include "src/mainGUI.h"
#include "src/physicsWorld.h"

int main(int argc, char *argv[])
{
    Q_INIT_RESOURCE(images);

	physicsWorld::initialize(100,100,20,5);

    QApplication a(argc, argv);
	
    MainGUI sim;
    sim.show();
	
    int ret = a.exec();
	
    physicsWorld::destroy();

    return ret;
}
