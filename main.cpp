#include <QtGui/QApplication>
#include "src/maincontroller.h"
#include "src/physicsWorld.h"

int main(int argc, char *argv[])
{
    Q_INIT_RESOURCE(images);

    QApplication a(argc, argv);
	
    physicsWorld::initialize(100,100,20,5);
	
    MainController w;
    w.show();
	
    int ret = a.exec();
	
    physicsWorld::destroy();

    return ret;
}
