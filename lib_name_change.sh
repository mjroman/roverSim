#!/bin/sh
mkdir QtRoverSimulator.app/Contents/Frameworks/
cp -R /Library/Frameworks/LinearMath.framework QtRoverSimulator.app/Contents/Frameworks
cp -R /Library/Frameworks/BulletCollision.framework QtRoverSimulator.app/Contents/Frameworks
cp -R /Library/Frameworks/BulletDynamics.framework QtRoverSimulator.app/Contents/Frameworks

install_name_tool -id @executable_path/../Frameworks/LinearMath.framework/Versions/2.75/LinearMath QtRoverSimulator.app/Contents/Frameworks/LinearMath.framework/Versions/2.75/LinearMath

install_name_tool -id @executable_path/../Frameworks/BulletCollision.framework/Versions/2.75/BulletCollision QtRoverSimulator.app/Contents/Frameworks/BulletCollision.framework/Versions/2.75/BulletCollision

install_name_tool -change /Library/Frameworks/LinearMath.framework/Versions/2.75/LinearMath @executable_path/../Frameworks/LinearMath.framework/Versions/2.75/LinearMath QtRoverSimulator.app/Contents/Frameworks/BulletCollision.framework/Versions/2.75/BulletCollision

install_name_tool -id @executable_path/../Frameworks/BulletDynamics.framework/Versions/2.75/BulletDynamics QtRoverSimulator.app/Contents/Frameworks/BulletDynamics.framework/Versions/2.75/BulletDynamics

install_name_tool -change /Library/Frameworks/BulletCollision.framework/Versions/2.75/BulletCollision @executable_path/../Frameworks/BulletCollision.framework/Versions/2.75/BulletCollision QtRoverSimulator.app/Contents/Frameworks/BulletDynamics.framework/Versions/2.75/BulletDynamics

install_name_tool -change /Library/Frameworks/LinearMath.framework/Versions/2.75/LinearMath @executable_path/../Frameworks/LinearMath.framework/Versions/2.75/LinearMath QtRoverSimulator.app/Contents/Frameworks/BulletDynamics.framework/Versions/2.75/BulletDynamics

install_name_tool -change /Library/Frameworks/BulletCollision.framework/Versions/2.75/BulletCollision @executable_path/../Frameworks/BulletCollision.framework/Versions/2.75/BulletCollision QtRoverSimulator.app/Contents/MacOs/QtRoverSimulator

install_name_tool -change /Library/Frameworks/BulletDynamics.framework/Versions/2.75/BulletDynamics @executable_path/../Frameworks/BulletDynamics.framework/Versions/2.75/BulletDynamics QtRoverSimulator.app/Contents/MacOs/QtRoverSimulator

install_name_tool -change /Library/Frameworks/LinearMath.framework/Versions/2.75/LinearMath @executable_path/../Frameworks/LinearMath.framework/Versions/2.75/LinearMath QtRoverSimulator.app/Contents/MacOs/QtRoverSimulator

otool -L QtRoverSimulator.app/Contents/MacOs/QtRoverSimulator

