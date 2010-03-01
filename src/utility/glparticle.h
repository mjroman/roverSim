#ifndef GLPARTICLE_H
#define GLPARTICLE_H

#include <QObject>
#include <OpenGL/gl.h>
#include "../simglobject.h"
#include "definitions.h"

typedef struct _particle
{
	Vertex	position;
	Vertex	motion;
	Vertex	color;
	float	rotation;
	float 	accel;
	float 	decel;
	float	scale;
}particle;

class GLParticle : public QObject, public simGLObject
{
private:
	int			partCount;
	particle	initPart;
	particle	*partList;
	
public:
	GLParticle(float x,float y, float z, simGLView* glView);
	~GLParticle();
	
	void setPosition(float x,float y,float z);
	void renderGLObject();
};
#endif // GLPARTICLE_H