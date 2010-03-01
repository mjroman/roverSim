#include "glparticle.h"
#include "rngs.h"

GLParticle::GLParticle(float x,float y,float z, simGLView* glView)
:
simGLObject(glView),
partCount(100)
{
	int i;
	// the initial particle is set at the maximum parameters
	initPart.position.x = x;
	initPart.position.y = y;
	initPart.position.z = z;
	initPart.motion.x = 0.1;
	initPart.motion.y = 0.1;
	initPart.motion.z = 0.2;
	initPart.color.x = 1;
	initPart.color.y = 1;
	initPart.color.z = 0;
	initPart.rotation = 90;
	initPart.accel = 0.08;
	initPart.decel = 0.0025;
	initPart.scale = 0.1;
	
	// initialize all particles
	partList = new particle[partCount];
	for(i=0;i<partCount;i++){
		partList[i] = initPart;
		partList[i].accel = Randomn()*0.2;
	}
}

GLParticle::~GLParticle()
{
	delete [] partList;
}

void GLParticle::setPosition(float x,float y,float z)
{
	initPart.position.x = x;
	initPart.position.y = y;
	initPart.position.z = z;
}
void GLParticle::renderGLObject()
{
	int i;

	glEnable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glDisable(GL_LIGHTING);
	//glBlendFunc(GL_DST_COLOR,GL_ZERO);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBindTexture(GL_TEXTURE_2D,m_view->getTexture(3));
	
	for(i=0;i<partCount;i++){
		if(partList[i].scale < 0.01){
			partList[i] = initPart;
			partList[i].motion.x *= (Randomn() - 0.5);
			partList[i].motion.y *= (Randomn() - 0.5);
			partList[i].accel *= Randomn();
			partList[i].rotation *= Randomn();
			partList[i].scale *= (Randomn() + 0.05);
		}
		else{
			partList[i].position.x += partList[i].motion.x;
			partList[i].position.y += partList[i].motion.y;
			partList[i].position.z += (partList[i].accel - partList[i].decel);
			partList[i].decel += 0.005;
			partList[i].rotation += 20;
			partList[i].scale -= 0.01;
		}
		glColor3f(partList[i].color.x,partList[i].color.y,partList[i].color.z);

		// draw the particle
		glPushMatrix();
		glTranslatef(partList[i].position.x,partList[i].position.y,partList[i].position.z);
		glRotatef(partList[i].rotation,1,0,0);
		glRotatef(partList[i].rotation,0,0,1);
		glScalef(partList[i].scale,partList[i].scale,partList[i].scale);
		// draw the particle
		glBegin (GL_QUADS);
		glTexCoord2d (0, 0);
		glVertex3f (-1, -1, 0);
		glTexCoord2d (1, 0);
		glVertex3f (1, -1, 0);
		glTexCoord2d (1, 1);
		glVertex3f (1, 1, 0);
		glTexCoord2d (0, 1);
		glVertex3f (-1, 1, 0);
		glEnd();
		
		glPopMatrix();
	}
	glEnable(GL_LIGHTING);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);
}
