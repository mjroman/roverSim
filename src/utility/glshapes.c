#include "glshapes.h"
#include "definitions.h"
#include <math.h>
#include <OpenGL/gl.h>

void wheel(float rad,float width)
{
    int i,j;
    float colorswitch = 0;
    float clr=0.0;
    glColor3f(clr, clr, clr);

    // draw the radial part of the wheel, traction part
    glBegin(GL_QUADS);
    for(i=360;i>=10;i-=10){
        if(colorswitch > 7) {
            colorswitch = 0;
            clr = !clr;
            glColor3f(clr, clr, clr);
        }
        else colorswitch++;
        glNormal3f(0,sin(DEGTORAD(i)),cos(DEGTORAD(i)));
        glVertex3f(-width/2,rad*sin(DEGTORAD(i-10)), rad*cos(DEGTORAD(i-10)));
        glVertex3f(width/2,rad*sin(DEGTORAD(i-10)), rad*cos(DEGTORAD(i-10)));
        glVertex3f(width/2,rad*sin(DEGTORAD(i)),rad*cos(DEGTORAD(i)));
        glVertex3f(-width/2,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
    }
    glEnd();

    // draw the inner and outer hubcaps
    j=-1;
	colorswitch = 0;
	clr = 1.0;
	glColor3f(clr, clr, clr);
	glBegin(GL_TRIANGLES);
	for(i=360;i>=10;i-=10){
		if(colorswitch > 7) {
			colorswitch = 0;
			clr = !clr;
			glColor3f(clr,clr,clr);
		}
		else colorswitch++;

		glNormal3f(-0.707, 0.707*sin(DEGTORAD(i)), 0.707*cos(DEGTORAD(i)));
		glVertex3f(-(width/2+0.02),0,0);
		glVertex3f(-width/2,rad*sin(DEGTORAD(i-10)), rad*cos(DEGTORAD(i-10)));
		glVertex3f(-width/2,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
	}
	glEnd();
	
	colorswitch = 0;
	clr = 1.0;
	glColor3f(clr, clr, clr);
	glBegin(GL_TRIANGLES);
	for(i=360;i>=10;i-=10){
		if(colorswitch > 7) {
			colorswitch = 0;
			clr = !clr;
			glColor3f(clr,clr,clr);
		}
		else colorswitch++;

		glNormal3f(0.707, 0.707*sin(DEGTORAD(i)), 0.707*cos(DEGTORAD(i)));
		glVertex3f((width/2+0.02),0,0);
		glVertex3f(width/2,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
		glVertex3f(width/2,rad*sin(DEGTORAD(i-10)), rad*cos(DEGTORAD(i-10)));
	}
	glEnd();
	

}

void tube(float rad,float width)
{
        int i;

        glBegin(GL_QUAD_STRIP);
        for(i=0;i<=360;i+=20){
                glNormal3f(0, sin(DEGTORAD(i)), cos(DEGTORAD(i)));
                glVertex3f(0,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
                glVertex3f(width,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
        }
        glEnd();

        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(1, 0, 0);
        glVertex3f(width,0,0);
        for(i=360;i>=0;i-=20){
                glVertex3f(width,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
        }
        glEnd();

        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(-1, 0, 0);
        glVertex3f(0,0,0);
        for(i=0;i<=360;i+=20){
                glVertex3f(0,rad*sin(DEGTORAD(i)), rad*cos(DEGTORAD(i)));
        }
        glEnd();
}

void box(float length,float width,float height)
{
    int i;
    Vertex extents = {length,width,height};
    int indices[36] = {
        0,1,2,
        3,2,1,
        4,0,6,
        6,0,2,
        5,1,4,
        4,1,0,
        7,3,1,
        7,1,5,
        5,4,7,
        7,4,6,
        7,2,3,
        7,6,2};

    static Vertex vertices[8]={
        {1,1,1},
        {-1,1,1},
        {1,-1,1},
        {-1,-1,1},
        {1,1,-1},
        {-1,1,-1},
        {1,-1,-1},
        {-1,-1,-1}};

    glBegin (GL_TRIANGLES);
    for (i=0;i<36;i+=3)
    {
        Vertex v1 = mult(vertices[indices[i]],extents);
        Vertex v2 = mult(vertices[indices[i+1]],extents);
        Vertex v3 = mult(vertices[indices[i+2]],extents);
        Vertex normal = normalCross(diff(v2,v1),diff(v3,v1));//(v2-v1).cross(v3-v1);

        glNormal3f(normal.x,normal.y,normal.z);
        glVertex3f (v1.x, v1.y, v1.z);
        glVertex3f (v2.x, v2.y, v2.z);
        glVertex3f (v3.x, v3.y, v3.z);
    }
    glEnd();
}

void wireBox(float length,float width,float height)
{
    int i;
    Vertex extents = {length,width,height};
    int indices[24] = {
        0,1,
		2,3,
        4,5,
        6,7,
        0,2,
        1,3,
        4,6,
        5,7,
        0,4,
        1,5,
        2,6,
        3,7};

    static Vertex vertices[8]={
        {1,1,1},
        {-1,1,1},
        {1,-1,1},
        {-1,-1,1},
        {1,1,-1},
        {-1,1,-1},
        {1,-1,-1},
        {-1,-1,-1}};

    glBegin (GL_LINES);
    for (i=0;i<24;i+=2)
    {
        Vertex v1 = mult(vertices[indices[i]],extents);
        Vertex v2 = mult(vertices[indices[i+1]],extents);

        //Vertex normal = normalCross(diff(v2,v1),diff(v3,v1));//(v2-v1).cross(v3-v1);

        if(i%2) glNormal3f(v1.x,v1.y,v1.z);
		else glNormal3f(v2.x,v2.y,v2.z);
        glVertex3f (v1.x, v1.y, v1.z);
        glVertex3f (v2.x, v2.y, v2.z);
    }
    glEnd();
}

void cylinder(float rad, float length, int res)
{
    int i;
    float py,pz;
    float ny,nz;

    glBegin(GL_QUAD_STRIP);
    for(i=360;i>=0; i-=res){
        ny = cos(DEGTORAD(i));
        nz = sin(DEGTORAD(i));
        py = rad*ny;
        pz = rad*nz;
        glNormal3f(0,ny,nz);
        glVertex3f(-length,py,pz);
        glVertex3f(length,py,pz);
    }
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(-1,0,0);
    glVertex3f(-length,0,0);
    for(i=360;i>=0;i-=res){
        py = rad*cos(DEGTORAD(i));
        pz = rad*sin(DEGTORAD(i));
        glNormal3f(-1,0,0);
        glVertex3f(-length,py,pz);
    }
    glEnd();

    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(1,0,0);
    glVertex3f(length,0,0);
    for(i=0;i<=360;i+=res){
        py = rad*cos(DEGTORAD(i));
        pz = rad*sin(DEGTORAD(i));
        glNormal3f(1,0,0);
        glVertex3f(length,py,pz);
    }
    glEnd();
}

void cone(float rad, float length, int res)
{
    int i;
    float px,py;
    float nx,ny;

    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0,0,1);
    glVertex3f(0,0,length/2);
    for(i=0;i<=360;i+=res){
        nx = cos(DEGTORAD(i));
        ny = sin(DEGTORAD(i));
        px = rad*nx;
        py = rad*ny;
        glNormal3f(nx,ny,0);
        glVertex3f(px,py,-length/2);
    }
    glEnd();
}

void conePoint(float rad, float length, int res)
{
	int i;
    float px,py;
    float nx,ny;

	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0,0,1);
	glVertex3f(0,0,0);
	
	for(i=0;i<=360;i+=res){
		nx = cos(DEGTORAD(i));
		ny = sin(DEGTORAD(i));
		px = rad*nx;
		py = rad*ny;
		//glNormal3f(nx,ny,0);
		glVertex3f(px,py,length);
	}
	glEnd();
}

void sphere(float rad, int lats, int longs)
{
    int i, j;
    for(i = 0; i <= lats; i++) {
        float lat0 = PI * (-0.5 + (float)(i - 1) / lats);
        float z0  = rad*sin(lat0);
        float zr0 =  rad*cos(lat0);

        float lat1 = PI * (-0.5 + (float)i / lats);
        float z1 = rad*sin(lat1);
        float zr1 = rad*cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for(j = 0; j <= longs; j++) {
            float lng = 2 * PI * (float)(j - 1) / longs;
            float x = cos(lng);
            float y = sin(lng);
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(x * zr1, y * zr1, z1);
            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(x * zr0, y * zr0, z0);
        }
        glEnd();
    }
}

void wireSymmetricHull(Vertex* pts,int num)
{
	int i;
	int halfNum = num/2;
	// draw the top loop of the hull
	glBegin(GL_LINE_LOOP);
	Vertex normal = normalCross(diff(pts[1],pts[0]),diff(pts[2],pts[0]));
	glNormal3f(normal.x,normal.y,normal.z);
	for(i = 0; i < halfNum; ++i)
	{
		glVertex3f(pts[i].x,pts[i].y,pts[i].z);
	}
	glEnd();
	
	// draw the bottom loop
	glBegin(GL_LINE_LOOP);
	glNormal3f(-normal.x,-normal.y,-normal.z);
	for(i = halfNum; i < num; ++i)
	{
		glVertex3f(pts[i].x,pts[i].y,pts[i].z);
	}
	glEnd();
	
	// draw the connecting edge lines
	glBegin(GL_LINES);
	glNormal3f(normal.x,normal.y,normal.z);
	for(i=0;i<halfNum;i++)
	{
		glVertex3f(pts[i].x,pts[i].y,pts[i].z);
		glVertex3f(pts[i+halfNum].x,pts[i+halfNum].y,pts[i+halfNum].z);
	}
	glEnd();
}

void radarFan(float* center, float rad)
{
	int i;
	static float j=1;
	static float fade=0.1;

	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPushMatrix();
	glTranslatef(center[0],center[1],center[2]);
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0,0,1);
	glColor4f(1,1,1,0.0);
	glVertex3f(0,0,0);
	glColor4f(0,1,0,fade);
	for(i=0;i<=360;i+=5){
				// if(i==j) glColor4f(1,1,0,0.3);
				// else glColor4f(1,0,0,0.1);
				//glVertex3f(m_detectRange*cos(DEGTORAD(i)),m_detectRange*sin(DEGTORAD(i)),0);
		glVertex3f(j*cos(DEGTORAD(i)),j*sin(DEGTORAD(i)),0);
	}
	glEnd();
	glPopMatrix();
	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);

	if(j>=rad) {
		if(fade > 0) fade-=0.01;
		else{
			j=1;
			fade=0.1;
		}
	}
	else j += 1;
}