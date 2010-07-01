#ifndef GLSHAPES_H
#define GLSHAPES_H
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

void wheel(float rad,float width);
void tube(float rad,float width);
void box(float length,float width,float height);
void wireBox(float length,float width,float height);
void cylinder(float rad, float length, int res);
void cone(float rad, float length, int res);
void conePoint(float rad, float length, int res);
void sphere(float rad, int lats, int longs);
void wireSymHull(Vertex* pts,int num);

#ifdef __cplusplus
}
#endif

#endif // GLSHAPES_H
