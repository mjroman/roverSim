#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define FARCLIPPING     1001.0
#define NEARCLIPPING    1.0//0.05

#define	PI				3.14159265358979323846
#define TWOPI			2*PI
#define HALFPI 			1.57079632679489661923
#define SQRTTWO			1.41421356
#define DEGTORAD(deg)           (((deg) * PI) / 180.0)
#define RADTODEG(rad)           (((rad) * 180.0) / PI)
#define MINIMUM(x,y)            (((x) < (y)) ? (x) : (y))
#define MAXIMUM(x,y)            (((x) > (y)) ? (x) : (y))
#define SQ(x)			((x)*(x))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _Vertex {
        float x;
        float y;
        float z;
} Vertex;

typedef struct _Point {
    float u;
    float v;
} Point;

typedef struct _Triangle {
        int v1;
        int v2;
        int v3;
} Triangle;

Vertex diff(Vertex v1,Vertex v2);
Vertex mult(Vertex v1,Vertex v2);
Vertex normalize(Vertex v1);
Vertex normalCross(Vertex v1,Vertex v2);
Vertex vertToVec(Vertex a,Vertex b);
Vertex sumNormals(int v0,int v1,int v2,int v3,int v4,Vertex *verts);
float distBtwVerts(Vertex p1,Vertex p2);

#ifdef __cplusplus
}
#endif

#endif // DEFINITIONS_H
