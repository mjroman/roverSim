#include <math.h>
#include "definitions.h"

Vertex diff(Vertex v1,Vertex v2)
{
    Vertex v3;

    v3.x = v1.x - v2.x;
    v3.y = v1.y - v2.y;
    v3.z = v1.z - v2.z;
    return v3;
}

Vertex mult(Vertex v1,Vertex v2)
{
    Vertex v3;

    v3.x = v1.x * v2.x;
    v3.y = v1.y * v2.y;
    v3.z = v1.z * v2.z;

    return v3;
}

Vertex normalize(Vertex v1)
{
    float r;
    r = sqrt((v1.x*v1.x) + (v1.y*v1.y) + (v1.z*v1.z));

    v1.x /= r;
    v1.y /= r;
    v1.z /= r;
    return v1;
}
Vertex normalCross(Vertex v1,Vertex v2)
{
        float r;
        Vertex v3;
        // a x b = ((a2b3 - a3b2),(a3b1 - a1b3),(a1b2 - a2b1))

        v3.x = v1.y*v2.z - v1.z*v2.y;
        v3.y = v1.z*v2.x - v1.x*v2.z;
        v3.z = v1.x*v2.y - v1.y*v2.x;

        r = sqrt((v3.x*v3.x) + (v3.y*v3.y) + (v3.z*v3.z));

        v3.x /= r;
        v3.y /= r;
        v3.z /= r;

        return v3;
}

Vertex vertToVec(Vertex a,Vertex b)
{
    Vertex vector;

    vector.x = b.x - a.x;
    vector.y = b.y - a.y;
    vector.z = b.z - a.z;

    return vector;
}

Vertex sumNormals(int v0,int v1,int v2,int v3,int v4,Vertex *verts)
{
    Vertex vec1,vec2,vec3,vec4;
    Vertex nsum,n1,n2,n3,n4;
    float r;

    if(v1 != -1) { vec1 = vertToVec(verts[v0],verts[v1]); }
    else{ vec1.x = 0; vec1.y = -1; vec1.z = 0; }
    if(v2 != -1) { vec2 = vertToVec(verts[v0],verts[v2]); }
    else{ vec2.x = 1; vec2.y = 0; vec2.z = 0; }
    if(v3 != -1) { vec3 = vertToVec(verts[v0],verts[v3]); }
    else{ vec3.x = 0; vec3.y = 1; vec3.z = 0; }
    if(v4 != -1) { vec4 = vertToVec(verts[v0],verts[v4]); }
    else{ vec4.x = -1; vec4.y = 0; vec4.z = 0; }

    n1 = normalCross(vec1,vec2);
    n2 = normalCross(vec2,vec3);
    n3 = normalCross(vec3,vec4);
    n4 = normalCross(vec4,vec1);

    nsum.x = n1.x + n2.x + n3.x + n4.x;
    nsum.y = n1.y + n2.y + n3.y + n4.y;
    nsum.z = n1.z + n2.z + n3.z + n4.z;

    r = sqrt((nsum.x*nsum.x) + (nsum.y*nsum.y) + (nsum.z*nsum.z));

    nsum.x /= r;
    nsum.y /= r;
    nsum.z /= r;

    return nsum;
}
