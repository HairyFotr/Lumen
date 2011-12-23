#ifndef LUMEN_GEOMETRY
#define LUMEN_GEOMETRY
#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <ni/XnOpenNI.h>
#include <ni/XnCppWrapper.h>
using namespace std;

class Vec3 {
public:
    Vec3(float X, float Y, float Z);
    Vec3();
    float x,y,z;
    
    Vec3 operator+(Vec3 v);
    Vec3 operator-(Vec3 v);
    Vec3 cross(Vec3 v);
    float dot(Vec3 v);
    float length();
    float angle(Vec3 v);
    bool operator==(Vec3 v);
    bool operator!=(Vec3 v);
    Vec3 operator-();
    void operator+=(Vec3 v);
    void operator-=(Vec3 v);
    void operator*=(float f);
    void operator/=(float f);
    bool isZero();
    void normalize();
};

class LinePoint {
    public:
    LinePoint(Vec3 p, Vec3 pp);
    
    Vec3 point, projpoint;
    float thickness;
};

class LinePoints {
protected:
    vector<LinePoint> items;
    
public:
    int Add(LinePoint item);
    int Add(Vec3 p, Vec3 pp);
    int Add(XnPoint3D p, XnPoint3D pp);
 
    void Clear(void);
    int Count(void);
    void Reverse(void);
    LinePoint& operator [](int ItemKey);
};

class Line {
public:
    Line();
    Line(float rr, float gg, float bb);
    Line(float rr, float gg, float bb, int br);
    LinePoints linePoints;
    int brush;
    float r,g,b,a;
    Vec3 avgPoint;
    int displayList;
    
    Vec3 calculateAvgPoint();    
    void compileLine();
    void renderLine();
};

class Lines{
protected:
    vector<Line> items;
    
public:
    int Add(Line item);

    void Clear(void);
    int Count(void);
    Line& operator [](int ItemKey);
};
#endif
