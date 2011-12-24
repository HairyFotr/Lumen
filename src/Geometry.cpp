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

//horrible, I know :P
extern GLUquadricObj* quadric;

class Vec3 {
public:
    Vec3(float X, float Y, float Z) { x = X; y = Y; z = Z; }
    Vec3() { x = 0; y = 0; z = 0; }
    float x,y,z;
    
    Vec3 operator+(Vec3 v) { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(Vec3 v) { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 cross(Vec3 v) { return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
    float dot(Vec3 v) { Vec3 n = Vec3(x*v.x, y*v.y, z*v.z); return n.x+n.y+n.z; }
    float length() { return sqrt(x*x+y*y+z*z); }
    float angle(Vec3 v) { return (180.0/M_PI * acos((*this).dot(v)/v.length())); }
    bool operator==(Vec3 v) { return (x==v.x && y==v.y && z==v.z); }
    bool operator!=(Vec3 v) { return !(*this==v); }
    Vec3 operator-() { return Vec3(-x,-y,-z); }
    void operator+=(Vec3 v) { x += v.x; y += v.y; z += v.z; }
    void operator-=(Vec3 v) { x -= v.x; y -= v.y; z -= v.z; }
    void operator*=(float f) { x *= f; y *= f; z *= f; }
    void operator/=(float f) { x /= f; y /= f; z /= f; }
    bool isZero() { return (x==0 && y==0 && z==0); }
    void normalize() { 
        float len = length();
        x/=len; y/=len; z/=len;
    }
};

class LinePoint {
    public:
    LinePoint(Vec3 p, Vec3 pp) { 
        point = p;
        projpoint = pp;
        thickness = 1;
    }
    
    Vec3 point, projpoint;//originalpoint, projected point
    float thickness;
};

class LinePoints {
protected:
    //The Vector container that will hold the collection of Items
    vector<LinePoint> items;
    
public:
    int Add(LinePoint item) {
        //variabilna debelina
        /*
        if(items.size() > 0) {
            Vec3 d = items[items.size()-1].point - item.point;
            item.thickness = (140-d.length())/50;
            if(item.thickness < 0.1) item.thickness = 0.1;
            if(item.thickness > 150) item.thickness = 150;
            
            //average, current has *2 weight
            float avg = item.thickness*2;
            int i;
            for(i = 1; (items.size()-i >= 0)&&(i < 5); i++) {
                avg += items[items.size()-i].thickness;
            }
            item.thickness = avg/(i+2);
        }//*/
    
        //Add the item to the container
        items.push_back(item); 

        //Return the position of the item within the container.
        return (items.size()-1);
    }
    int Add(Vec3 p, Vec3 pp) { return Add(LinePoint(p, pp)); }
    int Add(XnPoint3D p, XnPoint3D pp) { return Add(Vec3(p.X,p.Y,p.Z), Vec3(pp.X,pp.Y,pp.Z)); }    
    //LinePoint* GetLinePoint(int ItemKey) { return &(items[ItemKey]); }    
    //void Remove(int ItemKey) { items.erase(GetLinePoint(ItemKey)); }    
    void Clear(void) { items.clear(); }  
    int Count(void) { return items.size(); }      
    void Reverse(void) { reverse(items.begin(), items.end()); }      
    LinePoint& operator [](int ItemKey) { return items[ItemKey]; }
};

class Line {
public:
    Line() {
        brush = 0; 
        displayList = -1;
    }
    Line(float rr, float gg, float bb) {
        brush = 0; 
        r=rr;
        g=gg;
        b=bb;
        displayList = -1;
    }    
    Line(float rr, float gg, float bb, int br) {
        brush = br;
        r=rr;
        g=gg;
        b=bb;
        displayList = -1;
    }
    LinePoints linePoints;
    int brush;//tube, rectangle, flat, ...
    float r,g,b,a;
    Vec3 avgPoint;
    int displayList;
    
    Vec3 calculateAvgPoint() {
        //if Z of start > Z end, reverse
        if(linePoints[0].point.z > linePoints[linePoints.Count()-1].point.z && brush < 3) { linePoints.Reverse(); }
        
        //calculate avg
        Vec3 avg = Vec3(0,0,0);
        for(int i=0; i<linePoints.Count(); i++) {
            avg += linePoints[i].point;
        }
        avg /= linePoints.Count();
        avgPoint = avg;
        
        return avg;
    }
    
    void compileLine() {
        displayList = -1;
        int displaylist = glGenLists(1);
        glNewList(displaylist,GL_COMPILE);
        renderLine();
        glEndList();
        displayList = displaylist;
    }
        
    void renderLine() {
        if(displayList != -1 && glIsList(displayList)) {
            glCallList(displayList);
        } else {
            a=0.5;
            glColor4f(r, g, b, a);
            // Two-pass rendering - front/back
            glEnable(GL_CULL_FACE);
            for(int pass=0; pass<=1; pass++) {
                if(pass==0) glCullFace(GL_FRONT); else glCullFace(GL_BACK);
                
                bool start = true, end = false;
                
                for(int p = 0; p < linePoints.Count()-1; p++) {
                    if(p==linePoints.Count()-2) end = true;
                    
                    LinePoint lp1 = linePoints[p], lp2 = linePoints[p+1];
                    Vec3 vecA = lp1.projpoint;
                    Vec3 vecB = lp2.projpoint;
                    
                    if(vecA.isZero() || vecB.isZero()) continue;
                    
                    switch(brush) {
                        case 2: {
                            glBegin(GL_QUADS);
                            
                            int s[4*4]={+1,+1,-1,+1, 
                                        +1,-1,-1,-1,
                                        -1,-1,-1,+1,
                                        +1,-1,+1,+1};
                            float size = 3;
                            
                            Vec3 v[4];
                            if(start == true) { // start cap
                                start = false;
                                v[0] = vecA; v[0].x += +size*lp1.thickness; v[0].y += +size*lp1.thickness;
                                v[1] = vecA; v[1].x += -size*lp1.thickness; v[1].y += +size*lp1.thickness;
                                v[2] = vecA; v[2].x += -size*lp1.thickness; v[2].y += -size*lp1.thickness;
                                v[3] = vecA; v[3].x += +size*lp1.thickness; v[3].y += -size*lp1.thickness;
                                
                                Vec3 n;
                                for(int j=0; j<4; j++) n += v[j].cross(v[(j+1)%4]);
                                n.normalize();
                                glNormal3f(n.x, n.y, n.z);
                                
                                for(int k=0; k<4; k++) glVertex3f(v[k].x, v[k].y, v[k].z);
                            }
                            for(int i=0; i<4*4; i+=4) { // middle
                                v[0] = vecA; v[0].x += size*lp1.thickness*s[i+0]; v[0].y += size*lp1.thickness*s[i+1];
                                v[1] = vecA; v[1].x += size*lp1.thickness*s[i+2]; v[1].y += size*lp1.thickness*s[i+3];
                                v[2] = vecB; v[2].x += size*lp2.thickness*s[i+2]; v[2].y += size*lp2.thickness*s[i+3];
                                v[3] = vecB; v[3].x += size*lp2.thickness*s[i+0]; v[3].y += size*lp2.thickness*s[i+1];
                                
                                Vec3 n;
                                for(int j=0; j<4; j++) n += v[j].cross(v[(j+1)%4]);
                                n.normalize();
                                glNormal3f(n.x, n.y, n.z);
                                
                                for(int k=0; k<4; k++) glVertex3f(v[k].x, v[k].y, v[k].z);
                            }
                            if(end == true) { // end cap
                                end = false;
                                v[0] = vecB; v[0].x += +size*lp2.thickness; v[0].y += +size*lp2.thickness;
                                v[1] = vecB; v[1].x += -size*lp2.thickness; v[1].y += +size*lp2.thickness;
                                v[2] = vecB; v[2].x += -size*lp2.thickness; v[2].y += -size*lp2.thickness;
                                v[3] = vecB; v[3].x += +size*lp2.thickness; v[3].y += -size*lp2.thickness;

                                Vec3 n;
                                for(int j=0; j<4; j++) n += v[j].cross(v[(j+1)%4]);
                                n.normalize();
                                glNormal3f(n.x, n.y, n.z);
                                
                                for(int k=0; k<4; k++) glVertex3f(v[k].x, v[k].y, v[k].z);
                            }
                            glEnd();
                            break;
                        } case 1: {
                            glBegin(GL_QUADS);
                            Vec3 n = vecA.cross(vecB); n.normalize();
                            glNormal3f(n.x, n.y, n.z);
                            float size = 4;
                            glVertex3f(vecA.x+size*lp1.thickness, vecA.y, vecA.z);
                            glVertex3f(vecA.x-size*lp1.thickness, vecA.y, vecA.z);
                            glVertex3f(vecB.x-size*lp2.thickness, vecB.y, vecB.z);
                            glVertex3f(vecB.x+size*lp2.thickness, vecB.y, vecB.z);
                            glEnd();
                            break;
                        } case 0:default: {
                            Vec3 unit = Vec3(0,0,1);
                            Vec3 d = vecA - vecB;
                            Vec3 cross = unit.cross(d);
                            float angle = unit.angle(d);
                            
                            float size = 3.5;
                            int polycount = 20;
                            
                            if(start == true) { // start cap
                                start = false;
                                glPushMatrix();
                                glTranslatef(vecA.x,vecA.y,vecA.z);
                                glRotatef(angle,cross.x,cross.y,cross.z);
                                gluSphere(quadric, size*lp1.thickness, polycount,polycount);
                                glPopMatrix();
                            }

                            glPushMatrix();
                            glTranslatef(vecB.x,vecB.y,vecB.z);
                            glRotatef(angle,cross.x,cross.y,cross.z);
                            gluCylinder(quadric, size*lp1.thickness, size*lp2.thickness, d.length(), polycount,3);
                            gluSphere(quadric, size*lp2.thickness, polycount,polycount); // mid/end cap
                            glPopMatrix();
                            break;
                        }
                    }
                }
            }
            glDisable(GL_CULL_FACE);
        }
    }
};

class Lines {
protected:
    vector<Line> items;
    
public:
    int Add(Line item) {

        //insert line at the appropriate Z value
        item.calculateAvgPoint();
        if(items.size() > 0) {
            int i = items.size()-1;
            while(i >= 0 && items[i].avgPoint.z > item.avgPoint.z) i--;
            items.insert(items.begin() + i+1, item);
        } else {
            items.insert(items.begin(), item);
        }
        
        //Return the position of the item within the container.
        return (items.size()-1);
    }
    //Line* GetLine(int ItemKey) { return &(items[ItemKey]); }    
    //void Remove(int ItemKey) { items.erase(GetLine(ItemKey)); }  
    
    //TODO: deep clear - clear inner lists, delete objects and displaylists
    void Clear(void) { items.clear(); }  
    int Count(void) { return items.size(); }      
    Line& operator [](int ItemKey) { return items[ItemKey]; }
};
