/****************************************************************************
*                                                                           *
*  OpenNI 1.1 Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

//TODO
///Brushi - partikli
///Opacity - tekstura
    ///podatkovno strukturo za črte, 
        //ki se auto depth-sorta
    ///slika zakriva roko/sceno
        //kombinacija kinecta in kamer za dobivanje globine na kamera-sliki
///Barve

///Miško
///Geste - menu
///shranjevanje kalibracij

///join into one file... 
///how to get on github? take whole openni and branch lumen,scala,...?
///hires primesense camera... I can has!?
    //zaklenjeno na kinectu

#include "SceneDrawer.h"
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>
#include <stdio.h>
#include <string>
#include <map>
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <vector>
#include <algorithm>
using namespace std;

extern xn::UserGenerator g_UserGenerator;
extern xn::DepthGenerator g_DepthGenerator;
/////extern xn::ImageGenerator g_ImageGenerator;

extern XnBool g_bDrawBackground;
extern XnBool g_bDrawPixels;
extern XnBool g_bDrawSkeleton;
extern XnBool g_bPrintID;
extern XnBool g_bPrintState;
extern XnBool g_bRotate;
extern XnBool g_bLookFromHead;
extern XnBool g_bClear;
extern int g_TestVar;
extern int currentBrush;
extern XnUInt32 g_nCurrentUser;

#define MAX_DEPTH 10000
float g_pDepthHist[MAX_DEPTH];

#define _USE_MATH_DEFINES
#include <math.h>

/*
unsigned int getClosestPowerOfTwo(unsigned int n) {
    unsigned int m = 2;
    while(m < n) m<<=1;

    return m;
}
*/

//
//trackpad
//

#include <XnVHandPointContext.h>
#include <XnVSessionManager.h>
#include <XnVSelectableSlider2D.h>

extern XnBool g_bActive;
extern XnBool g_bIsInput;
extern XnBool g_bInSession;
extern XnBool g_bIsPushed;
extern XnUInt32 g_nCurrentFrame;

const XnUInt32 XN_PUSH_DISPLAY_FRAMES = 30;

extern XnFloat g_fXValue;
extern XnFloat g_fYValue;

extern XnUInt32 g_nXIndex;
extern XnUInt32 g_nYIndex;

extern XnUInt32 g_TP_XDim;
extern XnUInt32 g_TP_YDim;

const XnFloat GL_WIN_SIZE_X = 640.0;
const XnFloat GL_WIN_SIZE_Y = 480.0;

extern XnVSelectableSlider2D* g_pTrackPad;
extern XnVSessionManager* g_pSessionManager;

extern XnCallbackHandle g_nItemHoverHandle;
extern XnCallbackHandle g_nItemSelectHandle;
extern XnCallbackHandle g_nValueChangeHandle;

extern XnCallbackHandle g_nPrimaryCreateHandle;
extern XnCallbackHandle g_nPrimaryDestroyHandle;

extern XnUInt32 g_TrackPadHandle;

extern XnBool g_isPrintItemHover;
extern XnBool g_isPrintValueChange;
extern XnBool g_isInputStarted;

extern XnPoint3D CurrentItem;

extern XnBool g_bMouseDown;
extern XnBool g_bUseMouse;

// Drawing functions
void DrawLine(const XnPoint3D& ptMins, const XnPoint3D& ptMaxs, int width, double r = 1, double g = 1, double b = 1)
{
    const GLubyte ind[2] = {0, 1};
    GLfloat verts[6] = { ptMins.X, ptMins.Y, ptMins.Z, ptMaxs.X, ptMaxs.Y, ptMaxs.Z };
    glColor4f(r,g,b,1.0f);
    glVertexPointer(3, GL_FLOAT, 0, verts);
    glLineWidth(width);
    glDrawArrays(GL_LINES, 0, 2);
    glFlush();
}

void DrawFrame(const XnPoint3D& ptMins, const XnPoint3D& ptMaxs, int width, double r, double g, double b)
{
    XnPoint3D ptTopLeft = ptMins;
    XnPoint3D ptBottomRight = ptMaxs;

    // Top line
    DrawLine(xnCreatePoint3D(ptTopLeft.X, ptTopLeft.Y, 0),
        xnCreatePoint3D(ptBottomRight.X, ptTopLeft.Y, 0),
        width, r, g, b);
    // Right Line
    DrawLine(xnCreatePoint3D(ptBottomRight.X, ptTopLeft.Y, 0),
        xnCreatePoint3D(ptBottomRight.X, ptBottomRight.Y,0),
        width, r, g, b);
    // Bottom Line
    DrawLine(xnCreatePoint3D(ptBottomRight.X, ptBottomRight.Y,0),
        xnCreatePoint3D(ptTopLeft.X, ptBottomRight.Y,0),
        width, r, g, b);
    // Left Line
    DrawLine(xnCreatePoint3D(ptTopLeft.X, ptBottomRight.Y,0),
        xnCreatePoint3D(ptTopLeft.X, ptTopLeft.Y,0),
        width, r, g, b);
}

// More drawing
void DrowTrackPad()
{
    if(!g_bInSession) return;
  
    printf("drowtrackpad\n");

    XnDouble r, g, b;

    if(!g_bActive) {
        r = g = b = 1;
    } else if(!g_bIsInput) {
        r = g = b = 0.5;
    } else {
        r = b = 0;
        g = 1;
    }


  XnFloat width = 20;
  XnFloat x_step = (XnFloat)GL_WIN_SIZE_X/g_TP_XDim;
  for(XnUInt32 i=0 ; i<=g_TP_XDim ; ++i) {
    DrawLine(xnCreatePoint3D(GL_WIN_SIZE_X-(i*x_step), GL_WIN_SIZE_Y, 0.0),
      xnCreatePoint3D(GL_WIN_SIZE_X-(i*x_step), 0.0, 0.0),
      width, r, g, b);
  }

  XnFloat y_step = (XnFloat)GL_WIN_SIZE_Y/g_TP_YDim;
  for(XnUInt32 j=1 ; j<=g_TP_YDim ; ++j) {
    DrawLine(xnCreatePoint3D(GL_WIN_SIZE_X, GL_WIN_SIZE_Y-(j*y_step), 0.0),
     xnCreatePoint3D(0.0, GL_WIN_SIZE_Y-(j*y_step), 0.0),
     width, r, g, b); 
  }

  if(TRUE == g_isPrintItemHover) {
    XnPoint3D ptTopLeft = xnCreatePoint3D((CurrentItem.X*x_step), /*GL_WIN_SIZE_Y-*/(CurrentItem.Y*y_step), 0.0);
    XnPoint3D ptBottomRight = xnCreatePoint3D(((CurrentItem.X+1)*x_step), /*GL_WIN_SIZE_Y-*/((CurrentItem.Y+1)*y_step), 0.0);

    if(TRUE == g_isInputStarted)
      DrawFrame(ptTopLeft, ptBottomRight, width, 1, 1, 0);
    else
      DrawFrame(ptTopLeft, ptBottomRight, width, 0.2, 0.2, 0);
  }

  width /= 2;
  if(TRUE == g_isPrintValueChange) {
    XnFloat TopPointX = (g_fXValue)*GL_WIN_SIZE_X;
    XnFloat TopPointY = (g_fYValue)*GL_WIN_SIZE_Y;
    XnPoint3D ptTopLeft = xnCreatePoint3D(TopPointX, TopPointY, 0.0);
    XnPoint3D ptBottomRight = xnCreatePoint3D(TopPointX+width, TopPointY+width, 0.0);

    if(TRUE == g_isInputStarted)
      DrawFrame(ptTopLeft, ptBottomRight, width, 1, 0, 0);
    else
      DrawFrame(ptTopLeft, ptBottomRight, width, 0.2, 0, 0);
  }

  if(TRUE == g_bIsPushed) {
    XnPoint3D ptTopLeft = xnCreatePoint3D((CurrentItem.X*x_step), (CurrentItem.Y*y_step), 0.0);
    XnPoint3D ptBottomRight = xnCreatePoint3D(((CurrentItem.X+1)*x_step), ((CurrentItem.Y+1)*y_step), 0.0);
    DrawFrame(ptTopLeft, ptBottomRight, width, 1, 0.5, 0);

    ++g_nCurrentFrame;
    if(XN_PUSH_DISPLAY_FRAMES <= g_nCurrentFrame) {
      g_bIsPushed = FALSE;
      g_nCurrentFrame = 0;
    }
  }
}





//
//
//

XnFloat Colors[][3] = {{0,1,1}, {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0}, {1,.5,0}, {.5,1,0}, {0,.5,1}, {.5,0,1}, {1,1,.5}, {1,1,1}};
XnUInt32 nColors = 10;

void glPrintString(void *font, char *str) {
    int i,l = strlen(str);

    for(i=0; i<l; i++) glutBitmapCharacter(font,*str++);
}

XnPoint3D GetLimbPosition(XnUserID player, XnSkeletonJoint eJoint) {
    XnSkeletonJointPosition joint;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

    XnPoint3D pt = joint.position;

    g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);
    return pt;
}

void DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2) {
    if(!g_UserGenerator.GetSkeletonCap().IsTracking(player)) return;
    
    XnSkeletonJointPosition joint1, joint2;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);
    
    if(joint1.fConfidence <= 0 || joint2.fConfidence <= 0) return;
    
    XnPoint3D pt[2];
    pt[0] = joint1.position;
    pt[1] = joint2.position;
        
    g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
    glVertex3f(pt[0].X, pt[0].Y, pt[0].Z);
    glVertex3f(pt[1].X, pt[1].Y, pt[1].Z);
}

const XnChar* GetCalibrationErrorString(XnCalibrationStatus error) {
    switch (error) {
        case XN_CALIBRATION_STATUS_OK: return "OK";
        case XN_CALIBRATION_STATUS_NO_USER: return "NoUser";
        case XN_CALIBRATION_STATUS_ARM: return "Arm";
        case XN_CALIBRATION_STATUS_LEG: return "Leg";
        case XN_CALIBRATION_STATUS_HEAD: return "Head";
        case XN_CALIBRATION_STATUS_TORSO: return "Torso";
        case XN_CALIBRATION_STATUS_TOP_FOV: return "Top FOV";
        case XN_CALIBRATION_STATUS_SIDE_FOV: return "Side FOV";
        case XN_CALIBRATION_STATUS_POSE: return "Pose";
        default: return "Unknown";
    }
}
const XnChar* GetPoseErrorString(XnPoseDetectionStatus error) {
    switch (error) {
        case XN_POSE_DETECTION_STATUS_OK: return "OK";
        case XN_POSE_DETECTION_STATUS_NO_USER: return "NoUser";
        case XN_POSE_DETECTION_STATUS_TOP_FOV: return "Top FOV";
        case XN_POSE_DETECTION_STATUS_SIDE_FOV: return "Side FOV";
        case XN_POSE_DETECTION_STATUS_ERROR: return "General error";
        default: return "Unknown";
    }
}

GLUquadricObj* quadric;


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
        }*/
    
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
        r = (float)rand()/(float)RAND_MAX;
        g = (float)rand()/(float)RAND_MAX;
        b = (float)rand()/(float)RAND_MAX;
        displayList = -1;
    }    
    Line(int b) {
        brush = b;
        r = (float)rand()/(float)RAND_MAX;
        g = (float)rand()/(float)RAND_MAX;
        b = (float)rand()/(float)RAND_MAX;
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
            //r=1; g=0; b=0.3; 
            a=0.55;
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
                    
                    //brush = 1;
                    
                    switch(brush) {
                        case 4: {
                            glBegin(GL_QUADS);
                            
                            int s[4*4]={+1,+1,-1,+1, 
                                        +1,-1,-1,-1,
                                        -1,-1,-1,+1,
                                        +1,-1,+1,+1};
                            float size = 3;
                            
                            Vec3 v[4];
                            if(start == true) {
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
                            for(int i=0; i<4*4; i+=4) {
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
                            if(end == true) {
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
                        } case 3: {
                            glBegin(GL_QUADS);
                            Vec3 n = vecA.cross(vecB); n.normalize();
                            glNormal3f(n.x, n.y, n.z);
                            glVertex3f(vecA.x+4, vecA.y, vecA.z);
                            glVertex3f(vecA.x-4, vecA.y, vecA.z);
                            glVertex3f(vecB.x-4, vecB.y, vecB.z);
                            glVertex3f(vecB.x+4, vecB.y, vecB.z);
                            glEnd();
                            break;
                        } case 1:case 2:default: {
                            Vec3 unit = Vec3(0,0,1);
                            Vec3 d = vecA - vecB;
                            Vec3 cross = unit.cross(d);
                            float angle = unit.angle(d);
                            
                            float size = 3.5;
                            int polycount = 20;
                            if(currentBrush == 1) polycount = 8;
                            
                            if(start == true) {
                                start = false;
                                glPushMatrix();
                                glTranslatef(vecA.x,vecA.y,vecA.z);
                                glRotatef(angle,cross.x,cross.y,cross.z);
                                //glColor4f(r, g, b, a);
                                gluSphere(quadric, size*lp1.thickness, polycount,polycount);
                                glPopMatrix();
                            }

                            glPushMatrix();
                            glTranslatef(vecB.x,vecB.y,vecB.z);
                            glRotatef(angle,cross.x,cross.y,cross.z);
                            //glColor4f(r, g, b, a);
                            gluCylinder(quadric, size*lp1.thickness, size*lp2.thickness, d.length(), polycount,3);
                            //glColor4f(r, g, b, a);
                            gluSphere(quadric, size*lp2.thickness, polycount,polycount);
                            glPopMatrix();
                            break;
                        }
                    }
                }
            }
            glDisable(GL_CULL_FACE);
        }
    }
    /*int Add(Vec3 p, Vec3 pp) { return linePoints.Add(LinePoint(p, pp)); }
    int Add(XnPoint3D p, XnPoint3D pp) { return Add(Vec3(p.X,p.Y,p.Z), Vec3(pp.X,pp.Y,pp.Z)); }
    LinePoint& operator [](int ItemKey) { return linePoints[ItemKey]; }*/
};

class Lines {
protected:
    //The Vector container that will hold the collection of Items
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


#define MAX_POINTS 400
XnPoint3D lastPosition, lastPositionProj;
bool drawing = false;

//new
Line currentLine;
Lines lines;

bool g_pTexMapInit = false;
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;

cv::VideoCapture cap;
cv::Mat frame;
cv::Size size;
GLuint cameraImageTextureID;

XnPoint3D headpos;

float fov = 0;
float minZ = 10000, maxZ = 0;

void DrawDepthMap(const xn::ImageMetaData& imd, const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd) {
    if(!g_pTexMapInit) {
        // init quadric
        quadric = gluNewQuadric();
        gluQuadricNormals(quadric, GLU_SMOOTH);   // Create Smooth Normals
        //gluQuadricTexture(quadric, GL_TRUE);      // Create Texture Coords
        
        // init kinect texture
        g_nTexMapX = (((unsigned short)(dmd.FullXRes()-1) / 512) + 1) * 512;
        g_nTexMapY = (((unsigned short)(dmd.FullYRes()-1) / 512) + 1) * 512;
        g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));
        g_pTexMapInit = true;
    
        // init camera and its texture
        cap.open(0);
        cap >> frame;
        if(frame.data) {
            printf("%dx%d\n", frame.cols, frame.rows);
            glEnable(GL_TEXTURE_RECTANGLE_ARB);

            glGenTextures(1, &cameraImageTextureID);
            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);

            glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
            glDisable(GL_TEXTURE_RECTANGLE_ARB);
        }
        
        // init head position
        headpos.X = 0;
        headpos.Y = 0;
        headpos.Z = 0;
        
        // init mouse
        /*int mouseParam=5;
        cvSetMouseCallback("lumen",mouseHandler,&mouseParam);*/
    }
    
    const XnDepthPixel* pDepth = dmd.Data();
    /////const XnUInt8* pImage = imd.Data();

    /////int width = imd.XRes(), height = imd.YRes();
    int width = 640, height = 480;
    
    // render head-camera
    cap >> frame;

    if(frame.data) {
        size = frame.size();

        // set 2D viewpoint for head-camera view
        glDisable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, size.width, size.height, 0, -1.0, 1.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // set and render camera Texture
        if(frame.channels() == 3) {
            glTexImage2D( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, size.width, size.height, 0, GL_BGR, GL_UNSIGNED_BYTE, frame.data );
        } else if(frame.channels() == 4) {
            glTexImage2D( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, size.width, size.height, 0, GL_BGRA, GL_UNSIGNED_BYTE, frame.data );
        }

        // camera
        glEnable(GL_TEXTURE_RECTANGLE_ARB);
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraImageTextureID);

        glBegin(GL_QUADS);
        glTexCoord2i(0,size.height);          glVertex3f(0.0,size.height, 0.0);
        glTexCoord2i(size.width,size.height); glVertex3f(size.width,size.height, 0.0);
        glTexCoord2i(size.width,0);           glVertex3f(size.width,0.0, 0.0);
        glTexCoord2i(0,0);                    glVertex3f(0.0,0.0, 0.0);
        glEnd();
        glDisable(GL_TEXTURE_RECTANGLE_ARB);

        // user status indicator
        //if(headpos.X == 0 || isnan(headpos.X)) {
        if(g_nCurrentUser == -1) {
            glColor3f(1,0,0); //else glColor3f(0,1,0);
            glBegin(GL_QUADS);
            glVertex3f(5,25, 0.0);
            glVertex3f(25,25, 0.0);
            glVertex3f(25,5, 0.0);
            glVertex3f(5,5, 0.0);
            glEnd();
        }
        
        DrowTrackPad();
        //if(!g_bInSession) return;
        
        glEnable(GL_DEPTH_TEST);
    }//*/
    
    // setup the 3D viewpoint
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    /*if(35+g_TestVar*5 != fov) {
        fov = 35+g_TestVar*5;
        printf("%f\n", fov);
    }*/
    gluPerspective(25, (640/480+0.0), 1.0, 5000.0);
    glScalef(-1,1,1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    //if(g_bLookFromHead) {
    gluLookAt(headpos.X,headpos.Y,headpos.Z,    // camera position
              headpos.X,headpos.Y,headpos.Z-10, // look-at vector
              0.0,-1.0,0.0);                    // up vector 
    /*} else {
        gluLookAt(900.0,0.0,900.0, // camera position
                  0.0,0.0,0.0,     // look-at vector
                  0.0,-1.0,0.0);   // up vector 
    }*/
    
    /*glRotatef(rot,0,1,0);
    if(g_bRotate) {
      rot += 1.0;
    }*/

    /*
    if(g_bDrawPixels) {
        // kinect camera
        {
            const XnRGB24Pixel* pImageRow = imd.RGB24Data();
            XnRGB24Pixel* pTexRow = g_pTexMap + imd.YOffset() * g_nTexMapX;

            for(XnUInt y = 0; y < imd.YRes(); ++y) {
                const XnRGB24Pixel* pImage = pImageRow;
                XnRGB24Pixel* pTex = pTexRow + imd.XOffset();

                for(XnUInt x = 0; x < imd.XRes(); ++x, ++pImage, ++pTex) {
                    *pTex = *pImage;
                }

                pImageRow += imd.XRes();
                pTexRow += g_nTexMapX;
            }
        }
        // kinect depth
        {
            // Calculate the accumulative histogram (the yellow display...)
            xnOSMemSet(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));

            unsigned int nNumberOfPoints = 0;
            for(XnUInt y = 0; y < dmd.YRes(); ++y) {
                for(XnUInt x = 0; x < dmd.XRes(); ++x, ++pDepth) {
                    if(*pDepth != 0) {
                        g_pDepthHist[*pDepth]++;
                        nNumberOfPoints++;
                    }
                }
            }
            for(int nIndex=1; nIndex<MAX_DEPTH; nIndex++) {
                g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
            }
            if(nNumberOfPoints) {
                for(int nIndex=1; nIndex<MAX_DEPTH; nIndex++) {
                    g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
                }
            }

            xnOSMemSet(g_pTexMap, 0, g_nTexMapX*g_nTexMapY*sizeof(XnRGB24Pixel));
            
            const XnDepthPixel* pDepthRow = dmd.Data();
            XnRGB24Pixel* pTexRow = g_pTexMap + dmd.YOffset() * g_nTexMapX;

            for(XnUInt y = 0; y < dmd.YRes(); ++y) {
                const XnDepthPixel* pDepth = pDepthRow;
                XnRGB24Pixel* pTex = pTexRow + dmd.XOffset();

                for(XnUInt x = 0; x < dmd.XRes(); ++x, ++pDepth, ++pTex) {
                    if(*pDepth != 0) {
                        int nHistValue = g_pDepthHist[*pDepth];
                        pTex->nRed = (pTex->nRed+nHistValue)/2;
                        pTex->nGreen = (pTex->nGreen+nHistValue)/2;
                        pTex->nBlue = (pTex->nBlue+nHistValue)/2;
                    }
                }

                pDepthRow += dmd.XRes();
                pTexRow += g_nTexMapX;
            }
        }
        /*
        // Create the OpenGL texture map
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_nTexMapX, g_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, g_pTexMap);

        // Display the OpenGL texture map
        glColor4f(1,1,1,1);

        glBegin(GL_QUADS);

        int nXRes = dmd.FullXRes();
        int nYRes = dmd.FullYRes();

        // upper left
        glTexCoord2f(0, 0);
        glVertex2f(0, 0);
        // upper right
        glTexCoord2f((float)nXRes/(float)g_nTexMapX, 0);
        glVertex2f(width, 0);
        // bottom right
        glTexCoord2f((float)nXRes/(float)g_nTexMapX, (float)nYRes/(float)g_nTexMapY);
        glVertex2f(width, height);
        // bottom left
        glTexCoord2f(0, (float)nYRes/(float)g_nTexMapY);
        glVertex2f(0, height);

        glEnd();
    } */ //g_bDrawPixels

    if(g_bClear) {
        g_bClear = false;
        drawing = false;
        lines.Clear();
    }
    
    XnUserID aUsers[15];
    XnUInt16 nUsers = 15;
    g_UserGenerator.GetUsers(aUsers, nUsers);
    if(nUsers==0) {
        headpos.X = 0;
        headpos.Y = 0;
        headpos.Z = 0;
    }
    glDisable(GL_TEXTURE_2D);
    //for(int i = 0; i < nUsers; i++) {
    if(nUsers > 0 && g_nCurrentUser <= nUsers) {
        int i = g_nCurrentUser-1;
        XnUserID player = aUsers[i];
        XnSkeletonJoint eJoint = XN_SKEL_RIGHT_HAND;
        
        XnSkeletonJointPosition joint;
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);
        
        XnPoint3D currentPosition = joint.position;
        XnPoint3D currentPositionProj; g_DepthGenerator.ConvertRealWorldToProjective(1, &currentPosition, &currentPositionProj);

        if(joint.fConfidence > 0 && drawing==false && ((g_bUseMouse==true && g_bMouseDown==true) 
        ||(g_bUseMouse==false &&  
           abs(lastPosition.X-currentPosition.X) + 
           abs(lastPosition.Y-currentPosition.Y) + 
           abs(lastPosition.Z-currentPosition.Z)/15 > 35))) { // && currentPosition.Z < headpos.Z
            
            //start new line
            drawing = true;
            
            currentLine = Line(currentBrush);
            
            if(lastPosition.X != 0 && lastPosition.Y != 0 && lastPosition.Z != 0) {
                currentLine.linePoints.Add(lastPosition, lastPositionProj);
            }
            currentLine.linePoints.Add(currentPosition, currentPositionProj);
            
            printf("line begin %d (%1.2f,%1.2f,%1.2f)\n", lines.Count(),lastPosition.X,lastPosition.Y,lastPosition.Z);
        } else if(joint.fConfidence > 0 && drawing==true && ((g_bUseMouse==true && g_bMouseDown==false) 
               ||(g_bUseMouse==false &&
                  abs(lastPosition.X-currentPosition.X) + 
                  abs(lastPosition.Y-currentPosition.Y) + 
                  abs(lastPosition.Z-currentPosition.Z) < 20))) {
            drawing = false;
            
            printf("line end (%1.2f,%1.2f,%1.2f)\n", lastPosition.X,lastPosition.Y,lastPosition.Z);
            currentLine.compileLine();
            lines.Add(currentLine);            
        } else if(drawing) {
            currentLine.linePoints.Add(currentPosition, currentPositionProj);
        }
        lastPosition = joint.position;
        
        for(int l = 0; l < lines.Count(); l++) lines[l].renderLine();
        if(drawing) currentLine.renderLine();
        
        if(g_bDrawSkeleton) {
            glBegin(GL_LINES);
            glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);

            DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
            DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
            glEnd();
        }
        /*if(g_bDrawSkeleton) {
            glBegin(GL_LINES);
            glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);
            DrawLimb(aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK);
            
            DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
            DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
            DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
            
            DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
            DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
            DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
            
            DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
            DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);
            
            glEnd();
        }*/
        

        //headpos = GetLimbPosition(aUsers[i], XN_SKEL_NECK);
        headpos = GetLimbPosition(aUsers[i], XN_SKEL_HEAD);
    }
    glEnable(GL_TEXTURE_2D);//*/
    
    /*GLenum error = glGetError();
    if(error) printf("%s\n", gluErrorString(error));*/

    //glutSwapBuffers();
}
