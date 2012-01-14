//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#define LUMEN_CAMERA //use camera
//#define LUMEN_BACKDROP //use backdrop.jpg file instead of camera
#define LUMEN_TRACKER //use the Wrap 920AR tracker 

#include "LumenRender.h"
#include "Geometry.cpp"
#include "Smooth.cpp"
#include "Utils.h"
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#ifdef LUMEN_CAMERA
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#endif
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ni/XnOpenNI.h>
#include <ni/XnCppWrapper.h>
#include <boost/thread.hpp>
using namespace std;

extern xn::UserGenerator g_UserGenerator;
extern xn::DepthGenerator g_DepthGenerator;

extern bool drawSkeleton;
extern bool drawSquare;
extern bool headView;
extern bool doClear;
extern int currentBrush;
extern XnUInt32 currentUser;
extern bool isMouseDown;
extern bool isUsingMouse;
extern float rr;
extern float gg;
extern float bb;

GLUquadricObj* quadric;

#ifdef LUMEN_TRACKER
float initPitch, initYaw, initRoll;
SmoothData *Pitch, *Yaw, *Roll;

char* trackerData = new char[42];
bool trackerInit = false;
bool trackerLock = false;
boost::thread trackerThread;
ifstream file;

union mix_t {
  int16_t i;
  struct {
    char a;
    char b;
  } s;
} uni;
int getInt(char* pos) {
    uni.s.a = *(pos);
    uni.s.b = *(pos+1);
    int res = uni.i;
    return res;
}
void updateTracker() {
    if(trackerInit) {
        Pitch->insert((float)getInt(trackerData+2));
        Yaw->insert((float)getInt(trackerData+4));
        Roll->insert((float)getInt(trackerData+6));
    }
}
void readTracker() {
    if(trackerInit) {
        while(true) {
            trackerLock=true;
            file.read(trackerData, 42);
            updateTracker();
            trackerLock=false;
            //usleep(10*1000);
        }
    }
}

// yes I know this is horrible - fileRead blocks if no data.
boost::thread firstReadThread;
void firstRead() {
    file.read(trackerData, 42);
    if(getInt(trackerData)==-32767) {  
        int smoothlen=20;      
        Pitch = new SmoothData((float)getInt(trackerData+2),smoothlen,1);
        Yaw = new SmoothData((float)getInt(trackerData+4),smoothlen,1);
        Roll = new SmoothData((float)getInt(trackerData+6),smoothlen,1);
        trackerInit = true;
        for(int i=0; i<smoothlen; i++) {
            file.read(trackerData, 42);
            updateTracker();
        }
        initPitch = Pitch->get();
        initYaw = Yaw->get();
        initRoll = Roll->get();
        trackerThread = boost::thread(readTracker);
    }
}
void initTracker() {
    file.open("/dev/hidraw5", ios::in|ios::binary);
    usleep(1000*1000);
    if(file.is_open()) {
        file.rdbuf()->pubsetbuf(0, 0);
        firstReadThread = boost::thread(firstRead);
    }
}
#endif

#ifdef LUMEN_CAMERA
cv::VideoCapture cap;
cv::Mat frame;
cv::Size size;
GLuint cameraTextureID;
boost::thread camThread;
bool cameraInit = false;
bool cameraLock = false;
void initCamera() {
    #ifdef LUMEN_BACKDROP
    frame = cv::imread("backdrop.jpg");
    #else
    cap.open(-1);
    cap >> frame;
    #endif
    if(frame.data) {
        size = frame.size();
    
        glEnable(GL_TEXTURE_RECTANGLE_ARB);

        glGenTextures(1, &cameraTextureID);
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraTextureID);

        glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glDisable(GL_TEXTURE_RECTANGLE_ARB);

        if(frame.channels() == 3)
            glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGB, size.width,size.height, 0,GL_BGR,GL_UNSIGNED_BYTE, frame.data);
        else if(frame.channels() == 4)
            glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGBA, size.width,size.height, 0,GL_BGRA,GL_UNSIGNED_BYTE, frame.data);

        cameraInit = true;
    }
}
void readCamera() {
    if(cameraInit==true) {
        #ifdef LUMEN_BACKDROP
        #else
        cap >> frame;
        #endif
    }
    cameraLock = false;
}
void updateCamera() {
    #ifdef LUMEN_BACKDROP
    #else
    if(!cameraLock) {
        cameraLock = true;
        camThread.join();
        if(frame.data) {
            if(frame.channels() == 3)
                glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGB, size.width,size.height, 0,GL_BGR,GL_UNSIGNED_BYTE, frame.data);
            else if(frame.channels() == 4)
                glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGBA, size.width,size.height, 0,GL_BGRA,GL_UNSIGNED_BYTE, frame.data);
        }
        camThread = boost::thread(readCamera);
    }
    #endif
}
void renderCamera() {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, size.width, size.height, 0, -1.0, 1.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraTextureID);
    
    glBegin(GL_QUADS);
    glTexCoord2i(0,size.height);            glVertex3f(0,size.height, 0);
    glTexCoord2i(size.width,size.height);   glVertex3f(size.width,size.height, 0);
    glTexCoord2i(size.width,0);             glVertex3f(size.width,0, 0);
    glTexCoord2i(0,0);                      glVertex3f(0,0, 0);
    glEnd();
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
}
#endif

SmoothPoint *headpos;
SmoothPoint *lastPosition;
SmoothPoint *lastPositionProj;
bool drawingLine = false;

Line currentLine;
Lines lines;

bool firstRender = true;
bool firstUser = true;

void glPrintString(void *font, char *str) {
    int i,l = strlen(str);

    for(i=0; i<l; i++) glutBitmapCharacter(font,*str++);
}

XnPoint3D GetLimbPosition(XnUserID player, XnSkeletonJoint eJoint) {
    XnSkeletonJointPosition joint;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);
    
    return joint.position;
}
XnPoint3D getProj(XnPoint3D current) {
    XnPoint3D proj = xnCreatePoint3D(3,3,3);
    g_DepthGenerator.ConvertRealWorldToProjective(1, &current, &proj);
    return proj;
}
void DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2) {
    if(!g_UserGenerator.GetSkeletonCap().IsTracking(player)) return;
    
    XnSkeletonJointPosition joint1, joint2;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);
    
    if(joint1.fConfidence <= 0 || joint2.fConfidence <= 0) return;
    
    XnPoint3D pt[2] = {joint1.position, joint2.position};
        
    g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
    glVertex3f(pt[0].X, pt[0].Y, pt[0].Z);
    glVertex3f(pt[1].X, pt[1].Y, pt[1].Z);
}

void renderLumen() {
    if(firstRender) {
        // init quadric object
        quadric = gluNewQuadric();
        gluQuadricNormals(quadric, GLU_SMOOTH);
        
        // init camera and its texture
        #ifdef LUMEN_CAMERA
        initCamera();
        #endif
        #ifdef LUMEN_TRACKER
        initTracker();
        #endif
    }
    
    #ifdef LUMEN_CAMERA
    updateCamera();
    renderCamera();
    #endif
    
    // user interface
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 640, 480, 0, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if(currentUser == -1) {
        glColor4f(1,0,0,1); //else glColor3f(0,1,0);
    } else {
        glColor4f(rr,gg,bb,1); //else glColor3f(0,1,0);
    }
    if(drawSquare) {
        glBegin(GL_QUADS);
        glVertex3f(5,50, 0.0);
        glVertex3f(50,50, 0.0);
        glVertex3f(50,5, 0.0);
        glVertex3f(5,5, 0.0);
        glEnd();
    }

    //renderTrackPad();

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    // lines and body
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(25, (640/480+0.0), 1.0, 5000.0);
    glScalef(-1,1,1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    #ifdef LUMEN_TRACKER
    if(trackerInit) {
        glRotatef((Yaw->get()-initYaw)/7,0,1,0);
        glRotatef((Roll->get()-initRoll)/7,0,0,1);
        glRotatef((Pitch->get()-initPitch)/7,1,0,0);
    }
    #endif
    if(headpos != NULL) {
    gluLookAt(headpos->X(),headpos->Y(),headpos->Z(),    // camera position
              headpos->X(),headpos->Y(),headpos->Z()-10, // look-at vector
              0.0,-1.0,0.0);// up vector 
    }

    if(doClear) {
        doClear = false;
        drawingLine = false;
        lines.Clear();
    }
    
    XnUserID aUsers[15];
    XnUInt16 nUsers = 15;
    g_UserGenerator.GetUsers(aUsers, nUsers);

    //for(int i = 0; i < nUsers; i++) {
    if(nUsers>0 && currentUser>=0 && currentUser<=nUsers) {
        int i = currentUser-1;
        XnPoint3D hand = GetLimbPosition(aUsers[i], XN_SKEL_RIGHT_HAND);
        if(firstUser) {
            lastPosition = new SmoothPoint(hand, 7, 2);
            lastPositionProj = new SmoothPoint(getProj(hand), 7, 2);
        } else {
            lastPosition->insert(hand);
            lastPositionProj->insert(getProj(hand));
        }
        
        if(drawingLine==false && isUsingMouse==true && isMouseDown==true) { // line start
            //start new line
            drawingLine = true;
            currentLine = Line(rr,gg,bb, currentBrush);
            
            if(lastPosition->X()!=0 && lastPosition->Y()!=0 && lastPosition->Z()!=0) {
                currentLine.linePoints.Add(lastPosition->get(), lastPositionProj->get());
            }
            
            printf("line begin %d (%1.2f,%1.2f,%1.2f)\n", lines.Count(), lastPosition->X(),lastPosition->Y(),lastPosition->Z());
        } else if(drawingLine==true && isUsingMouse==true && isMouseDown==false) { // line end
            drawingLine = false;
            
            printf("line end (%1.2f,%1.2f,%1.2f)\n", lastPosition->X(),lastPosition->Y(),lastPosition->Z());
            currentLine.compileLine();
            lines.Add(currentLine);
        } else if(drawingLine) { // line mid
            currentLine.linePoints.Add(lastPosition->get(), lastPositionProj->get());
        }
        
        // render lines
        for(int l = 0; l < lines.Count(); l++) lines[l].renderLine();
        if(drawingLine) currentLine.renderLine();
        
        if(drawSkeleton) {
            glBegin(GL_LINES);
            glColor4f(1,1,1,1);

            DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
            DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
            glEnd();
        }

        XnPoint3D head = getProj(GetLimbPosition(aUsers[i], XN_SKEL_HEAD));
        if(firstUser) {
            headpos = new SmoothPoint(head, 7, 2);
        } else {
            headpos->insert(head);
        }
        firstUser = false;
    }
    firstRender = false;
}
