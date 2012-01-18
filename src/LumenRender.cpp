//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#define LUMEN_CAMERA //use camera
//#define LUMEN_BACKDROP //use backdrop.jpg tracker instead of camera
#define LUMEN_TRACKER //can has the Wrap 920AR tracker 
#define LUMEN_TRACKER_USE //use the tracker

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
extern bool menuEnabled;
extern bool menuEnabledInit;
extern bool menuScrollUp;
extern bool menuScrollDown;
extern bool menuClick;
extern float menuFadeIn;
extern XnUInt32 currentUser;
extern bool isMouseDown;
extern bool isUsingMouse;
extern int currentBrush;
extern int brushCount;
extern float rr;
extern float gg;
extern float bb;
extern float aa;

GLUquadricObj* quadric;

#ifdef LUMEN_TRACKER
float initPitch, initYaw, initRoll;
SmoothData *a, *b, *c;

char* trackerData = new char[42];
bool trackerInit = false;
bool trackerLock = false;
boost::thread trackerThread;
ifstream tracker;

union mix_t {
  int16_t i;
  struct {
    char a;
    char b;
  } s;
} uni;
int getInt16(char* pos) {
    uni.s.a = *(pos);
    uni.s.b = *(pos+1);
    int res = uni.i;
    return res;
}
union mix_t1 {
  int i;
  struct {
    char a;
    char b;
    char c;
    char d;
  } s;
} aaa;

int getInt(char* pos) {
    aaa.s.c = *(pos);
    aaa.s.d = *(pos+1);
    aaa.s.a = *(pos+2);
    aaa.s.b = *(pos+3);
    int res = aaa.i;
    return res;
}

float notnormal(int a, int min, int max) {
    return (((a-min)/1000.0)*360.0)/((max-min)/1000.0)-180.0;
}

float getVal(char* pos, int min, int max) {
    return notnormal(getInt(pos), min, max);
}

float lastok=0;
float straightYaw() { //looks ok if head isn't tilted by pitch/roll
    if(a->get()>0) {
        return (1.0*a->get()+1.5*(b->get()-150)+0.05*c->get());                    
    } else {
        return (1.0*a->get()+1.5*(b->get()-150)+0.05*c->get());                    
    }
}
float getYaw() {
    if(fabs(b->get()-150) < 5) {
        fprintf(stderr, "%2.2f", b->get()-150);
        lastok = straightYaw();
        return lastok;
    } else {
        int weight = 20;
        return (lastok*weight+straightYaw())/(weight+1.0);
    }
}

float getYawVal(char* pos)   { return getVal(pos, -28966489, 10944869); }
float getPitchVal(char* pos) { return getVal(pos, -24903612, 19529627); }
float getRollVal(char* pos)  { return getVal(pos, -98828161, 79757133); }

void updateTracker() {
    if(trackerInit) {
        a->insert(getYawVal(trackerData+2+0));
        b->insert(getPitchVal(trackerData+2+4));
        c->insert(getRollVal(trackerData+2+8));
    }
}
void readTracker() {
    if(trackerInit) {
        while(true) {
            trackerLock=true;
            tracker.read(trackerData, 42);
            updateTracker();
            trackerLock=false;
            //usleep(10*1000);
        }
    }
}

// yes I know this is horrible - fileRead blocks if no data.
boost::thread firstReadThread;
void firstRead() {
    tracker.read(trackerData, 42);
    if(getInt16(trackerData)==-32767) {
        int smoothlen=50;
        a = new SmoothData(getYawVal(trackerData+2+0),smoothlen,0);
        b = new SmoothData(getPitchVal(trackerData+2+4),smoothlen,0);
        c = new SmoothData(getRollVal(trackerData+2+8),smoothlen,0);
        trackerInit = true;
        for(int i=0; i<smoothlen; i++) {
            tracker.read(trackerData, 42);
            updateTracker();
        }
        //initPitch = Pitch->get();
        initYaw = getYaw();//Yaw->get();
        //initRoll = Roll->get();
        trackerThread = boost::thread(readTracker);
    }
}
void initTracker() {
    tracker.open("/dev/hidraw0", ios::in|ios::binary);
    usleep(1000*1000);
    if(tracker.is_open()) {
        #ifdef LUMEN_TRACKER_USE 
        tracker.rdbuf()->pubsetbuf(0, 0);
        firstReadThread = boost::thread(firstRead);
        #endif
    } else {
        fprintf(stderr, "Can't read from tracker");
        exit(1);
    }
}
#endif

#ifdef LUMEN_CAMERA
GLuint makeTexture(int width, int height, int channels, const GLvoid *data) {
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    GLuint texID;
    glGenTextures(1, &texID);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texID);

    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_RECTANGLE_ARB, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glDisable(GL_TEXTURE_RECTANGLE_ARB);

    if(channels == 3)
        glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGB, width,height, 0,GL_BGR,GL_UNSIGNED_BYTE, data);
    else if(channels == 4)
        glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGBA, width,height, 0,GL_BGRA,GL_UNSIGNED_BYTE, data);
        
    return texID;
}
cv::Mat makeTexture(string fileName, GLuint &texID) {
    cv::Mat mat = cv::imread(fileName);
    texID = makeTexture(mat.size().width, mat.size().height, mat.channels(), mat.data);
    return mat;
}

cv::VideoCapture capture;
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
    capture.open(-1);
    capture >> frame;
    #endif
    if(frame.data) {
        size = frame.size();        
        cameraTextureID = makeTexture(size.width, size.height, frame.channels(), frame.data);
        cameraInit = true;
    }
}
void readCamera() {
    if(cameraInit==true) {
        #ifdef LUMEN_BACKDROP
        #else
        capture >> frame;
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
            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraTextureID);
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
    
    glScalef(1.4,1.4,1.4);
    glTranslatef(-60,-105,0);
    
    glColor4f(1,1,1,1);
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
SmoothPoint *shoulderLeft, *shoulderRight;
SmoothPoint *lastPosition, *lastPositionProj;
extern bool drawingLine;
extern bool cancelLine;
extern float currentThickness;

Line currentLine;
Lines lines;

Line *menuSquiggle;
XnPoint3D menuHandInit;
int menuSelected = -1;
GLuint menuColorPic;
cv::Mat menuColorPicData;
XnPoint3D menuInitColorPoint;
XnPoint3D menuColorPoint;
GLuint menuCheckersPic;
cv::Mat menuCheckersPicData;

bool menuIsSelected = false;
void prevBrush() {
    currentBrush -= 1; 
    if(currentBrush<0) currentBrush = brushCount-1;
}
void nextBrush() {
    currentBrush += 1;
    currentBrush %= brushCount;
}


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

/*SmoothPoint *hand1=NULL, *hand2=NULL;
void DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2) {
    if(!g_UserGenerator.GetSkeletonCap().IsTracking(player)) return;
    
    XnSkeletonJointPosition joint1, joint2;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);
    
    if(joint1.fConfidence <= 0 || joint2.fConfidence <= 0) return;
    
    XnPoint3D pt[2] = {joint1.position, joint2.position};
    
    g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
    
    Vec3::makeLonger(pt[0], pt[1], 20);
    if(hand1==NULL) {
        hand1 = new SmoothPoint(pt[0], 12,1);
        hand2 = new SmoothPoint(pt[1], 12,1);
    } else {
        hand1->insert(pt[0]);
        lastPositionProj->insert(pt[1]);
    }
    
    glVertex3f(hand1->X(), hand1->Y(), hand1->Z());
    glVertex3f(lastPositionProj->X(), lastPositionProj->Y(), lastPositionProj->Z());
    
    /*glVertex3f(pt[0].X, pt[0].Y, pt[0].Z);
    glVertex3f(pt[1].X, pt[1].Y, pt[1].Z);*/
//}
void DrawLimb2(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2) {
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
void DrawLine(XnPoint3D p1, XnPoint3D p2) {
    glVertex3f(p1.X, p1.Y, p1.Z);
    glVertex3f(p2.X, p2.Y, p2.Z);
}

void drawQuad(float x, float y) {
    glBegin(GL_QUADS);
        glVertex3f(-x,-y, 0);
        glVertex3f(-x,+y, 0);
        glVertex3f(+x,+y, 0);
        glVertex3f(+x,-y, 0);
    glEnd();    
}
void drawQuad(float x1, float y1, float x2, float y2) {
    glBegin(GL_QUADS);
        glVertex3f(x1,y1, 0);
        glVertex3f(x1,y2, 0);
        glVertex3f(x2,y2, 0);
        glVertex3f(x2,y1, 0);
    glEnd();
}
void drawTexQuad(float x, float y, int size, GLuint texID) {
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, texID);
        
    glBegin(GL_QUADS);
    glTexCoord2i(0,size);    glVertex3f(-x,+y, 0);
    glTexCoord2i(size,size); glVertex3f(+x,+y, 0);
    glTexCoord2i(size,0);    glVertex3f(+x,-y, 0);
    glTexCoord2i(0,0);       glVertex3f(-x,-y, 0);
    glEnd();

    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glDisable(GL_TEXTURE_2D);
}

void drawArrow(float size) {
    glScalef(1.1*size,4*size,1*size);
    // 25x20
    glBegin(GL_QUADS);
        glVertex3f(0,0, 0);
        glVertex3f(-10,0, 0);
        glVertex3f(-25,-10, 0);
        glVertex3f(-15,-10, 0);
    glEnd();
    glBegin(GL_QUADS);
        glVertex3f(-25,-10, 0);
        glVertex3f(-10,-20, 0);
        glVertex3f(0,-20, 0);
        glVertex3f(-15,-10, 0);
    glEnd();
}
void drawArrow() {
    drawArrow(1);
}

bool fexists(const char *filename) {
    ifstream ifile(filename);
    return ifile;
}

/*void saveData() {
    lines
}*/

void cleanupLumen() {
    #ifdef LUMEN_CAMERA
    capture.release();
    #endif
    #ifdef LUMEN_TRACKER
    tracker.close();
    #endif
}

float maxX=-10000, minX=+10000;

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
        
        //menu
        //squiggle
        //TODO:a se prvi "neprojectan" point sploh kdaj rabi
        menuSquiggle = new Line(rr,gg,bb,aa, currentBrush);
        
        Vec3 dummy = Vec3(0,0,0);
        float i=0, n=4; //yarly :P
        menuSquiggle->linePoints.Add(dummy,{n*i++,+0.0f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,+0.7f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,+1.0f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,+0.6f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,+0.0f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,-0.7f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,-1.0f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,-0.6f*n,0});
        menuSquiggle->linePoints.Add(dummy,{n*i++,+0.1f*n,0});
        
        menuColorPicData = makeTexture("colorscale.png", menuColorPic);
        menuCheckersPicData = makeTexture("checkerboard.png", menuCheckersPic);
    }
    
    #ifdef LUMEN_CAMERA
    updateCamera();
    renderCamera();
    #endif
    
    // lines and body
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(10, (640/480+0.0), 1.0, 5000.0);
    glScalef(-1,1,1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    if(headpos != NULL) {
        #define LUMEN_NORMALHEAD
        #ifdef LUMEN_NORMALHEAD
        gluLookAt(headpos->X(),headpos->Y(),headpos->Z(),    // camera position
                  headpos->X(),headpos->Y(),headpos->Z()-10, // look-at vector
                  0.0,-1.0,0.0);// up vector 
        #else
        Vec3 shVec = (shoulderLeft->getVec3() - shoulderRight->getVec3());
        shVec = shVec.rotate90();
        shVec.normalize();
        //shVec *= 1;

        Vec3 headvec = headpos->getVec3()+shVec;

        if(rand01()>0.9) printf("[%1.2f, %1.2f]\n", shVec.x, shVec.z);

        gluLookAt(headpos->X(),headpos->Y(),headpos->Z(),    // camera position
                  headpos->X(),headpos->Y(),headpos->Z()+shVec.z, // look-at vector
                  0.0,-1.0,0.0);// up vector 
        #endif
        #ifdef LUMEN_TRACKER
        //glRotatef((getYaw()-initYaw)/10,0,1,0);
        #endif


    //if(headpos != NULL) {
        /*glTranslatef(headpos->X(),headpos->Y(),headpos->Z());

        if(trackerInit) {
            //fprintf(stderr, "%f\t%f\t%f", (Yaw->get()-initYaw), (Roll->get()-initRoll), (Pitch->get()-initPitch));


            glRotatef((Yaw->get()-initYaw)/2250000.0,1,0,0);
            glRotatef((Pitch->get()-initPitch)/2250000.0,0,1,0);
            glRotatef((Roll->get()-initRoll)/2250000.0,0,0,1);
        }

        glTranslatef(-headpos->X(),-headpos->Y(),-headpos->Z());
        /*#ifdef LUMEN_TRACKER
        if(trackerInit) {
            //fprintf(stderr, "%f\t%f\t%f", (Yaw->get()-initYaw), (Roll->get()-initRoll), (Pitch->get()-initPitch));


            glRotatef((Yaw->get()-initYaw)/2250000.0,1,0,0);
            glRotatef(-(Roll->get()-initRoll)/2250000.0,0,0,1);

            glRotatef((Pitch->get()-initPitch)/2250000.0,0,1,0);
        }
        #endif*/
        /*gluLookAt(0,0,0,    // camera position

                headpos->X(),headpos->Y(),headpos->Z(),
                  //261.32,256.29,3314.46,
                  //headpos->X(),headpos->Y(),headpos->Z()-10, // look-at vector
                  0.0,-1.0,0.0);// up vector */
        //glRotatef((Roll->get()-initRoll)/2250000.0,0,0,1);
        /*XnPoint3D shL = shoulderLeft->get();
        XnPoint3D shR = shoulderRight->get();
        Vec3 turn = Vec3( //shoulder vector

            shL.X-shR.X,
            shL.Y-shR.Y,
            shL.Z-shR.Z
        );

        turn = Vec3( //forward vector
            -turn.z,
            0,
            turn.x

        );
        turn.normalize();
        turn*=10;
        

        if(menuClick) fprintf(stderr, "[%f,%f]", turn.x, turn.z);
        */
        
        //fprintf(stderr, "[%f]\n", getYaw());
        //glRotatef((getYaw())/3,0,1,0);
            //}        
            /*gluLookAt(headpos->X(),headpos->Y(),headpos->Z(),
                  0,200,0,
                  //headpos->X()+getYaw(),headpos->Y(),headpos->Z(),
                  //headpos->X()+sin(getYaw()*3*M_PI/180.0),200,headpos->Z()+cos(getYaw()*3*M_PI/180.0),
                  0.0,-1.0,0.0);
        
/*        gluLookAt(headpos->X(),headpos->Y(),headpos->Z(),
                  0,200,0,
                  //headpos->X()+getYaw(),headpos->Y(),headpos->Z(),
                  //headpos->X()+sin(getYaw()*3*M_PI/180.0),200,headpos->Z()+cos(getYaw()*3*M_PI/180.0),
                  0.0,-1.0,0.0);
//        glRotatef(getYaw()/5,0,1,0);
        
        
//        fprintf(stderr, "[%f]\n", getYaw());
        /*fprintf(stderr, "[%3.2f  %3.2f  %3.2f  |  %f  %f  %f]\n", 
            headpos->X(),headpos->Y(),headpos->Z(),
            (headpos->X()+sin(getYaw()*3*M_PI/180.0)),200,(headpos->Z()+cos(getYaw()*3*M_PI/180.0)));*/
        //if(isMouseDown) glRotatef(getYaw(),1,0,0);
        
        //glRotatef((Pitch->get()-initPitch)/2250000.0,0,1,0);
        //glRotatef((Roll->get()-initRoll)/22500000.0,0,0,1);
        /*
        Line *l = new Line(rr,gg,bb,aa, currentBrush);        
        Vec3 dummy = Vec3(0,0,0);
        l->linePoints.Add(dummy,{100.52,-256.42,1765.34});
        l->linePoints.Add(dummy,{-153.22,-283.74,1852.76});
        l->renderLine();

        Line *l2 = new Line(rr,gg,bb,aa, currentBrush);        
        l2->linePoints.Add(dummy,{-163.88,-52.57,2291.42});
        l2->linePoints.Add(dummy,{-237.98,-406.69,2358.03});
        l2->renderLine();

        Line *l3 = new Line(rr,gg,bb,aa, currentBrush);        
        l3->linePoints.Add(dummy,{-250.17,-494.26,2449.86});
        l3->linePoints.Add(dummy,{-333.52,-245.33,2227.22});
        l3->renderLine();*/
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
        XnPoint3D elbow = GetLimbPosition(aUsers[i], XN_SKEL_RIGHT_ELBOW);
        hand = Vec3::makeLonger(elbow, hand, -100);
        if(firstUser) {
            lastPosition = new SmoothPoint(hand, 15, 1);
            lastPositionProj = new SmoothPoint(getProj(hand), 15, 1);
        } else {
            lastPosition->insert(hand);
            lastPositionProj->insert(getProj(hand));
        }

        /*if(headpos != NULL) {
            if(headpos->X()>maxX) maxX = headpos->X();
            if(headpos->X()<minX) minX = headpos->X();
            printf("(%1.2f,%1.2f,%1.2f)(%1.2f,%1.2f)\n", headpos->X(),headpos->Y(),headpos->Z(), minX, maxX);
        }*/

        if(!menuEnabled) {
            if(drawingLine==false && isUsingMouse==true && isMouseDown==true) { // line start
                //start new line
                drawingLine = true;
                currentLine = Line(rr,gg,bb,aa, currentBrush);
                
                if(lastPosition->X()!=0 && lastPosition->Y()!=0 && lastPosition->Z()!=0) {
                    currentLine.linePoints.Add(lastPosition->get(), lastPositionProj->get(), currentThickness);
                }
                
                printf("line begin %d (%1.2f,%1.2f,%1.2f)\n", lines.Count(), lastPosition->X(),lastPosition->Y(),lastPosition->Z());
            } else if(drawingLine==true && isUsingMouse==true && isMouseDown==false) { // line end
                drawingLine = false;
                
                printf("line end (%1.2f,%1.2f,%1.2f)\n", lastPosition->X(),lastPosition->Y(),lastPosition->Z());
                currentLine.compileLine();
                lines.Add(currentLine);
            } else if(drawingLine && cancelLine) { // cancel line
                isMouseDown = false;
                drawingLine = false;
            } else if(drawingLine) { // line mid
                currentLine.linePoints.Add(lastPosition->get(), lastPositionProj->get(), currentThickness);
            }
        }
        if(cancelLine) cancelLine = false;
        
        // render lines
        for(int l = 0; l < lines.Count(); l++) lines[l].renderLine();
        if(drawingLine) currentLine.renderLine();
        
        if(drawSkeleton) {
            glColor4f(1,1,1,1);
            glBegin(GL_LINES);
                DrawLine(lastPositionProj->get(), getProj(elbow));
                //DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
            glEnd();
            
            glColor4f(rr,gg,bb,aa);
            glTranslatef(lastPositionProj->X(), lastPositionProj->Y(), lastPositionProj->Z());
            switch(currentBrush) {
                case 0: {
                    float size = 3.5;
                    gluSphere(quadric, size*currentThickness, 15, 15); break;
                } case 1: {
                    glBegin(GL_QUADS);
                    //Vec3 n = cross(vecB); n.normalize();
                    //glNormal3f(n.x, n.y, n.z);
                    float size = 4;
                    glVertex3f(+size*currentThickness, 0, 0);
                    glVertex3f(-size*currentThickness, 0, 0);
                    glVertex3f(-size*currentThickness, 0, 0);
                    glVertex3f(+size*currentThickness, 0, 0);
                    glEnd();
                    break;
               } case 2: {
                    float size = 3;
                    glScalef(size*currentThickness, size*currentThickness, size*currentThickness);
                    glBegin(GL_QUADS);            
                         // top
                        glNormal3f( 0, 1, 0);
                        glVertex3f( 1, 1,-1);
                        glVertex3f(-1, 1,-1);
                        glVertex3f(-1, 1, 1);
                        glVertex3f( 1, 1, 1);
                        // bottom 
                        glNormal3f( 0,-1, 1);
                        glVertex3f( 1,-1, 1);
                        glVertex3f(-1,-1, 1);
                        glVertex3f(-1,-1,-1);
                        glVertex3f( 1,-1,-1);
                        // front
                        glNormal3f( 0, 0, 1);
                        glVertex3f( 1, 1, 1);
                        glVertex3f(-1, 1, 1); 
                        glVertex3f(-1,-1, 1);
                        glVertex3f( 1,-1, 1);
                        // back
                        glNormal3f( 0, 0,-1);
                        glVertex3f( 1,-1,-1);
                        glVertex3f(-1,-1,-1);
                        glVertex3f(-1, 1,-1);
                        glVertex3f( 1, 1,-1);
                        // left
                        glNormal3f(-1, 0, 0);
                        glVertex3f(-1, 1, 1);
                        glVertex3f(-1, 1,-1);
                        glVertex3f(-1,-1,-1);
                        glVertex3f(-1,-1, 1);
                        // right
                        glNormal3f( 1, 0, 0);
                        glVertex3f( 1, 1,-1);
                        glVertex3f( 1, 1, 1);
                        glVertex3f( 1,-1, 1);
                        glVertex3f( 1,-1,-1);
                    glEnd();
                    break; 
                }
            }
            glTranslatef(-lastPositionProj->X(), -lastPositionProj->Y(), -lastPositionProj->Z());
            //put this inside a method, you silly person
/*            XnSkeletonJointPosition joint;
            g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HAND, joint);
            if(joint.fConfidence > 0) {
                XnPoint3D pt = joint.position;
                
                g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

            }*/
            glColor4f(1,1,1,1);
            glBegin(GL_LINES);
            DrawLimb2(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_SHOULDER);
            glEnd();
        }

        XnPoint3D head = getProj(GetLimbPosition(aUsers[i], XN_SKEL_HEAD));
        XnPoint3D sh1 = getProj(GetLimbPosition(aUsers[i], XN_SKEL_LEFT_SHOULDER));
        XnPoint3D sh2 = getProj(GetLimbPosition(aUsers[i], XN_SKEL_RIGHT_SHOULDER));
        if(firstUser) {
            headpos = new SmoothPoint(head, 18, 1);
            printf("headpos (%1.2f,%1.2f,%1.2f)\n",  headpos->X(),headpos->Y(),headpos->Z());
            shoulderLeft = new SmoothPoint(sh1, 20, 1);
            shoulderRight = new SmoothPoint(sh2, 20, 1);
        } else {
            headpos->insert(head);
            shoulderLeft->insert(sh1);
            shoulderRight->insert(sh2);
        }
        firstUser = false;
    

        // user interface
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, 640, 480, 0, -1.0, 1.0);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        // menu
        //
        //
        //
        //
        //
        // yes yes... horrible and handcoded :)
        if(menuEnabled) {
            if(menuEnabledInit) {
                menuEnabledInit = false;
                menuIsSelected = false;
                menuHandInit = lastPosition->get();
                menuHandInit.X += 75;
                menuClick = false;
            }
            if(!menuIsSelected) {
                if(lastPosition->get().X > menuHandInit.X+50) menuSelected = 2; 
                else if(lastPosition->get().X < menuHandInit.X-50) menuSelected = 0; 
                else menuSelected = 1;

                if(menuClick) {
                    menuHandInit = lastPosition->get();
                    menuIsSelected = true;
                    if(menuIsSelected==1) {
                        menuInitColorPoint = lastPosition->get();
                        menuColorPoint = lastPosition->get();
                    }
                    menuClick = false;
                }
            }
            
            glColor4f(0,0,0,0.75*menuFadeIn);
            drawQuad(0,0,640,480);
            
            // helpful arrows
            if(menuIsSelected) {
                if(menuSelected==0) {
                    glPushMatrix();
                        glPushMatrix();
                            glColor4f(1,1,1, 0.8);
                            glTranslatef(80, 240-90, 0);
                            glRotatef(90,0,0,1);
                            drawArrow();
                        glPopMatrix();            
                        glPushMatrix();
                            glColor4f(1,1,1, 0.8);
                            glTranslatef(80, 240+90, 0);
                            glRotatef(90,0,0,1);
                            glScalef(-1,1,1);
                            drawArrow();
                        glPopMatrix();            
                    glPopMatrix();
                }
                if(menuSelected>=1) {
                    glPushMatrix();
                    if(menuSelected==2) glTranslatef(200,0,0);
                    glPushMatrix();
                        glPushMatrix();
                            glColor4f(1,1,1, 0.8);
                            glTranslatef(80+200, 240-90, 0);
                            glRotatef(90,0,0,1);
                            drawArrow();
                        glPopMatrix();            
                        glPushMatrix();
                            glColor4f(1,1,1, 0.8);
                            glTranslatef(80+200, 240+90, 0);
                            glRotatef(90,0,0,1);
                            glScalef(-1,1,1);
                            drawArrow();
                        glPopMatrix();            
                    glPopMatrix();
                    glPushMatrix();
                        glColor4f(1,1,1, 0.75);
                        glTranslatef(35+200, 240+40, 0);
                        drawArrow();
                    glPopMatrix();            
                    glPushMatrix();
                        glColor4f(1,1,1, 0.75);
                        glTranslatef(35+200+160+10, 240+40, 0);
                        glScalef(-1,1,1);
                        drawArrow();
                    glPopMatrix();            
                    glPopMatrix();
                }
            } else {
                glPushMatrix();
                    glColor4f(1,1,1, 0.75);
                    glTranslatef(35, 240+40, 0);
                    drawArrow();
                glPopMatrix();            
                glPushMatrix();
                    glColor4f(1,1,1, 0.75);
                    glTranslatef(620-15, 240+40, 0);
                    glScalef(-1,1,1);
                    drawArrow();
                glPopMatrix();            
            }

            glPushMatrix();
                // brush select
                if(menuIsSelected && menuSelected==0) {
                    if(menuScrollUp) {
                        prevBrush();
                        menuScrollUp = false;
                    }
                    if(menuScrollDown) {
                        nextBrush();
                        menuScrollDown = false;
                    }
                    
                    
                    if(lastPosition->get().Y > menuHandInit.Y+90) {
                        menuHandInit = lastPosition->get();
                        prevBrush();
                    } else if(lastPosition->get().Y < menuHandInit.Y-90) {
                        menuHandInit = lastPosition->get();
                        nextBrush();
                    }                    
                    
                    if(menuClick) {
                        menuIsSelected = false;
                        menuHandInit = lastPosition->get();
                        menuHandInit.X += 75;
                        menuClick = false;
                    }
                }
                
                glTranslatef(120,0,0);
                glPushMatrix();
                    glTranslatef(0,240,0);
                    if(menuSelected==0 && menuIsSelected) 
                        glColor4f(1,1,1,0.75*menuFadeIn);
                    else if(menuSelected==0) 
                        glColor4f(0.5,0.5,0.5,0.75*menuFadeIn);
                    else
                        glColor4f(0,0,0,0.75*menuFadeIn);
                        
                    drawQuad(80,80);
                    
                    //Spooky hardcoded territory
                    glEnable(GL_BLEND);
                    glEnable(GL_LIGHTING);
                    glEnable(GL_DEPTH_TEST);
                    glMatrixMode(GL_PROJECTION);
                    glPushMatrix();
                    glLoadIdentity();
                    gluPerspective(25, (640/480+0.0), 1.0, 5000.0);
                    glScalef(-1,1,1);
                    glMatrixMode(GL_MODELVIEW);
                    glPushMatrix();
                    glLoadIdentity();
                    
                    glTranslatef(22.25,-5,-225);
                    glScalef(0.7,1,1);
                    glRotatef(42,0,1,0);
                    glRotatef(33,1,0,0);
                    glRotatef(16,0,0,1);
                    menuSquiggle->setColor(rr,gg,bb,aa);
                    menuSquiggle->renderLine(currentBrush, currentThickness);
                    
                    glPopMatrix();                                
                    glMatrixMode(GL_PROJECTION);
                    glPopMatrix();
                    glMatrixMode(GL_MODELVIEW);
                    glDisable(GL_DEPTH_TEST);
                    glDisable(GL_LIGHTING);
                    glDisable(GL_BLEND);
                glPopMatrix();
                
                // color select
                if(menuIsSelected && menuSelected==1) {
                    if(menuClick) {                        
                        menuIsSelected = false;
                        menuHandInit = lastPosition->get();
                        menuClick = false;
                    }
                }
                glPushMatrix();
                    glTranslatef(200,240,0);
                    glEnable(GL_BLEND);
                    glColor4f(1,1,1,1*menuFadeIn);
                    drawTexQuad(80,80,512,menuColorPic);

                    if(menuSelected==1 && menuIsSelected) 
                        glColor4f(1,1,1,0);
                    else if(menuSelected==1) 
                        glColor4f(0.5,0.5,0.5,0.75*menuFadeIn);
                    else
                        glColor4f(0,0,0,0.75*menuFadeIn);
                    drawQuad(80,80);
                    glDisable(GL_BLEND);
                    if(menuIsSelected && menuSelected==1) {
                        menuColorPoint = lastPosition->get();
                        XnPoint3D mov = {
                            -(menuInitColorPoint.X-menuColorPoint.X)/2.0,
                            (menuInitColorPoint.Y-menuColorPoint.Y)/2.0,
                            0
                        };
                        if(fabs(mov.X)>78) mov.X = 78*copysign(1,mov.X);
                        if(fabs(mov.Y)>78) mov.Y = 78*copysign(1,mov.Y);

                        int x=(int)((mov.X+78)*(512/(160.0-4)));//map boxsize to texturesize
                        int y=(int)((mov.Y+78)*(512/(160.0-4)));
                        if(x<0) x=0; if(x>=512) x=512-1;//clamp
                        if(y<0) y=0; if(y>=512) y=512-1;

                        int step = menuColorPicData.step;
                        int nChannels = menuColorPicData.channels();

                        float B = menuColorPicData.data[y*step+x*nChannels+0]/255.0;
                        float G = menuColorPicData.data[y*step+x*nChannels+1]/255.0;
                        float R = menuColorPicData.data[y*step+x*nChannels+2]/255.0;
                        rr=R; gg=G; bb=B;
                        
                        glTranslatef(mov.X,mov.Y,0);
                        glColor4f(0,0,0,0.5);
                        drawQuad(6,6);
                        glColor4f(R,G,B,1);
                        drawQuad(4,4);
                    }                
                glPopMatrix();
                
                // opacity/thickness select
                if(menuIsSelected && menuSelected==2) {
                    if(menuClick) {
                        menuIsSelected = false;
                        menuHandInit = lastPosition->get();
                        menuHandInit.X -= 75;
                        menuClick = false;
                    }
                }
                glPushMatrix();
                    glTranslatef(400,240,0);
                    glEnable(GL_BLEND);
                    glColor4f(1,1,1,1*menuFadeIn);
                    drawTexQuad(80,80,256,menuCheckersPic);

                    if(menuSelected==2 && menuIsSelected) 
                        glColor4f(0,0,0,0.5*menuFadeIn);
                    else if(menuSelected==2) 
                        glColor4f(0.5,0.5,0.5,0.75*menuFadeIn);
                    else
                        glColor4f(0,0,0,0.75*menuFadeIn);
                        
                    drawQuad(80,80);
                    
                    int x=80, y=80;
                    glBegin(GL_QUADS);
                        glColor4f(1,1,1,0);
                        glVertex3f(-x,-y, 0);
                        glVertex3f(-x,+y, 0);
                        glColor4f(1,1,1,0.5*menuFadeIn);
                        glVertex3f(+x,+y, 0);
                        glVertex3f(+x,-y, 0);
                    glEnd();//*/
                    glDisable(GL_BLEND);

                    if(menuIsSelected && menuSelected==2) {
                        menuColorPoint = lastPosition->get();
                        XnPoint3D mov = {
                            -(menuInitColorPoint.X-menuColorPoint.X)/1.5,
                            (menuInitColorPoint.Y-menuColorPoint.Y)/1.5,
                            0
                        };
                        if(fabs(mov.X)>78) mov.X = 78*copysign(1,mov.X);
                        if(fabs(mov.Y)>78) mov.Y = 78*copysign(1,mov.Y);
                        glTranslatef(mov.X,mov.Y,0);
                        glColor4f(0,0,0,0.5);
                        drawQuad(6,6);
                        glColor4f(1,1,1,1);
                        drawQuad(4,4);
                        aa=(mov.X+78)/(78*2.0);
                        if(aa < 0.15) aa=0.1;
                        currentThickness=0.2+((mov.Y+78)/(78*2.0))*1.75;
                    }
                    
                glPopMatrix();   
            glPopMatrix();
            if(menuFadeIn<0.9) menuFadeIn += 0.04; else menuFadeIn = 1;
        } else if(menuFadeIn > 0) {
            menuFadeIn -= 0.04;
            glColor4f(0,0,0,0.75*menuFadeIn);
            drawQuad(0,0,640,480);
        }
        //cancel unhandled "events"
        menuClick = false;
        menuScrollUp = false;
        menuScrollDown = false;        
        
        //glDisable(GL_BLEND);
        if(currentUser == -1) {
            glColor4f(1,0,0,1);
        } else {
            glColor4f(rr,gg,bb,aa);
        }
        if(drawSquare) {
            glBegin(GL_QUADS);
            glVertex3f(5,50, 0.0);
            glVertex3f(50,50, 0.0);
            glVertex3f(50,5, 0.0);
            glVertex3f(5,5, 0.0);
            glEnd();
        }
        glColor4f(1,1,1,1);
        glEnable(GL_BLEND);

        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
    }

    firstRender = false;
}
