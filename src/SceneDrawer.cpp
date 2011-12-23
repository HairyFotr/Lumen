//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

//TODO
///Brushi - partikli
///Opacity - tekstura
///slika zakriva roko/sceno
    //kombinacija kinecta in kamer za dobivanje globine na kamera-sliki

#define LUMEN_CAMERA //use camera

#include "SceneDrawer.h"
#include "Geometry.cpp"
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
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
GLUquadricObj* quadric;
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
extern float rr;
extern float gg;
extern float bb;

// Drawing functions
void DrawLine(const XnPoint3D& ptMins, const XnPoint3D& ptMaxs, int width, double r = 1, double g = 1, double b = 1) {
    const GLubyte ind[2] = {0, 1};
    GLfloat verts[6] = { ptMins.X, ptMins.Y, ptMins.Z, ptMaxs.X, ptMaxs.Y, ptMaxs.Z };
    glColor4f(r,g,b,1.0f);
    glVertexPointer(3, GL_FLOAT, 0, verts);
    glLineWidth(width);
    glDrawArrays(GL_LINES, 0, 2);
    glFlush();
}

void DrawFrame(const XnPoint3D& ptMins, const XnPoint3D& ptMaxs, int width, double r, double g, double b) {
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
void DrowTrackPad() {
    if(!g_bInSession) return;
  
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
// end trackpad
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

XnPoint3D lastPosition, lastPositionProj;
bool drawing = false;

Line currentLine;
Lines lines;

bool lumenInit = false;

#ifdef LUMEN_CAMERA
cv::VideoCapture cap;
cv::Mat frame;
cv::Size size;
GLuint cameraTextureID;
boost::thread camThread;
bool cameraInit = false;
bool cameraLock = false;
void initCamera() {
    cap.open(-1);
    cap >> frame;
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

        cameraInit = true;
    }
}
void readCamera() {
    if(cameraInit==true) cap >> frame;
    cameraLock = false;
}
void updateCamera() {
    if(!cameraLock) {
        cameraLock = true;
        camThread.join();
        if(frame.data) {
            if(frame.channels() == 3)
                glTexImage2D( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGB, size.width, size.height, 0, GL_BGR, GL_UNSIGNED_BYTE, frame.data );
            else if(frame.channels() == 4)
                glTexImage2D( GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, size.width, size.height, 0, GL_BGRA, GL_UNSIGNED_BYTE, frame.data );            
        }
        camThread = boost::thread(readCamera);
    }
}
void renderCamera() {
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, size.width, size.height, 0, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraTextureID);

    glBegin(GL_QUADS);
    glTexCoord2i(0,size.height);          glVertex3f(0.0,size.height, 0.0);
    glTexCoord2i(size.width,size.height); glVertex3f(size.width,size.height, 0.0);
    glTexCoord2i(size.width,0);           glVertex3f(size.width,0.0, 0.0);
    glTexCoord2i(0,0);                    glVertex3f(0.0,0.0, 0.0);
    glEnd();
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glEnable(GL_DEPTH_TEST);
}
#endif

XnPoint3D headpos;

void RenderLumen() {
    if(!lumenInit) {
        lumenInit = true;
        
        // init quadric object
        quadric = gluNewQuadric();
        gluQuadricNormals(quadric, GLU_SMOOTH);

        // init camera and its texture
        #ifdef LUMEN_CAMERA
        initCamera();
        #endif
        
        // init head position
        headpos.X = 0;
        headpos.Y = 0;
        headpos.Z = 0;
    }
    
    #ifdef LUMEN_CAMERA
    updateCamera();
    renderCamera();
    #endif
    
    // user interface
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 640, 480, 0, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if(g_nCurrentUser == -1) {
        glColor4f(1,0,0,1); //else glColor3f(0,1,0);
    } else {
        glColor4f(rr,gg,bb,1); //else glColor3f(0,1,0);
    }
    glBegin(GL_QUADS);
    glVertex3f(5,50, 0.0);
    glVertex3f(50,50, 0.0);
    glVertex3f(50,5, 0.0);
    glVertex3f(5,5, 0.0);
    glEnd();

    //DrowTrackPad()

    glEnable(GL_DEPTH_TEST);

    // lines and body
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(25, (640/480+0.0), 1.0, 5000.0);
    glScalef(-1,1,1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    //if(g_bLookFromHead) {
    gluLookAt(headpos.X,headpos.Y,headpos.Z,    // camera position
              headpos.X,headpos.Y,headpos.Z-10, // look-at vector
              0.0,-1.0,0.0);                    // up vector 
    /*} else {
        gluLookAt(900.0,0.0,900.0,
                  0.0,0.0,0.0,
                  0.0,-1.0,0.0);
        glRotatef(rot,0,1,0);
        if(g_bRotate) rot += 1.0;
    }*/

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
           fabs(lastPosition.X-currentPosition.X) + 
           fabs(lastPosition.Y-currentPosition.Y) + 
           fabs(lastPosition.Z-currentPosition.Z)/15 > 35))) { // && currentPosition.Z < headpos.Z
            
            //start new line
            drawing = true;
            
            currentLine = Line(rr,gg,bb, currentBrush);
            
            if(lastPosition.X != 0 && lastPosition.Y != 0 && lastPosition.Z != 0) {
                currentLine.linePoints.Add(lastPosition, lastPositionProj);
            }
            currentLine.linePoints.Add(currentPosition, currentPositionProj);
            
            printf("line begin %d (%1.2f,%1.2f,%1.2f)\n", lines.Count(),lastPosition.X,lastPosition.Y,lastPosition.Z);
        } else if(joint.fConfidence > 0 && drawing==true && ((g_bUseMouse==true && g_bMouseDown==false) 
               ||(g_bUseMouse==false &&
                  fabs(lastPosition.X-currentPosition.X) + 
                  fabs(lastPosition.Y-currentPosition.Y) + 
                  fabs(lastPosition.Z-currentPosition.Z) < 20))) {
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

        headpos = GetLimbPosition(aUsers[i], XN_SKEL_HEAD);
    }
    glEnable(GL_TEXTURE_2D);
}
