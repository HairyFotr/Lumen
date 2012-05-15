//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#define LUMEN_CAMERA // use camera
//#define LUMEN_BACKDROP // use backdrop.jpg tracker instead of camera
#define LUMEN_TRACKER // Wrap 920AR tracker 
#define LUMEN_TRACKER_USE // use the tracker

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
#include <time.h>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ni/XnOpenNI.h>
#include <ni/XnCppWrapper.h>
#include <boost/thread.hpp>
using namespace std;

extern xn::Context g_Context;
extern xn::UserGenerator g_UserGenerator;
extern xn::DepthGenerator g_DepthGenerator;

extern time_t clickTimer;

extern int testNum;
extern int quitRequested;
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
        if(frame.data) {
            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, cameraTextureID);
            if(frame.channels() == 3)
                glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGB, size.width,size.height, 0,GL_BGR,GL_UNSIGNED_BYTE, frame.data);
            else if(frame.channels() == 4)
                glTexImage2D(GL_TEXTURE_RECTANGLE_ARB,0,GL_RGBA, size.width,size.height, 0,GL_BGRA,GL_UNSIGNED_BYTE, frame.data);
        }
        camThread.join();
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
    
    glScalef(1.35,1.35,1);
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

#ifdef LUMEN_TRACKER
float initPitch, initYaw, initRoll;
SmoothData *a, *b, *c;

char* trackerBlock = new char[42];
int16_t* trackerBlock16 = (int16_t*)trackerBlock;
double* normalBlock = new double[12];

double HiGyroMulti = (1/4.333333333333)*(M_PI/180.0);

#define GRAVITY 256

double multi[12] = {
    3.7037037037037037/1000, 3.3003300330033003/1000, 3.389830508474576/1000, 
    1*GRAVITY,1*GRAVITY,1*GRAVITY, 
    1,1,1, 
    HiGyroMulti,HiGyroMulti,HiGyroMulti
};
double offsets[12] = {
    100.0, -363.0, -5.0, 
    0,-40,0, 
    -31,22,-16, 
    -87,+40,-30
};

//WE SWAP SOME AXIS RELATIONSHIPS HERE
//TO GET IN LINE WITH AHRS CODE
// negative acc x and y
// negative gyro x then swap x and z
double getMagX() { return normalBlock[0]; }
double getMagY() { return normalBlock[1]; }
double getMagZ() { return normalBlock[2]; }

double getAccX() { return -normalBlock[3]; }
double getAccY() { return -normalBlock[4]; }
double getAccZ() { return normalBlock[5]; }

double getLoGyroX() { return -normalBlock[8]; }
double getLoGyroY() { return normalBlock[7]; }
double getLoGyroZ() { return -normalBlock[6]; }

double getHiGyroX() { return -normalBlock[11]; }
double getHiGyroY() { return normalBlock[10]; }
double getHiGyroZ() { return -normalBlock[9]; }

bool trackerInit = false;
bool trackerLock = false;
boost::thread trackerThread;
ifstream tracker;


void dt(timespec start, timespec end, timespec* dif) {
	if ((end.tv_nsec-start.tv_nsec)<0) {
		dif->tv_sec = end.tv_sec-start.tv_sec-1;
		dif->tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		dif->tv_sec = end.tv_sec-start.tv_sec;
		dif->tv_nsec = end.tv_nsec-start.tv_nsec;
	}
}

////// vector math

//Computes the dot product of two vectors
double Vector_Dot_Product(double vector1[3],double vector2[3])
{
  double op=0;
  
  for(int c=0; c<3; c++)
  {
  op+=vector1[c]*vector2[c];
  }
  
  return op; 
}

//Computes the cross product of two vectors
void Vector_Cross_Product(double vectorOut[3], double v1[3],double v2[3])
{
  vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
  vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
  vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar. 
void Vector_Scale(double vectorOut[3],double vectorIn[3], double scale2)
{
  for(int c=0; c<3; c++)
  {
   vectorOut[c]=vectorIn[c]*scale2; 
  }
}

void Vector_Add(double vectorOut[3],double vectorIn1[3], double vectorIn2[3])
{
  for(int c=0; c<3; c++)
  {
     vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(double a[3][3], double b[3][3],double mat[3][3])
{
  double op[3]; 
  for(int x=0; x<3; x++)
  {
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
      {
       op[w]=a[x][w]*b[w][y];
      } 
      mat[x][y]=0;
      mat[x][y]=op[0]+op[1]+op[2];
      
      double test=mat[x][y];
    }
  }
}




/////

double G_Dt=0;

// Euler angles
double roll=0;
double pitch=0;
double yaw=0;

double MAG_Heading;

double Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
double Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
double Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
double Omega_P[3]= {0,0,0};//Omega Proportional correction
double Omega_I[3]= {0,0,0};//Omega Integrator
double Omega[3]= {0,0,0};

double errorRollPitch[3]= {0,0,0};

double errorYaw[3]= {0,0,0};

double DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{	
    0,0,1  }
}; 
double Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


double Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

double GL_Matrix[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
double GL_MatrixT[16]={1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};

void Compass_Heading() {
  double MAG_X;
  double MAG_Y;
  double cos_roll;
  double sin_roll;
  double cos_pitch;
  double sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
  /*c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
  c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
  c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;*/
  
  // Tilt compensated Magnetic filed X:
  MAG_X = getMagX()*cos_pitch+getMagY()*sin_roll*sin_pitch+getMagZ()*cos_roll*sin_pitch;
  // Tilt compensated Magnetic filed Y:
  MAG_Y = getMagY()*cos_roll-getMagZ()*sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-MAG_Y,MAG_X);
  
  //if(MAG_Heading == nan) quitRequested = true; 
}

void Normalize(void)
{
  double error=0;
  double temporary[3][3];
  double renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/

double constrain(double x, double a, double b){
    double res = x;
    res = max(a,res);
    res = min(b,res);
    
    return res;
}

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
//#define Kp_YAW 1.2
#define Kp_YAW 3.5
#define Ki_YAW 0.00002

void Drift_correction(void)
{
  double mag_heading_x;
  double mag_heading_y;
  double errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static double Scaled_Omega_P[3];
  static double Scaled_Omega_I[3];
  double Accel_magnitude;
  double Accel_weight;
  
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector TODOxxx
  //Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  //Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
}
*/
/**************************************************/

void Matrix_update(void) {
  Gyro_Vector[0]=getHiGyroX(); //gyro x roll
  Gyro_Vector[1]=getHiGyroY(); //gyro y pitch
  Gyro_Vector[2]=getHiGyroZ(); //gyro Z yaw
  
  /*printf("gyro: %f %f %f\n", 
    Gyro_Vector[0]/G_Dt,
      Gyro_Vector[1]/G_Dt,
        Gyro_Vector[2]/G_Dt
  );*/
  
  Accel_Vector[0]=getAccX();
  Accel_Vector[1]=getAccY();
  Accel_Vector[2]=getAccZ();
    
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

void Euler_angles(void)
{
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);

//  	printf("!ANG:%f,%f,%f\n",roll,pitch,yaw);	  
}

void GLMatrix(void)
{
 GL_Matrix[0]=DCM_Matrix[0][0];
 GL_Matrix[4]=DCM_Matrix[0][1];
 GL_Matrix[8]=DCM_Matrix[0][2];
 GL_Matrix[12]=0;

 GL_Matrix[1]=DCM_Matrix[1][0];
 GL_Matrix[5]=DCM_Matrix[1][1];
 GL_Matrix[9]=DCM_Matrix[1][2];
 GL_Matrix[13]=0;

 GL_Matrix[2]=DCM_Matrix[2][0];
 GL_Matrix[6]=DCM_Matrix[2][1];
 GL_Matrix[10]=DCM_Matrix[2][2];    
 GL_Matrix[14]=0;

 GL_Matrix[3]=0;
 GL_Matrix[7]=0;
 GL_Matrix[11]=0;
 GL_Matrix[15]=1;

 GL_MatrixT[0]=DCM_Matrix[0][0];
 GL_MatrixT[1]=DCM_Matrix[0][1];
 GL_MatrixT[2]=DCM_Matrix[0][2];
 GL_MatrixT[3]=0;

 GL_MatrixT[4]=DCM_Matrix[1][0];
 GL_MatrixT[5]=DCM_Matrix[1][1];
 GL_MatrixT[6]=DCM_Matrix[1][2];
 GL_MatrixT[7]=0;

 GL_MatrixT[8]=DCM_Matrix[2][0];
 GL_MatrixT[9]=DCM_Matrix[2][1];
 GL_MatrixT[10]=DCM_Matrix[2][2];    
 GL_MatrixT[11]=0;

 GL_MatrixT[12]=0;
 GL_MatrixT[13]=0;
 GL_MatrixT[14]=0;
 GL_MatrixT[15]=1;

}

////////


timespec timer, timer_old, diff;


void updateTracker() {
    if(trackerInit){
        for(int i=0; i<12; i++) {
            normalBlock[i] = (trackerBlock16[i+1]+offsets[i])*multi[i]/1000.0;            
        }
        
        timer_old = timer;
        clock_gettime(CLOCK_REALTIME, &timer);
        dt(timer_old, timer, &diff);
        G_Dt = (diff.tv_sec + (diff.tv_nsec/1000000000.0));   // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
                
        // *** DCM algorithm
        Compass_Heading(); // Calculate magnetic heading  
        
        // Calculations...
        Matrix_update(); 
        Normalize();
        GLMatrix();
        Drift_correction();
        Euler_angles();
                // ***
    }
}
void readTracker() {
    if(trackerInit) {
        while(quitRequested == 0) {
            trackerLock=true;
            tracker.read(trackerBlock, 42);
            updateTracker();
            trackerLock=false;
        }
    }
}

// tracker.read blocks if no data.
boost::thread firstReadThread;
void firstRead() {
    if(tracker.is_open()) {
        tracker.rdbuf()->pubsetbuf(0, 0);
        tracker.read(trackerBlock, 42);
        if(trackerBlock16[0] == -32767) {
            clock_gettime(CLOCK_REALTIME, &timer);
            /*int smoothlen=10;
            a = new SmoothData(0,smoothlen,0);
            b = new SmoothData(0,smoothlen,0);
            c = new SmoothData(0,smoothlen,0);
            trackerInit = true;
            for(int i=0; i<smoothlen/2; i++) {
                tracker.read(trackerBlock, 42);
                updateTracker();
            }*/
            trackerInit = true;
            fprintf(stderr, "Tracker initialized.\n");
            trackerThread = boost::thread(readTracker);
        } else {
            fprintf(stderr, "Can't read from tracker: %d\n", trackerBlock16[0]);
            quitRequested = 2;
        }
    } else {
        fprintf(stderr, "Can't open tracker.\n");
        quitRequested = 2;
    }
}

void initTracker() {
    if(!trackerInit) {
        fprintf(stderr, "Tracker init started.\n");
        tracker.open("/dev/hidraw2", ios::in|ios::binary);
        #ifdef LUMEN_TRACKER_USE 
        firstReadThread = boost::thread(firstRead);
        #endif
    }
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
    return current;
    /*XnPoint3D proj = xnCreatePoint3D(3,3,3);
    g_DepthGenerator.ConvertRealWorldToProjective(1, &current, &proj);
    return proj;*/
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
void DrawLine(XnPoint3D p1, XnPoint3D p2) {
    float th = 0.7;
    glColor4f(1,1,1,1);
    glDisable(GL_LIGHTING);
    glBegin(GL_QUADS);
    glVertex3f(p1.X-th, p1.Y, p1.Z);
    glVertex3f(p1.X+th, p1.Y, p1.Z);
    glVertex3f(p2.X+th, p2.Y, p2.Z);
    glVertex3f(p2.X-th, p2.Y, p2.Z);
    glEnd();
    glEnable(GL_LIGHTING);
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

void cleanupLumen() {
    if(quitRequested==0) quitRequested = 1;
    #ifdef LUMEN_TRACKER
        fprintf(stderr, "Releasing tracker.\n");
        #ifdef LUMEN_TRACKER_USE
        trackerThread.join();
        #endif
        tracker.close();
    #endif
    #ifdef LUMEN_CAMERA
        fprintf(stderr, "Releasing camera.\n");
        camThread.join();
        capture.release();
    #endif
    //save();
}
/*
std::string timestr(time_t t) {
    std::stringstream strm;
    strm << "save/"
    strm << t;
    strm << ".lum"
    return strm.str();
}

void save() {
    ofstream f;
    f.open(timestr(time(0)));
    lines.toFile(f);
    f.close();
}
*/
XnPoint3D convertKinect(XnPoint3D in) {
    XnPoint3D out = { -in.Z, in.X, -in.Y };
    return out;
}
XnPoint3D convertGlasses(XnPoint3D in) {
    XnPoint3D out = { in.Y, -in.Z, -in.X };
    return out;
}

bool hadKinect = false;

float maxX=-10000, minX=+10000;
int cnt = 0;
void renderLumen() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    cnt++;
    if(firstRender) {
        fprintf(stderr, "Render init.\n");
        // init quadric object
        quadric = gluNewQuadric();
        gluQuadricNormals(quadric, GLU_SMOOTH);
     
        glDisable(GL_CULL_FACE);
        // init camera and its texture
        #ifdef LUMEN_CAMERA
        initCamera();
        #endif
        
        //menu squiggle
        //TODO:a se prvi "neprojectan" point sploh kdaj rabi
        menuSquiggle = new Line(rr,gg,bb,aa, currentBrush);
        
        Vec3 dummy = Vec3(0,0,0);
        float i=0, n=4;
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
        fprintf(stderr, "Render init done.\n");
    }
    
    #ifdef LUMEN_CAMERA
    updateCamera();
    renderCamera();
    #endif
   
    ///////
  /*  firstRender = false;
    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (640/480+0.0), 0.1, 500.0);
    glScalef(1,1,1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(0,0,0,
              2,0,0, // look-at vector
              //0,-1,0);
              0, 0, -1);// up vector 

    glMultMatrixd(GL_MatrixT);

    
//    glRotatef(roll*180/M_PI, 0,0,1);
//    glRotatef(pitch*180/M_PI, 1,0,0);
//    glRotatef((yaw+1.5)*180/M_PI, 0,-1,0);

    //glRotatef(cnt/10.0, 0,0,1);
    
    glPushMatrix();
    glScalef(0.2,0.2,0.2);
    glBegin(GL_QUADS);            
        glColor4f(0.25,0,0,1);
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

        glColor4f(0,0.25,0,1);
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

        glColor4f(0,0,0.25,1);
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
    glEnd();
    glPopMatrix();

    glBegin(GL_LINES);
    glColor4f(0.75,0,0,1);
    glVertex3d(-1, 0, 0);
    glVertex3d(1, 0, 0);
    glColor4f(0,0.75,0,1);
    glVertex3d(0, -1, 0);
    glVertex3d(0, 1, 0);
    glColor4f(0,0,1,1);
    glVertex3d(0, 0, -1);
    glVertex3d(0, 0, 1);

    glColor4f(1,0,0,1);
    glVertex3d(0, 0, 0);
    glVertex3d(getMagX(), getMagY(), getMagZ());

    glColor4f(0,1,0,1);
    glVertex3d(0, 0, 0);
    glVertex3d(getAccX(), getAccY(), getAccZ());
    glEnd();

    glutSwapBuffers();
    return;
    ////////////

//*/









    // lines and body
    /*glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(10, (640/480+0.0), 1.0, 5000.0);
    glScalef(-1,1,1);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    if(headpos != NULL) {
        #ifdef LUMEN_TRACKER_USE 
            glRotatef((yaw+1.5)*180/M_PI, 0,-1,0);
            //glRotatef(pitch*180/M_PI, 1,0,0);
            //glRotatef(roll*180/M_PI, 0,0,1);
            printf("%f %f %f \n", yaw, pitch, roll);
            gluLookAt(headpos->X(),headpos->Y(),headpos->Z(),    // camera position
                      headpos->X(),headpos->Y(),headpos->Z()-10, // look-at vector
                      0.0,-1.0,0.0);// up vector 
        #else
            gluLookAt(headpos->X(),headpos->Y(),headpos->Z(),    // camera position
                      headpos->X(),headpos->Y(),headpos->Z()-10, // look-at vector
                      0.0,-1.0,0.0);// up vector 
        #endif
    }*/

    //glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(10, (640/480+0.0), 0.1, 4000.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    gluLookAt(0,0,0,
              10,0,0, // look-at vector
              //0,-1,0);
              0, 0, -1);// up vector 
    
    glMultMatrixd(GL_MatrixT);
    glRotatef(188,0,0,1);
    glRotatef(-5,0,1,0);
    
    if(headpos != NULL) {
        glTranslatef(headpos->Z(), -headpos->X(), headpos->Y());
        //if(isMouseDown && lastPositionProj != NULL) {
        //printf("kin head: %f %f %f\n", -headpos->Z(), headpos->X(), -headpos->Y());
          //printf("kin  arm: %f %f %f\n", lastPositionProj->X(), lastPositionProj->Y(), lastPositionProj->Z());
        //}
    }
    //printf("matrix: %f %f %f %f \n", GL_MatrixT[0], GL_MatrixT[1], GL_MatrixT[2], GL_MatrixT[3]);

    /*glBegin(GL_QUADS);            
        glColor4f(0.25,0,0,0.85);

        glVertex3f(-1000, 90,    80);
        glVertex3f(-1000, -1200, 80);
        glVertex3f(-1000, -1200, 500);
        glVertex3f(-1000, 90,    500);

    glEnd();*/

    /*glPushMatrix();
    glScalef(20,20,20);
    glBegin(GL_QUADS);            
        glColor4f(0.25,0,0,0.85);
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

        glColor4f(0,0.25,0,0.85);
        // top
        glNormal3f( 0, 1, 0);
        glVertex3f( 1, 1,-1);
        glVertex3f(-1, 1,-1);
        glVertex3f(-1, 1, 1);
        glVertex3f( 1, 1, 1);
        // bottom 
        glNormal3f( 0,-1, 1);
        glVertex3f( 1,-1, 1);isMouseDown
        glVertex3f(-1,-1, 1);
        glVertex3f(-1,-1,-1);
        glVertex3f( 1,-1,-1);

        glColor4f(0,0,0.25,0.85);
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
    glEnd();
    glPopMatrix();*/

    if(doClear) {
        doClear = false;
        drawingLine = false;
        lines.Clear();
    }
    
    g_Context.WaitAnyUpdateAll();

    XnUserID aUsers[15];
    XnUInt16 nUsers = 15;
    g_UserGenerator.GetUsers(aUsers, nUsers);
    
    if(nUsers>0 && currentUser>=0 && currentUser<=nUsers) {
        int i = currentUser-1;
        XnPoint3D hand0 = GetLimbPosition(aUsers[i], XN_SKEL_RIGHT_HAND);
        XnPoint3D elbow0 = GetLimbPosition(aUsers[i], XN_SKEL_RIGHT_ELBOW);
        //hand0 = Vec3::makeLonger(elbow0, hand0, -100);
        hand0 = Vec3::makeLonger(elbow0, hand0, -115);

        XnPoint3D hand0proj = getProj(hand0);
        
        XnPoint3D hand = convertKinect(hand0);
        XnPoint3D handproj = convertKinect(hand0proj);
        XnPoint3D elbowproj = getProj(elbow0);
        elbowproj = convertKinect(elbowproj);

        if(firstUser) {
            lastPosition = new SmoothPoint(hand, 20, 1);
            lastPositionProj = new SmoothPoint(handproj, 20, 1);
        } else {
            lastPosition->insert(hand);
            lastPositionProj->insert(handproj);
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
                
                //printf("line begin %d (%1.2f,%1.2f,%1.2f)\n", lines.Count(), lastPosition->X(),lastPosition->Y(),lastPosition->Z());
            } else if(drawingLine==true && isUsingMouse==true && isMouseDown==false) { // line end
                drawingLine = false;
                
                //printf("line end (%1.2f,%1.2f,%1.2f)\n", lastPosition->X(),lastPosition->Y(),lastPosition->Z());
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
            DrawLine(lastPositionProj->get(), elbowproj);
            
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
            glColor4f(1,1,1,1);
            glBegin(GL_LINES);
            DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_SHOULDER);
            glEnd();
        }

        XnPoint3D head = getProj(GetLimbPosition(aUsers[i], XN_SKEL_HEAD));
        XnPoint3D sh1 = getProj(GetLimbPosition(aUsers[i], XN_SKEL_LEFT_SHOULDER));
        XnPoint3D sh2 = getProj(GetLimbPosition(aUsers[i], XN_SKEL_RIGHT_SHOULDER));
        if(firstUser) {
            headpos = new SmoothPoint(head, 50, 0);
            //printf("headpos (%1.2f,%1.2f,%1.2f)\n",  headpos->X(),headpos->Y(),headpos->Z());
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
            XnPoint3D lastPos = convertGlasses(lastPosition->get());
        
            if(menuEnabledInit) {
                menuEnabledInit = false;
                menuIsSelected = false;
                menuHandInit = lastPos;
                menuHandInit.X += 75;
                menuClick = false;
            }
            if(!menuIsSelected) {
                if(lastPos.X > menuHandInit.X+50) menuSelected = 2; 
                else if(lastPos.X < menuHandInit.X-50) menuSelected = 0; 
                else menuSelected = 1;

                if(menuClick) {
                    menuHandInit = lastPos;
                    menuIsSelected = true;
                    if(menuIsSelected==1) {
                        menuInitColorPoint = lastPos;
                        menuColorPoint = lastPos;
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
                    
                    
                    if(lastPos.Y > menuHandInit.Y+90) {
                        menuHandInit = lastPos;
                        prevBrush();
                    } else if(lastPos.Y < menuHandInit.Y-90) {
                        menuHandInit = lastPos;
                        nextBrush();
                    }                    
                    
                    if(menuClick) {
                        menuIsSelected = false;
                        menuHandInit = lastPos;
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
                        menuHandInit = lastPos;
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
                        menuColorPoint = lastPos;
                        XnPoint3D mov = {
                            (float)(-(menuInitColorPoint.X-menuColorPoint.X)/2.0),
                            (float)((menuInitColorPoint.Y-menuColorPoint.Y)/2.0),
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
                        menuHandInit = lastPos;
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
                        menuColorPoint = lastPos;
                        XnPoint3D mov = {
                            (float)(-(menuInitColorPoint.X-menuColorPoint.X)/1.5),
                            (float)((menuInitColorPoint.Y-menuColorPoint.Y)/1.5),
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
                        currentThickness=0.5+((mov.Y+78)/(78*2.0))*1.75;
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
        if(drawSquare) {
            int off = 3;
            glColor4f(0.1,0.1,0.1,0.8);
            glBegin(GL_QUADS);
            glVertex3f(10-off,70+off, 0.0);
            glVertex3f(70+off,70+off, 0.0);
            glVertex3f(70+off,10-off, 0.0);
            glVertex3f(10-off,10-off, 0.0);
            glEnd();

            glColor4f(rr,gg,bb,aa);
            glBegin(GL_QUADS);
            glVertex3f(10,70, 0.0);
            glVertex3f(70,70, 0.0);
            glVertex3f(70,10, 0.0);
            glVertex3f(10,10, 0.0);
            glEnd();
        }
        glColor4f(1,1,1,1);
        glEnable(GL_BLEND);
        
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        hadKinect = true;
    } else {
        if(hadKinect && time(0)-15 > clickTimer) {
            quitRequested = 1;
        } else if(!hadKinect && time(0)-45 > clickTimer) {
            quitRequested = 1;
        }
    }

    if(firstRender) {
        fprintf(stderr, "First render done.\n");
    }
    firstRender = false;
    glutSwapBuffers();
}
