//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <ni/XnOpenNI.h>
#include <ni/XnCodecIDs.h>
#include <ni/XnCppWrapper.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include "LumenRender.h"
#include "Utils.h"
#include <GL/glut.h>
#include <map>
#include <time.h>

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::ScriptNode g_ScriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Player g_Player;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";

bool cancelLine = FALSE;
bool drawingLine = FALSE;
bool userIsLost = FALSE;
bool drawSkeleton = TRUE;
bool drawSquare = TRUE;
int quitRequested = 0;
bool headView = TRUE;
bool doClear = FALSE;
bool menuEnabled = FALSE;
bool menuEnabledInit = FALSE;
bool menuScrollUp = FALSE;
bool menuScrollDown = FALSE;
bool menuClick = FALSE;
float menuFadeIn = 0.0;
bool isMouseDown = FALSE;
bool isUsingMouse = TRUE;
int g_TestVar = -2;
int brushCount = 3;
float currentThickness=0.9;
int currentBrush = 0;
int testNum = 0;

XnUInt32 currentUser = -1;


//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    printf("New User %d\n", nId);
    // New user found
    if(g_bNeedPose) 
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    else
        g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    printf("Lost user %d\n", nId);
    if(nId == currentUser) {
        currentUser = -1;
        userIsLost = true;
        
        /*XnUserID aUsers[15];
        XnUInt16 nUsers = 15;
        g_UserGenerator.GetUsers(aUsers, nUsers);
        printf("Lost... There are now %d\n", nUsers);*/
    }
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie) {
    printf("Pose %s detected for user %d\n", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
    printf("Calibration started for user %d\n", nId);
    /*XnUserID aUsers[15];
    XnUInt16 nUsers = 15;
    g_UserGenerator.GetUsers(aUsers, nUsers);
    printf("Started... There are now %d\n", nUsers);*/
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
    if(bSuccess) {
        // Calibration succeeded
        printf("Calibration complete, start tracking user %d\n", nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);

        /*XnUserID aUsers[15];
        XnUInt16 nUsers = 15;
        g_UserGenerator.GetUsers(aUsers, nUsers);
        printf("Added... There are now %d\n", nUsers);*/
    } else {
        // Calibration failed
        printf("Calibration failed for user %d\n", nId);
        if(g_bNeedPose)
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        else
            g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie) {
    if(eStatus == XN_CALIBRATION_STATUS_OK) {
        // Calibration succeeded
        printf("Calibration complete, start tracking user %d\n", nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
        if(currentUser==-1) currentUser = nId;
    } else {
        // Calibration failed
        printf("Calibration failed for user %d\n", nId);
        if(g_bNeedPose)
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        else
            g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}

std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie) {
    m_Errors[id].first = calibrationError;
}
void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie) {
    m_Errors[id].second = poseError;
}

#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"

// Save calibration to file
void SaveCalibration() {
    XnUserID aUserIDs[20] = {0};
    XnUInt16 nUsers = 20;
    g_UserGenerator.GetUsers(aUserIDs, nUsers);
    for(int i = 0; i < nUsers; ++i) {
        // Find a user who is already calibrated
        if(g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) {
            // Save user's calibration to file
            g_UserGenerator.GetSkeletonCap().SaveCalibrationDataToFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
            break;
        }
    }
}
// Load calibration from file
void LoadCalibration() {
    XnUserID aUserIDs[20] = {0};
    XnUInt16 nUsers = 20;
    g_UserGenerator.GetUsers(aUserIDs, nUsers);
    for(int i = 0; i < nUsers; ++i) {
        // Find a user who isn't calibrated or currently in pose
        if(g_UserGenerator.GetSkeletonCap().IsCalibrated(aUserIDs[i])) continue;
        if(g_UserGenerator.GetSkeletonCap().IsCalibrating(aUserIDs[i])) continue;

        // Load user's calibration from file
        XnStatus rc = g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(aUserIDs[i], XN_CALIBRATION_FILE_NAME);
        if(rc == XN_STATUS_OK) {
            // Make sure state is coherent
            g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUserIDs[i]);
            g_UserGenerator.GetSkeletonCap().StartTracking(aUserIDs[i]);
        }
        break;
    }
}

void CleanupExit() {
    fprintf(stderr, "Lumen Cleanup\n");
    cleanupLumen();
    fprintf(stderr, "OpenNI Cleanup\n");
    g_ScriptNode.Release();
    g_Context.Release();
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Player.Release();

    exit(quitRequested);
}

bool kinectInit = false;

// this function is called each frame
void glutDisplay(void) {
    if(kinectInit) renderLumen();
}

void glutIdle(void) {
    if(quitRequested != 0) 
        CleanupExit(); // Exit
    else     
        glutPostRedisplay(); // Display the frame
}

void glutKeyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 'a' : testNum++; fprintf(stderr, "(%d)", testNum); break;
        case 's' : testNum--; fprintf(stderr, "(%d)", testNum); break;
        case  27: quitRequested = 1; break;
        //case 's': drawSkeleton= !drawSkeleton; break;
        //case 'y': drawSquare = !drawSquare; break;
        //case 'S': SaveCalibration(); break;
        //case 'L': LoadCalibration(); break;
        //case 'h': headView = !headView; break;
        //case '1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9': currentBrush = key-'1'; break;
        case 'c': doClear = true; break;
        //case 'm': isUsingMouse = !isUsingMouse; break;
    }
}

float rr=0,gg=0,bb=0,aa=0.75;
void processMouse(int button, int state, int x, int y) {
    if(state == GLUT_DOWN) {
        //printf("%d\n", button);
        if(button == GLUT_LEFT_BUTTON) {
            isMouseDown = TRUE;
            menuClick = TRUE;
            if(rr==0&&gg==0&&bb==0) randomColor(rr,gg,bb);
        } else if(button == GLUT_RIGHT_BUTTON) {
            if(drawingLine) {
                cancelLine = true;
            } else {
                menuEnabled = !menuEnabled;
                if(menuEnabled) {
                    menuFadeIn = 0;
                    menuEnabledInit = true;
                }
            }            
        } else if(button == GLUT_MIDDLE_BUTTON) {
            randomColor(rr,gg,bb);
        }
        else if(button == 4) {//UP
            menuScrollUp = TRUE;
            /*if(!menuEnabled) {
                currentThickness += 0.1;
                if(currentThickness > 2) {
                    currentThickness = 2;
                }
            }*/
        } else if(button == 3) {//DOWN
            menuScrollDown = TRUE;
           /*if(!menuEnabled) {
                currentThickness -= 0.1;
                if(currentThickness < 0.2) {
                    currentThickness = 0.2;
                }
            }*/
        }
        /*if(button == 8 || button == 7) {
            system("scrot");
        }*/
    } else {
        isMouseDown = FALSE;
    }
}

void glInit(int* pargc, char** argv) {
    glutInit(pargc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(1024, 768);
     glutCreateWindow ("Lumen");
    glutFullScreen();
    glutSetCursor(GLUT_CURSOR_NONE);
    //lumen init
    randomColor(rr,gg,bb);
    //currentBrush = randInt(brushCount);    

    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);
    glutKeyboardFunc(glutKeyboard);
    glutMouseFunc(processMouse);
    //glutMotionFunc(processMouseActiveMotion);
    //glutPassiveMotionFunc(processMousePassiveMotion);
    //glutEntryFunc(processMouseEntry);

    glEnable(GL_MULTISAMPLE);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    
    // Opacity
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Light
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat position[] = {150,385,1000, 0.0};
    GLfloat ambient[] = {0.1, 0.1, 0.1, 1};
    GLfloat diffuse[] = {1, 1, 1, 1};
    GLfloat specular[] = {0, 0, 0, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    //glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 20);

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    GLfloat mat[] = {1, 1, 1, 1};
    GLfloat black[] = {0, 0, 0, 0};
    //glMaterialfv(GL_FRONT, GL_DIFFUSE, mat);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat);
    glMaterialfv(GL_FRONT, GL_SPECULAR, black);
    glEnable(GL_COLOR_MATERIAL);
    
    /*glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mat);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE );*/

    glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);
    
    glEnable(GL_TEXTURE_2D);
}

#define SAMPLE_XML_PATH "OpenNIConfig.xml"

#define CHECK_RC(nRetVal, what)                                      \
    if(nRetVal != XN_STATUS_OK) {                                    \
        printf("%s failed: %s\n", what, xnGetStatusString(nRetVal)); \
        return nRetVal;                                              \
    }

   
void ctrlChandler(int s){
   printf("Caught signal %d\n",s);
   quitRequested = 1;
}

int main(int argc, char** argv) {
    srand(time(NULL));
    #define LUMEN_TRACKER
    #define LUMEN_TRACKER_USE
    #ifdef LUMEN_TRACKER
    initTracker();
    #endif
    
    fprintf(stderr, "Init OpenGL.\n");
    glInit(&argc, argv);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSwapBuffers();    

    XnStatus nRetVal = XN_STATUS_OK;
    
    if(argc > 1) {
        nRetVal = g_Context.Init();
        CHECK_RC(nRetVal, "Init");
        nRetVal = g_Context.OpenFileRecording(argv[1], g_Player);
        if(nRetVal != XN_STATUS_OK) {
            printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
            return 2;
        }
    } else {
        xn::EnumerationErrors errors;
        nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_ScriptNode, &errors);
        if(nRetVal == XN_STATUS_NO_NODE_PRESENT) {
            XnChar strError[1024];
            errors.ToString(strError, 1024);
            printf("%s\n", strError);
            return (nRetVal);
        } else if(nRetVal != XN_STATUS_OK) {
            printf("Open failed: %s\n", xnGetStatusString(nRetVal));
            return (nRetVal);
        }
    }
    
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");
    /////nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
    /////CHECK_RC(nRetVal, "Find image generator");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if(nRetVal != XN_STATUS_OK) {
        nRetVal = g_UserGenerator.Create(g_Context);
        CHECK_RC(nRetVal, "Find user generator");
    }

    XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
    if(!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        printf("Supplied user generator doesn't support skeleton\n");
        return 2;
    }
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
    CHECK_RC(nRetVal, "Register to calibration start");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
    CHECK_RC(nRetVal, "Register to calibration complete");

    if(g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = FALSE;
        if(!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
            printf("Pose required, but not supported\n");
            return 2;
        }
        nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
        CHECK_RC(nRetVal, "Register to Pose Detected");
        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);

    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
    CHECK_RC(nRetVal, "Register to calibration in progress");

    nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
    CHECK_RC(nRetVal, "Register to pose in progress");

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    kinectInit = true;

    fprintf(stderr, "Kinect init done.\n");

    //catch ctrl c
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = ctrlChandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    
    fprintf(stderr, "Going into main loop.\n");
        
    glutMainLoop();    
}


