//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <ni/XnOpenNI.h>
#include <ni/XnCodecIDs.h>
#include <ni/XnCppWrapper.h>
#include "LumenRender.h"
#include "Utils.h"
#include <GL/glut.h>
#include <map>
#include <time.h>

//---------------------------------------------------------------------------
// Trackpad
//---------------------------------------------------------------------------
#include <nite/XnVHandPointContext.h>
#include <nite/XnVSessionManager.h>
#include <nite/XnVSelectableSlider2D.h>

XnBool g_bActive = FALSE;
XnBool g_bIsInput = FALSE;
XnBool g_bInSession = FALSE;
XnBool g_bIsPushed = FALSE;
XnUInt32 g_nCurrentFrame = 0;

XnFloat g_fXValue = 0.5f;
XnFloat g_fYValue = 0.5f;

XnUInt32 g_nXIndex = 0;
XnUInt32 g_nYIndex = 0;

XnUInt32 g_TP_XDim = 4;
XnUInt32 g_TP_YDim = 9;

const XnUInt32 XN_MIN_X_DIM = 2;
const XnUInt32 XN_MAX_X_DIM = 12;
const XnUInt32 XN_MIN_Y_DIM = 2;
const XnUInt32 XN_MAX_Y_DIM = 12;
//
xn::Context g_Context;
xn::ScriptNode g_ScriptNode;
XnVSelectableSlider2D* g_pTrackPad = NULL;
XnVSessionManager* g_pSessionManager = NULL;

XnCallbackHandle g_nItemHoverHandle = NULL;
XnCallbackHandle g_nItemSelectHandle = NULL;
XnCallbackHandle g_nValueChangeHandle = NULL;

XnCallbackHandle g_nPrimaryCreateHandle = NULL;
XnCallbackHandle g_nPrimaryDestroyHandle = NULL;

XnUInt32 g_TrackPadHandle = 0;

XnBool g_isPrintItemHover = TRUE;
XnBool g_isPrintValueChange = FALSE;
XnBool g_isInputStarted = FALSE;

XnPoint3D CurrentItem;

void XN_CALLBACK_TYPE TrackPad_ValueChange(XnFloat fXValue, XnFloat fYValue, void* cxt) {
    if(TRUE == g_isPrintValueChange) printf("Value changed: %f, %f\n", fXValue, fYValue);

    g_fXValue = fXValue;
    g_fYValue = fYValue;
}
void XN_CALLBACK_TYPE TrackPad_ItemHover(XnInt32 nXItem, XnInt32 nYItem, void* cxt) {
    if(TRUE == g_isPrintItemHover) printf("Hover: %d,%d\n", nXItem, nYItem);

    if((TRUE == g_bIsPushed) && (CurrentItem.X != nXItem || CurrentItem.Y != nYItem)) {
        g_bIsPushed = FALSE;
        g_nCurrentFrame = 0;
    }

    CurrentItem.X = nXItem;
    CurrentItem.Y = nYItem;
}

void XN_CALLBACK_TYPE TrackPad_ItemSelect(XnInt32 nXItem, XnInt32 nYItem, XnVDirection eDir, void* cxt) {
    printf("Select: %d,%d (%s)\n", nXItem, nYItem, XnVDirectionAsString(eDir));
    g_bIsPushed = TRUE;
}

void XN_CALLBACK_TYPE TrackPad_PrimaryCreate(const XnVHandPointContext* cxt, const XnPoint3D& ptFocus, void* UserCxt) {
    printf("TrackPad input has started!!!, point ID: [%d] ", cxt->nID);
    printf("Starting point position: [%f],[%f],[%f]\n", cxt->ptPosition.X, cxt->ptPosition.Y, cxt->ptPosition.Z);
    g_isInputStarted = TRUE;
}

void XN_CALLBACK_TYPE TrackPad_PrimaryDestroy(XnUInt32 nID, void* UserCxt) {
    //printf("TrackPad input has stopped!!!\n");
    g_isInputStarted = FALSE;
}

void InitiateTrackPad() {
    printf("TrackPad initiated.\n");
    if(NULL != g_pTrackPad) {
      g_pTrackPad->SetItemCount(g_TP_XDim, g_TP_YDim);
    }
}

void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& ptFocus, void* UserCxt) {
    printf("TrackPad startsession.\n");
    g_bInSession = true;
}
void XN_CALLBACK_TYPE SessionEnd(void* UserCxt) {
    printf("TrackPad endsession.\n");
    g_bInSession = false;
}

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Player g_Player;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";

XnBool drawSkeleton = TRUE;
XnBool quitRequested = FALSE;
XnBool headView = TRUE;
XnBool doClear = FALSE;
XnBool isMouseDown = FALSE;
XnBool isUsingMouse = TRUE;
int g_TestVar = -2;
int currentBrush = 0;
XnUInt32 currentUser = -1;


//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
void CleanupExit() {
    //trackpad
    if(NULL != g_pTrackPad) {
        if(NULL != g_nItemHoverHandle) g_pTrackPad->UnregisterItemHover(g_nItemHoverHandle);
        if(NULL != g_nValueChangeHandle) g_pTrackPad->UnregisterValueChange(g_nValueChangeHandle);
        if(NULL != g_nItemSelectHandle) g_pTrackPad->UnregisterItemSelect(g_nItemSelectHandle);
        if(NULL != g_nPrimaryDestroyHandle) g_pTrackPad->UnregisterPrimaryPointDestroy(g_nPrimaryDestroyHandle);
        if(NULL != g_nPrimaryCreateHandle) g_pTrackPad->UnregisterPrimaryPointCreate(g_nPrimaryCreateHandle);
    }

    if(NULL != g_pSessionManager) {
        if(0 != g_TrackPadHandle) g_pSessionManager->RemoveListener(g_TrackPadHandle);
        delete g_pSessionManager;
        g_pSessionManager = NULL;
    }

    if(NULL != g_pTrackPad) {
        delete g_pTrackPad;
        g_pTrackPad = NULL;
    }

    //skeleton
    g_ScriptNode.Release();
    g_DepthGenerator.Release();
    /////g_ImageGenerator.Release();
    g_UserGenerator.Release();
    g_Player.Release();
    g_Context.Release();

    exit(1);
}

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
    if(nId == currentUser) currentUser = -1;
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
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
    if(bSuccess) {
        // Calibration succeeded
        printf("Calibration complete, start tracking user %d\n", nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
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
        currentUser = nId;
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

// this function is called each frame
void glutDisplay(void) {
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    g_Context.WaitAnyUpdateAll();
    g_pSessionManager->Update(&g_Context);
    
    renderLumen();
    
    glutSwapBuffers();
}

void glutIdle(void) {
    if(quitRequested) CleanupExit();

    // Display the frame
    glutPostRedisplay();
}

void glutKeyboard(unsigned char key, int x, int y) {
    switch (key) {
        case  27: quitRequested = true; break;
        case 's': drawSkeleton= !drawSkeleton; break;
        case 'S': SaveCalibration(); break;
        case 'L': LoadCalibration(); break;
        case 'h': headView = !headView; break;
        case '1':case '2':case '3':case '4':case '5':case '6':case '7':case '8':case '9': currentBrush = key-'1'; break;
        case 'c': doClear = true; break;
        case 'm': isUsingMouse = !isUsingMouse; break;
    }
}

float rr=0,gg=0,bb=0;
void processMouse(int button, int state, int x, int y) {
    if(state == GLUT_DOWN) {
        if (button == GLUT_LEFT_BUTTON) {
            isMouseDown = TRUE;
            if(rr==0&&gg==0&&bb==0) randomColor(rr,gg,bb);
        } else if (button == GLUT_RIGHT_BUTTON) {
            currentBrush = (currentBrush+1)%3;
        } else if (button == GLUT_MIDDLE_BUTTON) {
            randomColor(rr,gg,bb);
        }
        /*else if (button == 4) {//UP
            colori += (colori+1)%colorCount
        } else if if (button == 3) {//DOWN
            colori += (colori+1)%colorCount
        }*/
    } else {
        isMouseDown = FALSE;
    }
}

void glInit(int* pargc, char** argv) {
    glutInit(pargc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutInitWindowSize(640, 480);
    glutCreateWindow ("Lumen");
    glutFullScreen();
    glutSetCursor(GLUT_CURSOR_NONE);
    randomColor(rr,gg,bb);
    
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

#define SAMPLE_XML_PATH "../../SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)                                      \
    if(nRetVal != XN_STATUS_OK) {                                    \
        printf("%s failed: %s\n", what, xnGetStatusString(nRetVal)); \
        return nRetVal;                                              \
    }

int main(int argc, char** argv) {
    srand(time(NULL));
    XnStatus nRetVal = XN_STATUS_OK;
    
    if(argc > 1) {
        nRetVal = g_Context.Init();
        CHECK_RC(nRetVal, "Init");
        nRetVal = g_Context.OpenFileRecording(argv[1], g_Player);
        if(nRetVal != XN_STATUS_OK) {
            printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
            return 1;
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
    
    //trackpad
    g_pSessionManager = new XnVSessionManager();

    if(NULL == g_pSessionManager) {
        printf("Couldn't create PointTracker!! (out of memory)\n");
        CleanupExit();
    }

    if(NULL == g_pTrackPad) {
        g_pTrackPad = new XnVSelectableSlider2D(g_TP_XDim, g_TP_YDim);
    }
    if(NULL == g_pTrackPad) {
        printf("Couldn't create TrackPad!! (out of memory)\n");
        CleanupExit();
    }
    
    //skeleton    
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
        return 1;
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
            return 1;
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
    
    //trackpad
    // Initialize the point tracker
    XnStatus rc = g_pSessionManager->Initialize(&g_Context, "Wave,RaiseHand,Click", "Wave,RaiseHand,Click");
    if (rc != XN_STATUS_OK) {
        printf("Couldn't initialize the Session Manager: %s\n", xnGetStatusString(rc));
        CleanupExit();
    }
    g_pSessionManager->RegisterSession(NULL, &SessionStart, &SessionEnd, NULL);

    // Add TrackPad to the point tracker
    g_TrackPadHandle = g_pSessionManager->AddListener(g_pTrackPad);

    // Register for the Hover event of the TrackPad
    g_nItemHoverHandle = g_pTrackPad->RegisterItemHover(NULL, &TrackPad_ItemHover);
    // Register for the Value Change event of the TrackPad
    g_nValueChangeHandle = g_pTrackPad->RegisterValueChange(NULL, &TrackPad_ValueChange);
    // Register for the Select event of the TrackPad
    g_nItemSelectHandle = g_pTrackPad->RegisterItemSelect(NULL, &TrackPad_ItemSelect);

    // Register for Input Start event of the TrackPad
    g_nPrimaryCreateHandle = g_pTrackPad->RegisterPrimaryPointCreate(NULL, &TrackPad_PrimaryCreate);
    // Register for Input Stop event of the TrackPad
    g_nPrimaryDestroyHandle = g_pTrackPad->RegisterPrimaryPointDestroy(NULL, &TrackPad_PrimaryDestroy);

    glInit(&argc, argv);
    glutMainLoop();
}
