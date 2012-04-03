#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#include <string>
#include <map>
#include <vector>
#include <time.h>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;

cv::Mat splash;
GLuint splashID;


// this function is called each frame
void glutDisplay(void) {
}

void glutIdle(void) {
}

void processKeyboard(unsigned char key, int x, int y) {
    if(key==53) exit(1); // "5" :)
    if(key==27 || key==13) exit(0);
}

void processMouse(int button, int state, int x, int y) {
    if(state == GLUT_DOWN) exit(0);
}

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

int width = 1024;
int height = 768;

void renderSplash() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1.0, 1.0);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, splashID);

    glTranslatef(0,0,0);
    glColor4f(1,1,1,1);
    glBegin(GL_QUADS);
        glTexCoord2i(0,height);       glVertex3f(0,height, 0);
        glTexCoord2i(width,height);   glVertex3f(width,height, 0);
        glTexCoord2i(width,0);        glVertex3f(width,0, 0);
        glTexCoord2i(0,0);            glVertex3f(0,0, 0);
    glEnd();
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
}

void glInit(int* pargc, char** argv) {
    glutInit(pargc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(1024, 768);
    glutCreateWindow ("Lumen");
    glutFullScreen();
    glutSetCursor(GLUT_CURSOR_NONE);
    
    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);
    glutKeyboardFunc(processKeyboard);
    glutMouseFunc(processMouse);
    
    glEnable(GL_TEXTURE_2D);
}

int main(int argc, char** argv) {
    glInit(&argc, argv);

    splash = makeTexture("Lumen.png", splashID);
    renderSplash();
    glutSwapBuffers();
    
    glutMainLoop();
}


