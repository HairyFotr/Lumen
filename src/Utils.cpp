#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

clock_t getTime() { return clock() / (CLOCKS_PER_SEC / 1000); }
clock_t getTimeSince(clock_t sincetime) { return getTime()-sincetime; }

void randomColor(float& r, float& g, float& b) {
    r = (float)rand()/(float)RAND_MAX;
    g = (float)rand()/(float)RAND_MAX;
    b = (float)rand()/(float)RAND_MAX;
    if(r+b+g < 1 || (fabs(r-g)<0.125 && fabs(r-b)<0.125 && fabs(b-g)<0.125) ) randomColor(r,g,b);
}

