#include <time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdlib.h>

clock_t getTime() { return clock(); } /// (CLOCKS_PER_SEC / 1000); }
clock_t getTimeSince(clock_t sincetime) { return getTime()-sincetime; }

float rand01() {
    return (float)rand()/(float)RAND_MAX;
}
int randInt(int lim) {
    return rand()%lim;
}

void randomColor(float& r, float& g, float& b) {
    r = rand01();
    g = rand01();
    b = rand01();
    if(r+b+g < 1 || (fabs(r-g)<0.125 && fabs(r-b)<0.125 && fabs(b-g)<0.125) ) randomColor(r,g,b);
}

