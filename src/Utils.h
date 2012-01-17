#ifndef LUMEN_UTILS
#define LUMEN_UTILS
    #include <time.h>

    clock_t getTime();
    clock_t getTimeSince(clock_t sincetime);

    float rand01();
    int randInt(int lim);
    void randomColor(float& r, float& g, float& b);

#endif
