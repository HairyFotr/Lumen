class SmoothData { //moving average
    int logLen;
    float* log;
    int logCurr;
    int lastWeight;
    bool update;
    float precomp;
public:
    SmoothData(float elt, int len, int last) { 
        logLen = len;
        lastWeight = last;
        log = new float[logLen];
        for(int i=0; i<logLen; i++) log[i] = elt;
        logCurr=0;
        precomp = elt;
        update = true;
        get();
    }
    
    
    void insert(float elt) {
        if(logLen > 0) {
            logCurr++;
            logCurr %= logLen;
            log[logCurr] = elt;
            update = true;
        }
    }
    
    float get() {
        if(update) {
            float res=0;
            if(logLen > 0) {
                for(int i=0; i<logLen; i++) res += log[i];
                res += log[logCurr]*lastWeight;
                res /= logLen+lastWeight;
            } else {
                res = 0;
            }
            precomp = res;
            update = false;
            return res;
        } else {
            return precomp;
        }
    }
    ~SmoothData() {
        delete [] log;
    }
};

class SmoothPoint {
    SmoothData *x,*y,*z;
public:
    SmoothPoint(XnPoint3D p, int len, int last) {
        x = new SmoothData(p.X, len,last);
        y = new SmoothData(p.Y, len,last);
        z = new SmoothData(p.Z, len,last);
    }
    float X() { return x->get(); }
    float Y() { return y->get(); }
    float Z() { return z->get(); }
    void insert(XnPoint3D elt) {
        x->insert(elt.X);
        y->insert(elt.Y);
        z->insert(elt.Z);
    }
    XnPoint3D get() { return {x->get(), y->get(), z->get()}; }
    Vec3 getVec3() { return Vec3(x->get(), y->get(), z->get()); }
};
