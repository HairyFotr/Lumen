#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

int main () {
    ifstream file;
    file.open("/dev/hidraw0", ios::in|ios::binary);
    usleep(100*1000);
    if(file.is_open()) {
        // Unbuffered reading 
        file.rdbuf()->pubsetbuf(0, 0);

        // 42-byte blocks
        char* block = new char[42];
        
        int cnt = 0;
        while(true) { 
            file.read(block, 42);
            cnt++;
            if(cnt%4==0) {
                int16_t* block16 = (int16_t*)block;
                
                printf("%d %d %d %d %d %d %d %d %d %d %d %d\n", 
                    block16[1], block16[2], block16[3],     // Magnetometer 
                    block16[4], block16[5], block16[6],     // Accelerometer?
                    block16[7], block16[8], block16[9],     // Gyro1?
                    block16[10], block16[11], block16[12]); // Gyro2?
                
                fflush(stdout);
            }
        }
        file.close();
    } else {
        printf("Could not open device");
    }
    return 0;
}
