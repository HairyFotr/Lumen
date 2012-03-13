#include <iostream>
#include <iomanip>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

int main(int argc, char* argv[]) {
	if(argc == 1) {
        fprintf(stderr, "No device set\n");
		return 1;
	}
    ifstream file;
    file.open(argv[1], ios::in|ios::binary);
    //usleep(100*1000);
    if(file.is_open()) {
        // Unbuffered reading 
        file.rdbuf()->pubsetbuf(0, 0);

        // 42-byte blocks
        char* block = new char[42];
        time_t timestamp;
        
        int cnt = 0;
        while(true) {
            file.read(block, 42);
			time(&timestamp);
            int16_t* block16 = (int16_t*)block;
            
            printf("%ld %d %d %d %d %d %d %d %d %d %d %d %d\n", 
            	timestamp,
                block16[1], block16[2], block16[3],     // Magnetometer 
                block16[4], block16[5], block16[6],     // Accelerometer
                block16[7], block16[8], block16[9],     // Low sensitivity Gyro
                block16[10], block16[11], block16[12]); // High sensitivity Gyro
            
            fflush(stdout);
        }
        file.close();
    } else {
        fprintf(stderr, "Could not open device\n");
        return 2;
    }
    return 0;
}
