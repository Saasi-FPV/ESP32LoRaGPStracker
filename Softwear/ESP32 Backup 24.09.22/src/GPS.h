#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>

#define RXD2 16
#define TXD2 17

#define GPSBAUD 9600

class GPS{

    private:
        TinyGPSPlus gps;


    public:
        void init();
        void loop();
        float getlat();
        float getlon();
        int satellites();
};




#endif