#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>


//RX TX verkehrt

#define RXD2 16
#define TXD2 17
#define RESET 4

#define GPSBAUD 9600

class GPS{

    private:
        TinyGPSPlus gps;


    public:
        void init();
        void loop();
        float getlat();
        float getlon();
        bool validPos();
        int satellites();
        void doSleep();
        void doWakeup();
};




#endif