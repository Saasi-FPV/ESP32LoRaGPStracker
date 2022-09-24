#include <GPS.h>

//Init of the GPS HardwearSerial
void GPS::init(){
    Serial1.begin(GPSBAUD, SERIAL_8N1, RXD2, TXD2);
    //Serial1.begin(GPSBAUD);
}

//Called every loop to refresh gps Struct
void GPS::loop(){
    while(Serial1.available() > 0){
        if (gps.encode(Serial1.read())){
        }
    }
}

//returns latitude as float in Decimal degrees
float GPS::getlat(){
    if(gps.location.isValid()){
        return gps.location.lat();
    }else {
        return false;
    }
}

//returns longitude as float in Decimal degrees
float GPS::getlon(){
    if(gps.location.isValid()){
        return gps.location.lng();
    }else {
        return false;
    }
}

//returns numbers of fixed Satelites as int
int GPS::satellites(){
    int x = gps.satellites.value();
    if(x > 0)
        return x;
    else
        return false;
}