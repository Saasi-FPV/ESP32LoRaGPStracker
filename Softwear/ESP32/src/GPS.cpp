#include <GPS.h>

//Init of the GPS HardwearSerial
void GPS::init(){
    Serial1.begin(GPSBAUD, SERIAL_8N1, RXD2, TXD2);
    pinMode(RESET, OUTPUT);
    doWakeup();
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

//returns if Position is Valid
bool GPS::validPos(){
    if(gps.location.isValid()){
        return 1;
    }
    else{
        return 0;
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

//Sends command to Sleep
void GPS::doSleep(){
    Serial1.println("$PMTK161,0*28<CR><LF>");
    delay(200);
    Serial1.println("$PMTK161,0*28<CR><LF>");
    delay(200);
}


//Sends command to wakeup
void GPS::doWakeup(){
    digitalWrite(RESET, LOW);
    delay(20);
    digitalWrite(RESET, HIGH);
}