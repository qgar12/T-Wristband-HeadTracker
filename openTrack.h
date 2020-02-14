#ifndef _OPEN_TRACK_H
#define _OPEN_TRACK_H

#include <WiFi.h>
#include <WiFiUdp.h>

const int openTrackServerPort = 4242;								
const char* openTrackServerIpStr = "192.168.0.54";

WiFiUDP UDP;

struct OpenTrackPackage {
    double x = 0;
    double y = 0;
    double z = 0;
    double yaw = 0;
    double pitch = 0;
    double roll = 0;
};

void openTrackSend(OpenTrackPackage pack) {

    UDP.beginPacket(openTrackServerIpStr, openTrackServerPort);
    UDP.write((byte *)&pack,sizeof pack);		
    UDP.endPacket();

}

#endif
