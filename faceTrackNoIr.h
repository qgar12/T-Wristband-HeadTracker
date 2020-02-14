#ifndef _FACE_TRACK_NO_IR_H_
#define _FACE_TRACK_NO_IR_H_

#include <WiFi.h>
#include <WiFiUdp.h>

const int faceTrackServerPort = 5550;								
const char* faceTrackServerIpStr = "192.168.0.54";

WiFiUDP fUDP;

long frameNb = 0;

// see http://facetracknoir.sourceforge.net/Trackers/UDP.htm
struct FaceTrackPackage {
    double x = 0;
    double y = 0;
    double z = 0;
    double yaw = 0;
    double pitch = 0;
    double roll = 0;
    long _frameNb = 0;
};

void faceTrackSend(FaceTrackPackage pack) {
    pack._frameNb = frameNb++;
    fUDP.beginPacket(faceTrackServerIpStr, faceTrackServerPort);
    fUDP.write((byte *)&pack,sizeof pack);		
    fUDP.endPacket();

}

#endif
