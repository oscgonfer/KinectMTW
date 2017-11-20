#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include "ofxSimpleSerial.h"

#define HOST "localhost"
#define PORT_OSC_QLAB 53000

using namespace std;

struct vectorCompare
{
    const vector<int> & value_vector;
    
    vectorCompare(const vector<int> & val_vec):
    value_vector(val_vec) {}
    
    bool operator()(int i1, int i2)
    {
        return value_vector[i1] > value_vector[i2];
    }
};

class ofApp : public ofBaseApp{

	public:
    
    void setup();
    void update();
    void draw();
    void exit();
        
    void drawPointCloud();
    
    void keyPressed(int key);


    ofxKinect kinect;
    
    ofxCvColorImage colorImg;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    ofxCvContourFinder contourFinder;
    
    bool bThreshWithOpenCV;
    
    int nearThreshold;
    int farThreshold;
    
    int angle;
    
    float roomSizeX;
    float roomSizeY;

    #define totNumBox 4
    int minArea = 350;
    
    int drawingAreaX;
    int drawingAreaY;
    int drawingPositionX;
    int drawingPositionY;


    float gridXR[totNumBox];
    float gridYR[totNumBox];
    float gridXL[totNumBox];
    float gridYL[totNumBox];

    int arrayRequesting[totNumBox][totNumBox][totNumBox];
    int arrayPlaying[totNumBox][totNumBox][totNumBox];
    int arrayTime[totNumBox][totNumBox][totNumBox];
    int arrayPotentialFade[totNumBox][totNumBox][totNumBox];
    float arrayLastTimePlayed[totNumBox][totNumBox][totNumBox];
    int arrayFadeOut6Sent[totNumBox][totNumBox][totNumBox];
    int arrayFadeOut12Sent[totNumBox][totNumBox][totNumBox];
    int arrayFadeOut18Sent[totNumBox][totNumBox][totNumBox];
    int arrayFadeOut25Sent[totNumBox][totNumBox][totNumBox];
    int layerGrid = 0;
    bool resetAll = false;
    int timePermanentCue = 5;
    #define maxPermanentCuesAtMax 2
    int maxPermanentCues = maxPermanentCuesAtMax + 2;
    int lengthVectorPermanentCuePrev = 1000;
    
    bool displayCal = true;
    bool mouseControl;
    bool fadedCues = false;
    
    bool gridRight = false;
    bool gridLeft = false;
    bool GridCal = false;
    bool VerticalCal = false;
    bool HorizontalCal = false;
    int indexGrid = 0;
    int timeElapsedMin = 1;
    
    
    // used for sending the osc messages to qlab
    ofxOscSender sender_QLAB;
    //ofxOscSender sender_DLIGHT;
    
    //SERIAL
    ofxSimpleSerial	Serial;
    ofSerialDeviceInfo SerialInfo;
    
    string		messageSerial;

    
};
