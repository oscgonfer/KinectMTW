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
//#define PORT_OSC_DLIGHT 7000


struct sortClass {
    bool operator() (int i, int j) {return(i<j);}
} sortObject;

struct MyComparator
{
    const vector<float> & value_vector;
    
    MyComparator(const vector<float> & val_vec):
    value_vector(val_vec) {}
    
    bool operator()(int i1, int i2)
    {
        return value_vector[i1] < value_vector[i2];
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
    float boxWallDistanceX;
    float boxWallDistanceY;
    
    int boxPixelSize;
    int boxSize;
    float boxDistanceX;
    float boxDistanceY;

    #define totNumBox 4
    int minArea;
    
    int drawingAreaX;
    int drawingAreaY;
    int drawingPositionX;
    int drawingPositionY;
    
    double boxHorSize;
    double boxVerSize;

    float gridXR[totNumBox];
    float gridYR[totNumBox];
    float gridXL[totNumBox];
    float gridYL[totNumBox];

    int arrayRequesting[totNumBox][totNumBox][totNumBox];
    int arrayPlaying[totNumBox][totNumBox][totNumBox];
    float arrayTime[totNumBox][totNumBox][totNumBox];
    int arrayPotentialFade[totNumBox][totNumBox][totNumBox];
    float arrayLastTimePlayed[totNumBox][totNumBox][totNumBox];
    int layerGrid = 0;
    bool resetAll = false;
    int timePermanentCue = 5;
    #define maxPermanentCuesAtMax 4
    int maxPermanentCues = maxPermanentCuesAtMax + 4;
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
    
    
    // used for sending the osc messages to qlab
    ofxOscSender sender_QLAB;
    //ofxOscSender sender_DLIGHT;
    
    //SERIAL
    ofxSimpleSerial	Serial;
    ofSerialDeviceInfo SerialInfo;
    
    string		messageSerial;

    
};
