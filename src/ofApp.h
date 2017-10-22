#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"
#include <sstream>
#include <string>

#define HOST "localhost"
#define PORT_OSC 53000

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
    bool bDrawPointCloud;
    
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

    int totNumBox;
    int minArea;
    
    int drawingAreaX;
    int drawingAreaY;
    int drawingPositionX;
    int drawingPositionY;
    
    double boxHorSize;
    double boxVerSize;
    
    bool isBox;
    bool inPosition;
    
    ofVec4f gridX;
    ofVec4f gridY;
    
    
    // used for viewing the point cloud
    ofEasyCam easyCam;
    
    ofxOscSender sender;
};
