#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
    // open an outgoing connection to HOST:PORT
    sender.setup( HOST, PORT_OSC );
    
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();		// opens first available kinect
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 235;
    farThreshold = 229;
    
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // zero the KINECT - tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // BOX PARAMETERS AND AREA
    
    minArea = 30;
    boxPixelSize = 50;
    
    boxSize = 300;
    roomSizeX = 4000;
    roomSizeY = 3800;

    boxWallDistanceX = 800;
    boxWallDistanceY = 700;
    boxDistanceX = (roomSizeX-(totNumBox*boxSize+2*boxWallDistanceX))/(totNumBox-1);
    boxDistanceY = (roomSizeY-(totNumBox*boxSize+2*boxWallDistanceY))/(totNumBox-1);

    
    for (int n=0; n<totNumBox; n++) {
        for (int m=0; m<totNumBox; m++){
            for (int o =0; o<totNumBox; o++){
                arrayRequesting [n][m][o]=0;
                arrayPlaying [n][m][o]=0;
                arrayTime [n][m][o]=0;
                arrayPotentialFade [n][m][o]=0;
                arrayLastTimePlayed [n][m][o]=0;
            }
        }
    }
    
    displayCal = false;
    mouseControl = false;
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    ofBackground(100, 100, 100);
    
    kinect.update();
    
    // there is a new frame and we are connected
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThreshold, true);
            grayThreshFar.threshold(farThreshold,false);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        // update the cv images
        grayImage.flagImageChanged();
        
        // find contours which are between the size of 20 pixels and 1/2 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayImage, minArea, (kinect.width*kinect.height)/2, 20, false);
        
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    ofSetColor(255, 255, 255);
    
    // draw from the live kinect
    // depth IR
    
    // GRID DEFINITION
    float gridLineX = 0;
    float gridLineY = 0;
    double boxLeft = 0;
    double boxTop = 0;
    
    if(displayCal) {
        drawingAreaX = ofGetWidth();
        drawingAreaY = ofGetHeight();
        drawingPositionX = 0;
        drawingPositionY = 0;
    }else {
        drawingAreaX = 350;
        drawingAreaY = 250;
        drawingPositionX = 420;
        drawingPositionY = 270;
        
        kinect.drawDepth(10, 10, drawingAreaX, drawingAreaY);
        // RGB
        kinect.draw(drawingPositionX, 10, drawingAreaX, drawingAreaY);
        contourFinder.draw(drawingPositionX, 10, drawingAreaX, drawingAreaY);
        // depth contour
        grayImage.draw(10, drawingPositionY, drawingAreaX, drawingAreaY);
        contourFinder.draw(10, drawingPositionY, drawingAreaX, drawingAreaY);
    }
    
    //HORIZONTAL GRID
    ofSetColor(120,255,255);
    
    boxHorSize = boxSize/roomSizeX*drawingAreaX;
    boxVerSize = boxSize/roomSizeY*drawingAreaY;
    
    for (int auxX= 0; auxX < totNumBox; auxX+=1) {
        boxLeft = (boxWallDistanceX + (boxSize+boxDistanceX)*auxX)/roomSizeX;
        gridLineX = boxLeft*drawingAreaX+drawingPositionX;
        gridX [auxX] = gridLineX;
    }
    
    // VERTICAL GRID
    ofSetColor(190,0,255);
    
    for (int auxY = 0;  auxY< totNumBox; auxY++) {
        boxTop = (boxWallDistanceY+(boxSize+boxDistanceY)*auxY)/roomSizeY;
        gridLineY = boxTop*drawingAreaY+drawingPositionY;
        gridY [auxY] = gridLineY;
    }

    //BOX DETECTION
    ofColor c(255, 255, 255);
    ofColor backC(150, 150, 150);
    ofRectangle backR(drawingPositionX,drawingPositionY, drawingAreaX, drawingAreaY);
    ofSetColor(backC);
    ofDrawRectangle(backR);

    // DRAW GRID

    //HORIZONTAL GRID
    ofSetColor(120,255,255);
    
    for (int auxX= 0; auxX < totNumBox; auxX+=1) {
        ofDrawLine(gridX [auxX],drawingPositionY,gridX [auxX],drawingPositionY+drawingAreaY);
        ofDrawLine(gridX [auxX]+boxHorSize,drawingPositionY,gridX [auxX]+boxSize/roomSizeX*drawingAreaX,drawingPositionY+drawingAreaY);
    }
    
    // VERTICAL GRID
    ofSetColor(190,0,255);
    
    for (int auxY = 0;  auxY< totNumBox; auxY++) {
        ofDrawLine(drawingPositionX,gridY [auxY],drawingPositionX+drawingAreaX,gridY [auxY]);
        ofDrawLine(drawingPositionX,gridY [auxY]+boxVerSize,drawingPositionX+drawingAreaX,gridY [auxY]+boxSize/roomSizeY*   drawingAreaY);
    }
    
    int counterBlobs = 0;
    // Find contours
    if (mouseControl){
        counterBlobs = 1;
        
    } else {
        counterBlobs = contourFinder.nBlobs;
    }
    
    for(int i = 0; i < counterBlobs; i++) {
        
        //POINT REMAPPING
        ofPoint p;
        
        if (mouseControl){
            p.x = mouseX;
            p.y = mouseY;
            p.x /= kinect.width;
            p.x *= drawingAreaX;
            p.x += drawingPositionX;
            p.y /= kinect.height;
            p.y *= drawingAreaY;
            p.y += drawingPositionY;
        }else{
            ofRectangle r = contourFinder.blobs.at(i).boundingRect;
            p.x /= ofGetWidth();
            p.x *= drawingAreaX;
            p.x += drawingPositionX;
            p.y /= ofGetHeight();
            p.y *= drawingAreaY;
            p.y += drawingPositionY;
       
            // RECTANGLE REMAPPING
            r.x /= kinect.width;
            r.x *= drawingAreaX;
            r.x += drawingPositionX;
            r.y /= kinect.height;
            r.y *= drawingAreaY;
            r.y += drawingPositionY;
            r.width /= kinect.width;
            r.width *= drawingAreaX;
            r.height /= kinect.height;
            r.height *= drawingAreaY;
        }

        std::vector<int> vectorPermanentCue (0);
        std::vector <float> vectorPermanentCueTime (0);
        std::vector<int>::iterator it;
        std::vector<float>::iterator it2;
        
        it = vectorPermanentCue.begin();
        it2 = vectorPermanentCueTime.begin();
        int lengthVectorPermanentCue = 1;
        
        // CHECK BLOB POSITION AND TRIGGER OSC MESSAGES
        for (int countGridX = 0; countGridX < totNumBox; countGridX++)
        {
            for (int countGridY = 0; countGridY < totNumBox; countGridY++){
                if (p.x>gridX[countGridX] && p.x < (gridX[countGridX] + boxHorSize) && p.y > gridY[countGridY] && p.y < (gridY[countGridY]+boxVerSize)){
                    //if (arrayPlaying[countGridX][countGridY][layerGrid] == 0){
                    arrayRequesting[countGridX][countGridY][layerGrid] = 1;
         	           //} else {
                        //arrayRequesting[countGridX][countGridY][layerGrid] = 0;
                    //}
                } else {
                    arrayRequesting[countGridX][countGridY][layerGrid] = 0;
                    /*if (arrayPlaying[countGridX][countGridY][layerGrid] == 1){
                        arrayRequesting[countGridX][countGridY][layerGrid] = 0;
                        if (ofGetElapsedTimef() - arrayTime[countGridX][countGridY][layerGrid] < timePermanentCue) {
                            arrayTime[countGridX][countGridY][layerGrid] = 0;
                        }
                    } else {
                        arrayRequesting[countGridX][countGridY][layerGrid] = 0;
                        arrayTime[countGridX][countGridY][layerGrid] = 0;
                    }*/
                }
                
                if (arrayRequesting[countGridX][countGridY][layerGrid] == 1 && arrayPlaying[countGridX][countGridY][layerGrid] == 0){
                    ofxOscMessage m;
                    m.setAddress("/cue/" +ofToString(countGridX+1)+ofToString(countGridY+1)+ofToString(layerGrid+1)+"/start");
                    sender.sendMessage(m);
                    m.setAddress("/cue/" +ofToString(countGridX+1)+ofToString(countGridY+1)+ofToString(layerGrid+1)+"/sliderLevel/0 0");
                    sender.sendMessage(m);
                    arrayPlaying[countGridX][countGridY][layerGrid] = 1;
                    arrayTime[countGridX][countGridY][layerGrid] = ofGetElapsedTimef();
                    arrayPotentialFade[countGridX][countGridY][layerGrid] = 0;
                } else {
                    if (arrayRequesting[countGridX][countGridY][layerGrid] == 0 && arrayTime[countGridX][countGridY][layerGrid]>0) {
                        if (ofGetElapsedTimef()-arrayTime[countGridX][countGridY][layerGrid] > timePermanentCue){
                            arrayPotentialFade[countGridX][countGridY][layerGrid] = 1;
                        } else {
                            ofxOscMessage m;
                            m.setAddress("/cue/"+ofToString(countGridX+1)+ofToString(countGridY+1)+ofToString(layerGrid+1)+"/stop");
                            sender.sendMessage(m);
                            arrayPlaying[countGridX][countGridY][layerGrid] = 0;
                            arrayTime[countGridX][countGridY][layerGrid] = 0;
                            arrayLastTimePlayed[countGridX][countGridY][layerGrid]=ofGetElapsedTimef();
                            arrayPotentialFade[countGridX][countGridY][layerGrid] = 0;
                        }
                    }
                }
                
                if (arrayPlaying[countGridX][countGridY][layerGrid] == 1){
                    c.setHsb(i * 64, 255, 255);
                    ofSetColor(c);
                    //ofDrawRectangle(r);
                    if (arrayPotentialFade[countGridX][countGridY][layerGrid] == 1) {
                        it = vectorPermanentCue.insert(it, (countGridX+1)*100+(countGridY+1)*10+layerGrid);
                        it2 = vectorPermanentCueTime.insert(it2, arrayTime[countGridX][countGridY][layerGrid]);
                        lengthVectorPermanentCue++;
                    }
                }
            }
        }
        
        if (lengthVectorPermanentCue > maxPermanentCuesAtMax) {
            //SORT THE VALUES FROM THE vectorPermanentCue at the vectorPermanentCueTime Order
            std::sort(vectorPermanentCue.begin(), vectorPermanentCue.end(), MyComparator(vectorPermanentCueTime));
            
            for (int i = 1; i < lengthVectorPermanentCue;i++){ //we start at 1
                ofxOscMessage m;
                if (i<maxPermanentCuesAtMax+1){
                    //SET NORMAL LEVEL HERE
                    if (vectorPermanentCue[i]>0){
                        m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 0ty");
                    }
                } else {
                    if (i>maxPermanentCues) {
                        m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/stop");
                    } else {
                        switch (i) {
                            case maxPermanentCuesAtMax + 1:
                                //SET FIRST DROP DOWN LEVEL
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -6");
                                break;
                            case maxPermanentCuesAtMax + 2:
                                //SET SECOND DROP DOWN LEVEL
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -12");
                                break;
                            case maxPermanentCuesAtMax + 3:
                                //SET THIRD DROP DOWN LEVEL
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -18");
                                break;
                            case maxPermanentCuesAtMax + 4:
                                //SET LAST DROP DOWN LEVEL
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -25");
                                break;
                        }
                    }
                }
                sender.sendMessage(m);
            }
        } else {
            for (int i = 1; i < lengthVectorPermanentCue;i++){
                //SET NORMAL LEVEL HERE
                ofxOscMessage m;
                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 0");
                sender.sendMessage(m);
            }
        }
        
        // DRAW ALL CONTOURS ON TOP OF THE DETECTED ONES
        contourFinder.draw(drawingPositionX, drawingPositionY, drawingAreaX, drawingAreaY);
    }
    
    // draw instructions
    ofSetColor(255, 255, 255);
    stringstream reportStream;
    
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
        << "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
    << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
    << "set near threshold " << nearThreshold << " (press: + -)" << endl
    << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
    << ", fps: " << ofGetFrameRate() << endl
    << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
    << "press z to decrease minArea for blob detection and x to increase " << minArea << endl
    << "press g to increase boxSize and h to increase: " << boxPixelSize <<endl
    << "press p to alternate between calibration and blob detection " << displayCal <<endl
    << "press m to alternate between mouse and blob detection " << mouseControl <<endl;
    
    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    ofDrawBitmapString(reportStream.str(), 20, 552);
}

//--------------------------------------------------------------
void ofApp::exit() {
    kinect.setCameraTiltAngle(0); // zero the tilt on exit
    kinect.close();
    
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
    switch (key) {
        case ' ':
            bThreshWithOpenCV = !bThreshWithOpenCV;
            break;
            
        case '>':
        case '.':
            farThreshold ++;
            if (farThreshold > 255) farThreshold = 255;
            break;
            
        case '<':
        case ',':
            farThreshold --;
            if (farThreshold < 0) farThreshold = 0;
            break;
            
        case '+':
        case '=':
            nearThreshold ++;
            if (nearThreshold > 500) nearThreshold = 500;
            break;
            
        case '-':
            nearThreshold --;
            if (nearThreshold < 0) nearThreshold = 0;
            break;
            
        case 'w':
            kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
            break;
            
        case 'o':
            kinect.setCameraTiltAngle(angle); // go back to prev tilt
            kinect.open();
            break;
            
        case 'c':
            kinect.setCameraTiltAngle(0); // zero the tilt
            kinect.close();
            break;
            
        case '1':
            kinect.setLed(ofxKinect::LED_GREEN);
            break;
            
        case '2':
            kinect.setLed(ofxKinect::LED_YELLOW);
            break;
            
        case '3':
            kinect.setLed(ofxKinect::LED_RED);
            break;
            
        case '4':
            kinect.setLed(ofxKinect::LED_BLINK_GREEN);
            break;
            
        case '5':
            kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
            break;
            
        case '0':
            kinect.setLed(ofxKinect::LED_OFF);
            break;
            
        case OF_KEY_UP:
            angle++;
            if(angle>30) angle=30;
            kinect.setCameraTiltAngle(angle);
            break;
            
        case OF_KEY_DOWN:
            angle--;
            if(angle<-30) angle=-30;
            kinect.setCameraTiltAngle(angle);
            break;
        case 'z':
            minArea--;
            if(minArea<0) minArea = 0;
            break;
        case 'x':
            minArea++;
            break;
        case 'g':
            boxPixelSize--;
            if(boxPixelSize<0) boxPixelSize = 0;
            break;
        case 'h':
            boxPixelSize++;
            break;
        case 'p':
            if (displayCal==true) {
                displayCal=false;
            } else {
                displayCal = true;
            }
            break;
        case 'm':
            if (mouseControl==true) {
                mouseControl=false;
            } else {
                mouseControl = true;
            }
            break;
    }
}
