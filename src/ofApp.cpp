#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
    // open an outgoing connection to HOST:PORT
    sender_QLAB.setup( HOST, PORT_OSC_QLAB );
    //sender_DLIGHT.setup( "225.9.99.9", PORT_OSC_DLIGHT );
    
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
    
    nearThreshold = 185;
    farThreshold = 55;
    
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(6);
    
    // zero the KINECT - tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    //DISPLAY INIT
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
    }
    
    //HORIZONTAL GRID
    gridXL[0] = 0;
    gridXR[0] = 264;
    gridXL[1] = 328;
    gridXR[1] = 474;
    gridXL[2] = 558;
    gridXR[2] = 708;
    gridXL[3] = 794;
    gridXR[3] = ofGetWidth();
    
    // VERTICAL GRID
    gridYL[0] = 0;
    gridYR[0] = 122;
    gridYL[1] = 191;
    gridYR[1] = 368;
    gridYL[2] = 451;
    gridYR[2] = 610;
    gridYL[3] = 672;
    gridYR[3] = ofGetHeight();

    // INIT ARRAYS
    for (int n=0; n<totNumBox; n++) {
        for (int m=0; m<totNumBox; m++){
            for (int o =0; o<totNumBox; o++){
                arrayRequesting [n][m][o]=0;
                arrayPlaying [n][m][o]=0;
                arrayTime [n][m][o]=0;
                arrayPotentialFade [n][m][o]=0;
                arrayLastTimePlayed [n][m][o]=0;
                arrayFadeOut6Sent[n][m][o]=0;
                arrayFadeOut12Sent[n][m][o]=0;
                arrayFadeOut18Sent[n][m][o]=0;
                arrayFadeOut25Sent[n][m][o]=0;
            }
        }
    }
    
    mouseControl = false;
    
    //Serial
    Serial.listDevices();
    
    vector <ofSerialDeviceInfo> deviceList = Serial.getDeviceList();
    cout <<"Device list size "<< ofToString(deviceList.size())<< endl;
    
    //FIND ARDUINO'S INDEX HERE
    Serial.setup(0, 9600); //FIX IF THIS
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
    
    if(Serial.available() > 0) {
        int LEN = 2;
        char c[LEN + 1];
        int numBytes = 0;
        numBytes = Serial.readByte();
        if (numBytes !=-2) {
            cout << "Read from Serial " << numBytes<< endl;
            switch (numBytes) {
                case 1:
                    layerGrid = 0;
                    resetAll = false;
                    break;
                case 2:
                    layerGrid = 1;
                    resetAll = false;
                    break;
                case 3:
                    layerGrid = 2;
                    resetAll = false;
                    break;
                case 4:
                    layerGrid = 3;
                    resetAll = false;
                    break;
                case 48:
                    resetAll = true;
                    break;
            }
        }
        // SEND MESSAGE TO DLIGHT via QLAB
        ofxOscMessage m;
        if (resetAll == true) {
            m.setAddress("/cue/999/start");
        } else {
            m.setAddress("/cue/" +ofToString((layerGrid+1)*100)+"/start");
        }
        sender_QLAB.sendMessage(m);

        cout << "layerGrid " << layerGrid << endl;;
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    ofSetColor(255, 255, 255);
    
    // draw from the live kinect
    // depth IR
    ofColor c(255, 255, 255);
    if(displayCal) {
        drawingAreaX = ofGetWidth();
        drawingAreaY = ofGetHeight();
        drawingPositionX = 0;
        drawingPositionY = 0;
        kinect.drawDepth(drawingPositionX, drawingPositionY, drawingAreaX, drawingAreaY);
    }else {
        drawingAreaX = 350;
        drawingAreaY = 250;
        drawingPositionX = 420;
        drawingPositionY = 270;

        
        // RGB
        kinect.draw(drawingPositionX, 10, drawingAreaX, drawingAreaY);
        contourFinder.draw(drawingPositionX, 10, drawingAreaX, drawingAreaY);
        // depth contour
        grayImage.draw(10, drawingPositionY, drawingAreaX, drawingAreaY);
        contourFinder.draw(10, drawingPositionY, drawingAreaX, drawingAreaY);
        

        ofColor backC(150, 150, 150);
        ofRectangle backR(drawingPositionX,drawingPositionY, drawingAreaX, drawingAreaY);
        ofSetColor(backC);
        ofDrawRectangle(backR);
    }

    //BOX DETECTION

    // DRAW GRID

    //HORIZONTAL GRID
    ofSetColor(255,200,0);
    
    for (int auxX= 0; auxX < totNumBox; auxX++) {
        ofDrawLine(gridXL [auxX],drawingPositionY,gridXL [auxX],drawingPositionY+drawingAreaY);
        ofDrawLine(gridXR [auxX],drawingPositionY,gridXR [auxX],drawingPositionY+drawingAreaY);
    }
    
    // VERTICAL GRID
    ofSetColor(255,200,0);
    
    for (int auxY = 0;  auxY< totNumBox; auxY++) {
        ofDrawLine(drawingPositionX,gridYL [auxY],drawingPositionX+drawingAreaX,gridYL [auxY]);
        ofDrawLine(drawingPositionX,gridYR [auxY],drawingPositionX+drawingAreaX,gridYR [auxY]);
    }
    
    int counterBlobs = 0;
    // Find contours
    if (mouseControl){
        counterBlobs = 1;
        
    } else {
        counterBlobs = contourFinder.nBlobs;
    }
    
    if (resetAll){
        ofxOscMessage m;
        
        m.setAddress("/PanicAll");
        sender_QLAB.sendMessage(m);
        // RESET EVERYTHING
        for (int countLayer = 0; countLayer < 4; countLayer++) {
            for (int countGridX = 0; countGridX < totNumBox; countGridX++){
                for (int countGridY = 0; countGridY < totNumBox; countGridY++){
                    
                    m.setAddress("/cue/" +ofToString(countGridX+1)+ofToString(countGridY+1)+ofToString(countLayer+1)+"/sliderLevel/0 0");
                    sender_QLAB.sendMessage(m);
                    arrayRequesting [countGridX][countGridY][countLayer]=0;
                    arrayPlaying [countGridX][countGridY][countLayer]=0;
                    arrayTime [countGridX][countGridY][countLayer]=0;
                    arrayPotentialFade [countGridX][countGridY][countLayer]=0;
                    arrayLastTimePlayed [countGridX][countGridY][countLayer]=0;
                    arrayFadeOut6Sent[countGridX][countGridY][countLayer]=0;
                    arrayFadeOut12Sent[countGridX][countGridY][countLayer]=0;
                    arrayFadeOut18Sent[countGridX][countGridY][countLayer]=0;
                    arrayFadeOut25Sent[countGridX][countGridY][countLayer]=0;
                    
                }
            }
        }
        resetAll = false;
        layerGrid = 0;
    }
    
    std::vector<int> vectorPermanentCue (0);
    std::vector<int> vectorPermanentCueTime (0);
    std::vector<int>::iterator it;
    std::vector<int>::iterator it2;
    
    it = vectorPermanentCue.begin();
    it2 = vectorPermanentCueTime.begin();
    int lengthVectorPermanentCue = 0;
    
    bool addedCue = false;
    for(int i = 0; i < counterBlobs; i++) {
        

        ofPoint p;
        if (mouseControl){
            //POINT REMAPPING

            p.x = mouseX;
            p.y = mouseY;
            
            p.x /= ofGetWidth();
            p.x *= drawingAreaX;
            p.x += drawingPositionX;

            p.y /= ofGetHeight();
            p.y *= drawingAreaY;
            p.y += drawingPositionY;
            stringstream reportStream2;
            reportStream2 << "P position" <<p.x<<" - "<<p.y<< endl;

            ofDrawBitmapString(reportStream2.str(), 500, 600);
            
        }else{
            //if (countourFinder.blobs.at(i).area>minArea) {
            
                p = contourFinder.blobs.at(i).centroid;
            
                p.x /= kinect.width;
                p.x *= drawingAreaX;
                p.x += drawingPositionX;
            
                p.y /= kinect.height;
                p.y *= drawingAreaY;
                p.y += drawingPositionY;
            
                stringstream reportStream3;
                reportStream3 << "P position" <<p.x<<" - "<<p.y<< endl;
            
                ofDrawBitmapString(reportStream3.str(), 500, 600);
                // RECTANGLE REMAPPING
                /*ofRectangle r = contourFinder.blobs.at(i).boundingRect;
            
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
                 */
            //}
            
        }

        addedCue = false;
        
        // CHECK BLOB POSITION
        for (int countGridX = 0; countGridX < totNumBox; countGridX++)
        {
            for (int countGridY = 0; countGridY < totNumBox; countGridY++){
                
                if (p.x>gridXL[countGridX] && p.x < (gridXR[countGridX]) && p.y > gridYL[countGridY] && p.y < (gridYR[countGridY])){

                    arrayRequesting[countGridX][countGridY][layerGrid] = 1;

                } else {
                    
                    arrayRequesting[countGridX][countGridY][layerGrid] = 0;

                }
            }
        }
    }
    
    for (int countGridX = 0; countGridX < totNumBox; countGridX++)
    {
        for (int countGridY = 0; countGridY < totNumBox; countGridY++){
        
            if (arrayRequesting[countGridX][countGridY][layerGrid] == 1 && arrayPlaying[countGridX][countGridY][layerGrid] == 0){
                // SEND MESSAGE TO QLAB
                ofxOscMessage m;
                    
                m.setAddress("/cue/" +ofToString(countGridX+1)+ofToString(countGridY+1)+ofToString(layerGrid+1)+"/start");
                sender_QLAB.sendMessage(m);
                    
                m.setAddress("/cue/" +ofToString(countGridX+1)+ofToString(countGridY+1)+ofToString(layerGrid+1)+"/sliderLevel/0 0");
                sender_QLAB.sendMessage(m);
                    
                    
                arrayPlaying[countGridX][countGridY][layerGrid] = 1;
                arrayTime[countGridX][countGridY][layerGrid] = ofGetElapsedTimef();
                arrayPotentialFade[countGridX][countGridY][layerGrid] = 0;
            } else {
                if (arrayRequesting[countGridX][countGridY][layerGrid] == 0 && arrayTime[countGridX][countGridY][layerGrid]>0) {
                    /*
                    if (ofGetElapsedTimef()-arrayTime[countGridX][countGridY][layerGrid] > timePermanentCue){
                            arrayPotentialFade[countGridX][countGridY][layerGrid] = 1;
                        if (arrayPlaying[countGridX][countGridY][layerGrid] == 1 && fadedCues == false) {
                                addedCue=true;
                        }
                    } else {*/
                        ofxOscMessage m;
                        m.setAddress("/cue/"+ofToString(countGridX+1)+ofToString(countGridY+1)+ofToString(layerGrid+1)+"/stop");
                        sender_QLAB.sendMessage(m);
                        arrayPlaying[countGridX][countGridY][layerGrid] = 0;
                        arrayTime[countGridX][countGridY][layerGrid] = 0;
                        arrayLastTimePlayed[countGridX][countGridY][layerGrid]=ofGetElapsedTimef();
                        arrayPotentialFade[countGridX][countGridY][layerGrid] = 0;
                    //}
                }
            }
        }
        
        /*
        for (int countGridX = 0; countGridX < totNumBox; countGridX++)
        {
            for (int countGridY = 0; countGridY < totNumBox; countGridY++){

                if (arrayPlaying[countGridX][countGridY][layerGrid] == 1){
                    if (arrayPotentialFade[countGridX][countGridY][layerGrid] == 1) {
                        it = vectorPermanentCue.insert(it, (countGridX+1)*100+(countGridY+1)*10+layerGrid+1);
                        it2 = vectorPermanentCueTime.insert(it2, arrayTime[countGridX][countGridY][layerGrid]);
                        lengthVectorPermanentCue++;
                    }
                }
            }
        }*/
    }
    /*
    if (lengthVectorPermanentCuePrev < lengthVectorPermanentCue) {
        fadedCues = false;
    }
     
    if (lengthVectorPermanentCue > maxPermanentCuesAtMax && addedCue==true && fadedCues == false) {
        addedCue = false;
        fadedCues = true;
        lengthVectorPermanentCuePrev = lengthVectorPermanentCue;
        //SORT THE VALUES FROM THE vectorPermanentCue at the vectorPermanentCueTime Order
        sort(vectorPermanentCue.begin(), vectorPermanentCue.end(), vectorCompare(vectorPermanentCueTime));

        //sort(vectorPermanentCueTime.begin(),vectorPermanentCueTime.end());
        for (int i = 0; i < lengthVectorPermanentCue;i++){
            int resetGridX = round(vectorPermanentCue[i]/100)-1;
            int resetGridY = round((vectorPermanentCue[i]-(1+resetGridX)*100)/10)-1;
            int resetGridZ = round(vectorPermanentCue[i]-(1+resetGridX)*100-(1+resetGridY)*10)-1;
            ofxOscMessage m;
            if (i<maxPermanentCuesAtMax){
                //SET NORMAL LEVEL HERE
                if (arrayFadeOut6Sent[resetGridX][resetGridY][resetGridZ]==0 && arrayFadeOut12Sent[resetGridX][resetGridY][resetGridZ]==0 && arrayFadeOut18Sent[resetGridX][resetGridY][resetGridZ]==0 && arrayFadeOut25Sent[resetGridX][resetGridY][resetGridZ]==0 && vectorPermanentCue[i]>2){
                        
                    m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 0");
                        sender_QLAB.sendMessage(m);
                }
            } else {

                int timeCheck = ofGetElapsedTimef();
                    
                if (i>maxPermanentCues) {
                    if (timeCheck-arrayTime[resetGridX][resetGridY][resetGridZ] > 15 && (arrayFadeOut6Sent[resetGridX][resetGridY][resetGridZ]==1 || arrayFadeOut12Sent[resetGridX][resetGridY][resetGridZ]==1 || arrayFadeOut18Sent[resetGridX][resetGridY][resetGridZ]==1 || arrayFadeOut25Sent[resetGridX][resetGridY][resetGridZ]==1)){
                        m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/stop");
                        sender_QLAB.sendMessage(m);
                        m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 0");
                        sender_QLAB.sendMessage(m);
                        //Not playing anymore
                        arrayRequesting [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayPlaying [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayTime [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayPotentialFade [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayLastTimePlayed [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut6Sent [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut12Sent [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut18Sent [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut25Sent [resetGridX][resetGridY][resetGridZ] = 0;
                    }
                } else {

                    switch (i) {
                        case maxPermanentCuesAtMax + 1:
                            //SET FIRST DROP DOWN LEVEL
                            if (timeCheck-arrayTime[resetGridX][resetGridY][resetGridZ] > 10 && arrayFadeOut12Sent[resetGridX][resetGridY][resetGridZ]== 0 && arrayFadeOut18Sent[resetGridX][resetGridY][resetGridZ]==0 && arrayFadeOut25Sent[resetGridX][resetGridY][resetGridZ]==0){
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -6");
                                sender_QLAB.sendMessage(m);
                                arrayFadeOut6Sent[resetGridX][resetGridY][resetGridZ]=1;
                            }
                            break;
                        case maxPermanentCuesAtMax + 2:
                            //SET SECOND DROP DOWN LEVEL
                            if (timeCheck-arrayTime[resetGridX][resetGridY][resetGridZ] > 10 && arrayFadeOut18Sent[resetGridX][resetGridY][resetGridZ]==0 && arrayFadeOut25Sent[resetGridX][resetGridY][resetGridZ]==0){
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -12");
                                sender_QLAB.sendMessage(m);
                                arrayFadeOut12Sent[resetGridX][resetGridY][resetGridZ]=1;
                            }
                            break;
                        case maxPermanentCuesAtMax + 3:
                            //SET THIRD DROP DOWN LEVEL
                            if (timeCheck-arrayTime[resetGridX][resetGridY][resetGridZ] > 10 && arrayFadeOut25Sent[resetGridX][resetGridY][resetGridZ]==0){
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -18");
                                sender_QLAB.sendMessage(m);
                                arrayFadeOut18Sent[resetGridX][resetGridY][resetGridZ]=1;
                            }
                            break;
                        case maxPermanentCuesAtMax + 4:
                            //SET LAST DROP DOWN LEVEL
                            if (timeCheck-arrayTime[resetGridX][resetGridY][resetGridZ] > 10){
                                m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 -25");
                                sender_QLAB.sendMessage(m);
                                arrayFadeOut25Sent[resetGridX][resetGridY][resetGridZ]=1;
                            }
                            break;
                    }
                        
                    if (timeCheck-arrayTime[resetGridX][resetGridY][resetGridZ] > 600){
                        m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/stop");
                        sender_QLAB.sendMessage(m);
                        m.setAddress("/cue/"+ofToString(vectorPermanentCue[i])+"/sliderLevel/0 0");
                        sender_QLAB.sendMessage(m);
                        //Not playing anymore
                        arrayRequesting [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayPlaying [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayTime [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayPotentialFade [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayLastTimePlayed [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut6Sent [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut12Sent [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut18Sent [resetGridX][resetGridY][resetGridZ] = 0;
                        arrayFadeOut25Sent [resetGridX][resetGridY][resetGridZ] = 0;
                    }
                }
            }
        }
    }*/
        
    // DRAW ALL CONTOURS ON TOP OF THE DETECTED ONES
    contourFinder.draw(drawingPositionX, drawingPositionY, drawingAreaX, drawingAreaY);
        
    
    
    // draw instructions
    ofSetColor(255, 255, 200);
    stringstream reportStream;
    
    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
        << "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
    float calibrated_value = 0;
    if (GridCal==true) {
        if (HorizontalCal == true) {
            if (gridRight == true) {
                calibrated_value = gridXR[indexGrid];
            }
            if (gridLeft == true) {
                calibrated_value = gridXL[indexGrid];
            }                }
        if (VerticalCal ==true) {
            if (gridRight == true) {
                calibrated_value = gridYR[indexGrid];
            }
            if (gridLeft == true) {
                calibrated_value = gridYL[indexGrid];
            }
        }
    }
    
    reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
    << "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
    << "set near threshold " << nearThreshold << " (press: + -)" << endl
    << "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
    << ", fps: " << ofGetFrameRate() << endl
    << "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
    << "press z to decrease minArea for blob detection and x to increase " << minArea << endl
    << "press p to alternate between calibration and blob detection " << displayCal <<endl
    << "press m to alternate between mouse and blob detection " << mouseControl <<endl
    << "Mouse position " << mouseX << " - "<< mouseY << endl
    << "----------------------------------"<< endl
    << "Grid Size calibration on/off with k/l " << GridCal << endl
    << "Select Grid ROW / COLUMN to calibrate 1q 2w 3e 4r " << indexGrid <<endl
    << "Vertical grid calibration v/h VerticalCal " << VerticalCal << " // HorizontalCal " << HorizontalCal << endl
    << "Left or right side grid calibration u/i gridRight " << gridRight << " // gridLeft " << gridLeft <<endl

    << "Calibrated value: " <<  calibrated_value <<endl;
    
    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    ofDrawBitmapString(reportStream.str(), 20, 450);
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
            
        case 'a':
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
        case 'v':
            if (GridCal==true) {
                VerticalCal = true;
                HorizontalCal = false;
            }
            break;
        case 'h':
            if (GridCal==true) {
                HorizontalCal = true;
                VerticalCal = false;
            }
            break;
        case 'k':
            GridCal = true;
            break;
        case 'l':
            GridCal = false;
            break;
        case 'q':
            indexGrid=0;
            break;
        case 'w':
            indexGrid=1;
            break;
        case 'e':
            indexGrid=2;
            break;
        case 'r':
            indexGrid=3;
            break;
        case 'u':
            gridRight=true;
            gridLeft=false;
            break;
        case 'i':
            gridRight=false;
            gridLeft=true;
            break;
        case OF_KEY_RIGHT:
            if (GridCal==true) {
                if (HorizontalCal == true) {
                    if (gridRight == true) {
                        gridXR[indexGrid] = gridXR[indexGrid]+2;
                    }
                    if (gridLeft == true) {
                        gridXL[indexGrid] = gridXL[indexGrid]+2;
                    }
                }
                if (VerticalCal ==true) {
                    if (gridRight == true) {
                        gridYR[indexGrid] = gridYR[indexGrid]+2;
                    }
                    if (gridLeft == true) {
                        gridYL[indexGrid] = gridYL[indexGrid]+2;
                    }
                }
            }
            break;
            
        case OF_KEY_LEFT:
            if (GridCal==true) {
                if (HorizontalCal == true) {
                    if (gridRight == true) {
                        gridXR[indexGrid] = gridXR[indexGrid]-2;
                    }
                    if (gridLeft == true) {
                        gridXL[indexGrid] = gridXL[indexGrid]-2;
                    }                }
                if (VerticalCal ==true) {
                    if (gridRight == true) {
                        gridYR[indexGrid] = gridYR[indexGrid]-2;
                    }
                    if (gridLeft == true) {
                        gridYL[indexGrid] = gridYL[indexGrid]-2;
                    }
                }
            }
            break;
    }
}
