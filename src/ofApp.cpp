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
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
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
    
    minArea = 30;
    boxPixelSize = 100;
    
    boxSize = 300;
    totNumBox = 4;
    roomSizeX = 4000;
    roomSizeY = 3800;

    boxWallDistanceX = 800;
    boxWallDistanceY = 700;
    boxDistanceX = (roomSizeX-(totNumBox*boxSize+2*boxWallDistanceX))/(totNumBox-1);
    boxDistanceY = (roomSizeY-(totNumBox*boxSize+2*boxWallDistanceY))/(totNumBox-1);
    
    drawingAreaX = 350;
    drawingAreaY = 250;
    drawingPositionX = 420;
    drawingPositionY = 270;
    
    boxHorSize = boxSize/roomSizeX*drawingAreaX;
    boxVerSize = boxSize/roomSizeY*drawingAreaY;

    bThreshWithOpenCV = true;
    isBox = false;
    inPosition = false;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;

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
    
    if(bDrawPointCloud) {
        easyCam.begin();
        drawPointCloud();
        easyCam.end();
    } else {
        // draw from the live kinect
        kinect.drawDepth(10, 10, drawingAreaX, drawingAreaY);
        kinect.draw(drawingPositionX, 10, drawingAreaX, drawingAreaY);
        contourFinder.draw(drawingPositionX, 10, drawingAreaX, drawingAreaY);
        
        grayImage.draw(10, drawingPositionY, drawingAreaX, drawingAreaY);
        contourFinder.draw(10, drawingPositionY, drawingAreaX, drawingAreaY);

    
        //BOX DETECTION
        ofColor c(255, 255, 255);
        ofColor backC(150, 150, 150);
        ofRectangle backR(drawingPositionX,drawingPositionY, drawingAreaX, drawingAreaY);
        ofSetColor(backC);
        ofDrawRectangle(backR);
    
        float gridLineX = 0;
        float gridLineY = 0;
        double boxLeft = 0;
        double boxTop = 0;
    
        //HORIZONTAL GRID
        ofSetColor(120,255,255);
    
        for (int auxX= 0; auxX < 4; auxX+=1) {
            boxLeft = (boxWallDistanceX + (boxSize+boxDistanceX)*auxX)/roomSizeX;
            gridLineX = boxLeft*drawingAreaX+drawingPositionX;
            gridX [auxX] = gridLineX;
            ofDrawLine(gridLineX,drawingPositionY,gridLineX,drawingPositionY+drawingAreaY);
            ofDrawLine(gridLineX+boxHorSize,drawingPositionY,gridLineX+boxSize/roomSizeX*drawingAreaX,drawingPositionY+drawingAreaY);
        }
    
        // VERTICAL GRID
        ofSetColor(190,0,255);
    
        for (int auxY = 0;  auxY< 4; auxY++) {
            boxTop = (boxWallDistanceY+(boxSize+boxDistanceY)*auxY)/roomSizeY;
            gridLineY = boxTop*drawingAreaY+drawingPositionY;
            gridY [auxY] = gridLineY;
            ofDrawLine(drawingPositionX,gridLineY,drawingPositionX+drawingAreaX,gridLineY);
            ofDrawLine(drawingPositionX,gridLineY+boxVerSize,drawingPositionX+drawingAreaX,gridLineY+boxSize/roomSizeY*drawingAreaY);
        }
    
        for(int i = 0; i < contourFinder.nBlobs; i++) {
            ofRectangle r = contourFinder.blobs.at(i).boundingRect;
            ofPoint p = contourFinder.blobs.at(i).centroid;
            p.x /= kinect.width;
            p.x *= drawingAreaX;
            p.x += drawingPositionX;
            p.y /= kinect.height;
            p.y *= drawingAreaY;
            p.y += drawingPositionY;
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
            
            
            if (contourFinder.blobs.at(i).area > boxPixelSize) isBox = true;

            for (int countGridX = 0; countGridX < totNumBox; countGridX++)
            {
                for (int countGridY = 0; countGridY < totNumBox; countGridY++){
                    if (p.x>gridX[countGridX] && p.x < (gridX[countGridX] + boxHorSize) && p.y > gridY[countGridY] && p.y < (gridY[countGridY]+boxVerSize)){
                        inPosition = true;
                        ofxOscMessage m;
                        
                        m.setAddress("/cue/" +ofToString(countGridX+1)+ofToString(countGridY+1)+ "/start");
                        sender.sendMessage( m );
                    }
                }
            }

            if (isBox && inPosition) {
                c.setHsb(i * 64, 255, 255);
                ofSetColor(c);
                ofDrawRectangle(r);
            }
            inPosition=false;
            isBox=false;
            contourFinder.draw(drawingPositionX, drawingPositionY, drawingAreaX, drawingAreaY);
        }
    
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
    << "press g to increase boxSize and h to increase: " << boxPixelSize <<endl;
    
    if(kinect.hasCamTiltControl()) {
        reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
    ofDrawBitmapString(reportStream.str(), 20, 552);
}

void ofApp::drawPointCloud(){
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();

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
            
        case'p':
            bDrawPointCloud = !bDrawPointCloud;
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
    }
}
