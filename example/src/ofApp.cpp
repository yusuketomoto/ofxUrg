#include "ofMain.h"
#include "ofxUrg.h"

class ofApp : public ofBaseApp {
    
    ofxUrg::Device urg;
    ofEasyCam cam;
    
public:
    void setup()
    {
        ofSetFrameRate(60);
        ofSetVerticalSync(true);
        ofBackground(0);
        
        urg.setMode(ofxUrg::DISTANCE_INTENSITY);
        urg.setupEthernet();
        
        ofLogNotice("Product", urg.productType());
        ofLogNotice("Serial", urg.serialId());
        ofLogNotice("Status", urg.status());
        ofLogNotice("State", urg.state());
        ofLogNotice("Firmware version", urg.firmwareVersion());
        
        urg.start();
    }
    
    void update()
    {
        urg.update();
    }
    
    void draw()
    {
        cam.begin();
        ofPushMatrix();
        float s = 0.1;
        ofScale(s, s, s);
        ofRotateZ(-90);
        urg.drawDebugPolar();
        ofPopMatrix();
        cam.end();
        
        ofDrawBitmapString(ofToString(ofGetFrameRate(), 0), 20, 20);
    }
};

//========================================================================
int main( ){
	ofSetupOpenGL(1024,768,OF_WINDOW);			// <-------- setup the GL context
    
	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp(new ofApp());
    
}
