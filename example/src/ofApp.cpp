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
        
        urg.setupEthernet();
        
        cout << urg.productType() << endl;
        cout << urg.firmwareVersion() << endl;
        cout << urg.serialId() << endl;
        cout << urg.status() << endl;
        cout << urg.state() << endl;
        
        urg.start();
    }
    
    void update()
    {
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
    
    void keyPressed(int key)
    {
        
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
