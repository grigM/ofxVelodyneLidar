#include "ofApp.h"
#include "ofUtils.h"


//--------------------------------------------------------------
void ofApp::setup(){
    ofSetVerticalSync(true);
    ofSetFrameRate(60);
    cam.setNearClip(0.0);
    cam.setFarClip(60000.0);
    
    cam.setAutoDistance(true);
    
    capturePtr = ofPtr<HDL32ECapture>(new HDL32ECapture());
    //capturePtr->openSensorStream(2368);
    capturePtr->openCaptureFile(ofToDataPath("HDL32-V2_R into Butterfield into Digital Drive.pcap"), true);
    
    
    mesh.setMode(OF_PRIMITIVE_POINTS);
    
    ofEnableDepthTest();
    glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
    glPointSize(2); // make the points bigger
}
void ofApp::exit() {
 
    capturePtr->~HDL32ECapture();
}
//--------------------------------------------------------------
void ofApp::update(){
    //capturePtr->captureSensor();
    
    std::vector<Laser> lasers;
    capturePtr->retrieve(lasers, true) ;
    

    
    //ofLog() << "lasers.size(): "<< lasers.size();
    if(lasers.size()>0 && mesh.getVertices().size()!=lasers.size()){
        mesh.clearVertices();
    }
    
    
    
    //std::vector<ofVec3f> buffer( lasers.size() );
    for( const Laser& laser : lasers ){
        float distance = static_cast<float>( laser.distance );
        float azimuth  = laser.azimuth  * PI / 180.0;
        float vertical = laser.vertical * PI / 180.0;
        
        
        
        float x = ( distance * cos( vertical ) ) * sin( azimuth ) ;
        float y = ( distance * cos( vertical ) ) * cos( azimuth ) ;
        float z = ( distance * sin( vertical ) ) ;
        
        if( x == 0.0f && y == 0.0f && z == 0.0f ){
            //x = std::numeric_limits<float>::quiet_NaN();
            //y = std::numeric_limits<float>::quiet_NaN();
            //z = std::numeric_limits<float>::quiet_NaN();
        }else{
            ofVec3f pos(x, y, z);
            //ofVec3f pos(x+ofRandom(-5,5), y+ofRandom(-5,5), z+ofRandom(-5,5));
            //ofLog() << laser.intensity;
            //mesh.addColor(ofColor(laser.intensity*10, 0, 0));
            mesh.addVertex(pos);
        }
        
        //ofLog() << "x:" << x << " y:" << y << " z:" << z << endl;
        //buffer.push_back( ofVec3f( x, y, z ) );
        
        
       
    }
    
    
    
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofClear(0,0,0);
    
    //ofBackgroundGradient(ofColor::gray, ofColor::black, OF_GRADIENT_CIRCULAR);
    
    // even points can overlap with each other, let's avoid that
    cam.begin();
    //ofScale(2, -2, 2); // flip the y axis and zoom in a bit
    ofRotateY(90);
    ofTranslate(ofGetWidth() / 2, ofGetHeight() / 2);
    ofSetColor(255, 0, 0);
    mesh.draw();
    cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}

