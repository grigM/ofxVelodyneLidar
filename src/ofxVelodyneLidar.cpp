
#include "ofxVelodyneLidar.h"


//using namespace "ofxVelodyneLidar;





VelodyneCapture::VelodyneCapture()
{
    
};
        

VelodyneCapture::~VelodyneCapture()
{
    close();
};

/*
VelodyneCapture::VelodyneCapture(int _port )
{
    port = _port;
    //open(port);
};
 */

const bool VelodyneCapture::openSensorStream(int _port)
{
    if(_port){
        port = _port;
    }
    udpReceiver = ofPtr<ofxAsio::UDP::Socket>( new ofxAsio::UDP::Socket(port));
    
    if(!isThreadRunning()){
        startThread();
    }
    return true;
};

const bool VelodyneCapture::openCaptureFile(string _filename, bool _fileLoopPlay){
   
    
    filename =  _filename;
    isPcapLoopPlay = _fileLoopPlay;
    
    char error[PCAP_ERRBUF_SIZE];
   pcap = pcap_open_offline( filename.c_str(), error );
    
  
    
    if( !pcap ){
        throw std::runtime_error( error );
        return false;
    }
    
    // Convert PCAP_NETMASK_UNKNOWN to 0xffffffff
    struct bpf_program filter;
    std::ostringstream oss;
    if( pcap_compile( pcap, &filter, oss.str().c_str(), 0, 0xffffffff ) == -1 ){
        throw std::runtime_error( pcap_geterr( pcap ) );
        return false;
    }
    
    if( pcap_setfilter( pcap, &filter ) == -1 ){
        throw std::runtime_error( pcap_geterr( pcap ) );
        return false;
    }
    
    
    filename =  _filename;
    
    if(!isThreadRunning()){
        startThread();
    }
    
    
    return true;
    
}
        
        
        
// Check Open
const bool VelodyneCapture::isOpen()
{
    
    lock();
    //return udpConnection.HasSocket();
    return true;
};

// Check Run
const bool VelodyneCapture::isRun()
{
    // Returns True when Thread is Running or Queue is Not Empty
    lock();
    return ( isThreadRunning() || !queue.empty() );
}

// Close Capture
void VelodyneCapture::close()
{
    if(isThreadRunning()) {
        stopThread();
        ofSleepMillis(10);
        waitForThread(false);
    }
    
    if(udpReceiver!=nullptr){
        udpReceiver->close();
    }
    
    
    if( pcap!=nullptr ){
        pcap_close( pcap );
        pcap = nullptr;
        //filename = "";
    }
    
    
    // Clear Queue
    std::queue<std::vector<Laser>>().swap( queue );
};

// Retrieve Capture Data
void VelodyneCapture::retrieve( std::vector<Laser>& lasers, const bool sort )
{
    // Pop One Rotation Data from Queue
    if(lock()){
        if( !queue.empty() ){
            lasers = queue.front();
            if( sort ){
                std::sort( lasers.begin(), lasers.end() );
            }
            queue.pop();
        }
        unlock();
    }
};










void VelodyneCapture::threadedFunction(){
    while(isThreadRunning())
    {
        
        //return;
        
        //run = true;
        
        if(udpReceiver!=nullptr && pcap==nullptr){
            lock();
            
            
                shared_ptr<ofxAsio::UDP::DataGram>  dataGram = udpReceiver->receive();
                //int bytesReceived = udpConnection.Receive(udpMessage,8000000);
                const auto & message = dataGram->getMessage();
                msg_str = message.getMessageString();
            
                //std::cout << "message.size: " << message.size()  << std::endl;
            
                if (message.size() != 1206)
                {
                    ofLog() << "bytesReceived:" << message.size();
                    // Data-Packet Specifications says that laser-packets are 1206 byte long.
                    //  That is : (2+2+(2+1)*32)*12 + 4 + 1 + 1
                    //                #lasers^   ^#firingPerPkt
                    return;
                }
            
            
            unlock();
            
            
            // Retrieve Unix Time ( microseconds )
            const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>( now.time_since_epoch() );
            const long long unixtime = epoch.count();
            
            // Convert to DataPacket Structure
            // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
            const DataPacket* packet = reinterpret_cast<const DataPacket*>( message.data() );
            assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );
            
            // Caluculate Interpolated Azimuth
            
            float interpolated = 0.0;
            if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
                interpolated = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition ) / 2.0;
            }
            else{
                interpolated = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition ) / 2.0;
            }
            
            
            // Processing Packet
            for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                // Retrieve Firing Data
                
                
                
                const FiringData firing_data = packet->firingData[firing_index];
                
                for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                    
                    float azimuth =  static_cast<float>(firing_data.rotationalPosition);
                    
                    if( laser_index >= MAX_NUM_LASERS )
                    {
                        azimuth += interpolated;
                    }
                    // Reset Rotation Azimuth
                    if( azimuth >= 36000 )
                    {
                        azimuth -= 36000;
                    }
                    
                    
                    // Complete Retrieve Capture One Rotation Data
                    // ofLog() << "last_azimuth:" << last_azimuth << " azimuth: " <<  azimuth;
                    
                    if( last_azimuth > azimuth ){
                        
                        if(lock()){
                            queue.push( lasers );
                            lasers.clear();
                            unlock();
                        }
                    }
                    
                    
                    
                    Laser laser;
                    laser.azimuth = azimuth / 100.00;
                    laser.vertical = lut[laser_index % MAX_NUM_LASERS];
                    laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance;
                    laser.intensity = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].intensity;
                    laser.index =  laser_index % MAX_NUM_LASERS ;
                    laser.time = unixtime;
                    
                    lasers.push_back( laser );
                    
                    last_azimuth = azimuth;
                }
            }
            
            //ofLog() << "udp thread";
            
            //run = false;
        }else if(pcap!=nullptr && udpReceiver==nullptr){
            
            //ofLog() << "pcap thread";
            
            
            
            struct pcap_pkthdr* header;
            const unsigned char* data;
            const int ret = pcap_next_ex( pcap, &header, &data );
            
            if( ret <= 0 ){
                if(isPcapLoopPlay){
                    openCaptureFile(filename, isPcapLoopPlay);
                }else{
                    break;
                }
            }
            
            // Check Packet Data Size
            // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
            if( ( header->len - 42 ) != 1206 ){
                continue;
            }
           
            // Retrieve Unix Time ( microseconds )
            std::stringstream ss;
            ss << header->ts.tv_sec << std::setw( 6 ) << std::left << std::setfill( '0' ) << header->ts.tv_usec;
            const long long unixtime = std::stoll( ss.str() );
            
           
            
            // Convert to DataPacket Structure ( Cut Header 42 bytes )
            // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
            const DataPacket* packet = reinterpret_cast<const DataPacket*>( data + 42 );
            assert( packet->sensorType == 0x21 || packet->sensorType == 0x22 );
            
            // Wait This Thread Difference Time
            if( last_time.tv_sec == 0 )
            {
                last_time = header->ts;
            }
            
            if( last_time.tv_usec > header->ts.tv_usec )
            {
                last_time.tv_usec -= 1000000;
                last_time.tv_sec++;
            }
            
            
            const unsigned long long delay = ( ( header->ts.tv_sec - last_time.tv_sec ) * 1000000 ) + ( header->ts.tv_usec - last_time.tv_usec );
            std::this_thread::sleep_for( std::chrono::microseconds( delay ) );
            last_time = header->ts;
            
            // Caluculate Interpolated Azimuth
            double interpolated = 0.0;
            if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
                interpolated = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition ) / 2.0;
            }
            else{
                interpolated = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition ) / 2.0;
            }
            
            // Processing Packet
            for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
                // Retrieve Firing Data
                const FiringData firing_data = packet->firingData[firing_index];
                for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
                    // Retrieve Rotation Azimuth
                    double azimuth = static_cast<double>( firing_data.rotationalPosition );
                    
                    // Interpolate Rotation Azimuth
                    if( laser_index >= MAX_NUM_LASERS )
                    {
                        azimuth += interpolated;
                    }
                    
                    // Reset Rotation Azimuth
                    if( azimuth >= 36000 )
                    {
                        azimuth -= 36000;
                    }
                    
                    // Complete Retrieve Capture One Rotation Data
                    if( last_azimuth > azimuth ){
                        // Push One Rotation Data to Queue
                        if(lock()){
                            queue.push( lasers );
                            lasers.clear();
                            
                            unlock();
                        }
                        
                    }
                    
                    Laser laser;
                    laser.azimuth = azimuth / 100.0;
                    laser.vertical = lut[laser_index % MAX_NUM_LASERS];
                    laser.distance = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].distance;
                    laser.intensity = firing_data.laserReturns[laser_index % MAX_NUM_LASERS].intensity;
                    laser.index =  laser_index % MAX_NUM_LASERS ;
                    laser.time = unixtime;
                    
                    lasers.push_back( laser );
                    
                    // Update Last Rotation Azimuth
                    last_azimuth = azimuth;
                }
            }
            
            
        }
        
        
        
    }
}
        

    

VLP16Capture::VLP16Capture(int _port )/*: VelodyneCapture(_port )*/
{
    MAX_NUM_LASERS = 16;
    lut = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0 };
    
    initialize();
};
        
        
        
VLP16Capture::~VLP16Capture()
{
};
        

void VLP16Capture::initialize()
{
    VelodyneCapture::MAX_NUM_LASERS = MAX_NUM_LASERS;
    VelodyneCapture::lut = lut;
};


        
HDL32ECapture::HDL32ECapture( int _port)/*: VelodyneCapture(_port )*/
{
    MAX_NUM_LASERS = 32;
    lut = { -30.67, -9.3299999, -29.33, -8.0, -28, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16, 5.3299999, -14.67, 6.6700001, -13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67 };
    
    initialize();
};
        
        
        
        
HDL32ECapture::~HDL32ECapture()
{
   
};
        
void HDL32ECapture::initialize()
{
    VelodyneCapture::MAX_NUM_LASERS = MAX_NUM_LASERS;
    VelodyneCapture::lut = lut;
};


