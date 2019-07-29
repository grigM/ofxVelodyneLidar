/*
 
 _|         _|    _|   _|      _|     _|_|         _|_|_|       _|_|     _|_|_|     _|    _|
 _|         _|    _|   _|_|    _|   _|    _|       _|    _|   _|    _|   _|    _|   _|  _|
 _|         _|    _|   _|  _|  _|   _|_|_|_|       _|_|_|     _|_|_|_|   _|_|_|     _|_|
 _|         _|    _|   _|    _|_|   _|    _|       _|         _|    _|   _|    _|   _|  _|
 _|_|_|_|     _|_|     _|      _|   _|    _|       _|         _|    _|   _|    _|   _|    _|
 

http://lunapark.space

 
 generated in https://asciiartgen.now.sh/
 
*/


#include "ofThread.h"
#include "ofxAsio.h"
#include <queue>
#include <pcap.h>

struct Laser
{
    float azimuth;
    float vertical;
    float distance;
    int intensity;
    int index;
    long long time;
    
    
    bool operator < ( const struct Laser& laser ) const
    {
        if( azimuth == laser.azimuth ){
            return index < laser.index;
        }
        else{
            return azimuth < laser.azimuth;
        }
    }
    
};


    
    class VelodyneCapture: public ofThread
    {
        public:
        
            std::vector<timeval> ts_array;
        
            ofPtr<ofxAsio::UDP::Socket> udpReceiver;
            string msg_str;
        
            pcap_t* pcap = nullptr;
            bool isPcapLoopPlay = false;
            std::string filename = "";
        
        
            std::queue<std::vector<Laser>> queue;
        
        
        
            int MAX_NUM_LASERS;
            std::vector<float> lut;
        
            static const int LASER_PER_FIRING = 32;
            static const int FIRING_PER_PKT = 12;
        
            #pragma pack(push, 1)
            typedef struct LaserReturn
            {
                uint16_t distance;
                uint8_t intensity;
            } LaserReturn;
            #pragma pack(pop)
        
            struct FiringData
            {
                uint16_t blockIdentifier;
                uint16_t rotationalPosition;
                LaserReturn laserReturns[LASER_PER_FIRING];
            };
        
            struct DataPacket
            {
                FiringData firingData[FIRING_PER_PKT];
                uint32_t gpsTimestamp;
                uint8_t mode;
                uint8_t sensorType;
            };
        
        
            // Constructor
            VelodyneCapture();
        
            ~VelodyneCapture();
        
            void setThreaded(bool threaded);
            const bool openSensorStream(int _port = 2368 );
            const bool openCaptureFile(string _filename, bool _fileLoopPlay);
        
        
            const bool isOpen();
            const bool isRun();
            void close();
            void retrieve( std::vector<Laser>& lasers, const bool sort = false );
        
            
            struct timeval last_time = { 0 };
            float last_azimuth = 0.0;
            std::vector<Laser> lasers;
        
        private:
            void threadedFunction();
        
            int port;
            // Operator Retrieve Capture Data with Sort
            void operator >> ( std::vector<Laser>& lasers )
            {
                // Retrieve Capture Data
                retrieve( lasers, false );
            };
        
        protected:
            bool threaded;
        
    };
    

class VLP16Capture : public VelodyneCapture
{
    
    public:
        
        VLP16Capture(int _port = 2368 );
        
        
        
        ~VLP16Capture();
        
    private:
        void initialize();
};
    
class HDL32ECapture : public VelodyneCapture
{
    
        
        public:
            HDL32ECapture( int _port = 2368 );
            ~HDL32ECapture();
        
        private:
            void initialize();
};

