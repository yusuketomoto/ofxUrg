#pragma once

#include "Urg_Driver.h"

#define OFX_URG_BEGIN_NAMESPACE namespace ofx { namespace Urg {
#define OFX_URG_END_NAMESPACE } }

OFX_URG_BEGIN_NAMESPACE

static const string DEFAULT_HOST = "192.168.0.10";
static const int DEFAULT_PORT = 10940;

enum ConnectionType
{
    SERIAL,
    ETHERNET
};
enum Mode
{
    DISTANCE,
    DISTANCE_INTENSITY,
    MULTIECHO,
    MULTIECHO_INTENSITY
};

class Device : public ofThread
{
    
public:
    Device()
    :device_or_ip_name(DEFAULT_HOST)
    ,baudrate_or_port_number(DEFAULT_PORT)
    ,connect_type(ETHERNET)
    ,active(false)
    ,is_frame_new(false)
    {}
    
    virtual ~Device()
    {
        if (urg.is_open()) urg.close();
    }
    
    void setupSerial(const string& device_name = "", int baudrate = 115200)
    {
        connect_type = SERIAL;
        
        if (!device_name.empty())
        {
            device_or_ip_name = device_name;
        }
        else
        {
            device_or_ip_name = "/dev/tty.usbmodemfa131";
        }
        baudrate_or_port_number = baudrate;
        
        open();
    }
    
    void setupEthernet(const string& ip = DEFAULT_HOST, int port = DEFAULT_PORT)
    {
        connect_type = ETHERNET;
        device_or_ip_name = ip;
        baudrate_or_port_number = port;
        
        open();
    }
    
    void open()
    {
        if (!urg.open(device_or_ip_name.c_str(), baudrate_or_port_number,
                      (connect_type==SERIAL) ? qrk::Urg_driver::Serial : qrk::Urg_driver::Ethernet))
        {
            ofLogError("Urg connection failed", urg.what());
        }
        
        urg.set_scanning_parameter(minStep(), maxStep());
        urg.set_sensor_time_stamp(ofGetElapsedTimef());
    }
    
    void close() { urg.close(); }
    
    bool start(const Mode& mode = DISTANCE_INTENSITY)
    {
        urg.stop_measurement();
        
        this->mode = mode;
        
        qrk::Lidar::measurement_type_t measurement_type;
        switch (mode) {
            case DISTANCE:
                measurement_type = qrk::Lidar::Distance;
                break;
            case DISTANCE_INTENSITY:
                measurement_type = qrk::Lidar::Distance_intensity;
                break;
            case MULTIECHO:
                measurement_type = qrk::Lidar::Multiecho;
                break;
            case MULTIECHO_INTENSITY:
                measurement_type = qrk::Lidar::Multiecho_intensity;
                break;
        }
        
        active = urg.start_measurement(measurement_type, qrk::Urg_driver::Infinity_times, 0);
        startThread();
        return active;
    }
    void stop()
    {
        urg.stop_measurement();
        stopThread();
    }
    
    void drawDebug(float width = ofGetWindowWidth(), float height = ofGetWindowHeight()) const
    {
        ofMutex mutex;
        mutex.lock();
        for (int i=0; i<data.size(); i++)
        {
            float x = ofMap(i, 0, data.size(), 0, width, true);
            float y = ofMap(data[i], 0, 1500, 0, height, true);
            ofLine(x, 0, x, y);
        }
        mutex.unlock();
    }
    void drawDebugPolar() const
    {
        ofMesh m;
        m.setMode(OF_PRIMITIVE_TRIANGLE_FAN);
        m.addVertex(ofVec3f::zero());
        
        ofMutex mutex;
        mutex.lock();
        for (int i=0; i<data.size(); i++)
        {
            float r = data[i];
            if (r > 60000) continue;
            float theta = urg.index2rad(i);
            float x = r * cos(theta);
            float y = r * sin(theta);
            m.addVertex(ofVec3f(x,y));
        }
        mutex.unlock();
        
        m.draw();
    }
    bool isFrameNew() const { return is_frame_new; }
    
    string  productType(void)       const { return urg.product_type(); }
    string  firmwareVersion(void)   const { return urg.firmware_version(); }
    string  serialId(void)          const { return urg.serial_id(); }
    string  status(void)            const { return urg.status(); }
    string  state(void)             const { return urg.state(); }
    
    int     minStep(void)           const { return urg.min_step(); }
    int     maxStep(void)           const { return urg.max_step(); }
    long    minDistance(void)       const { return urg.min_distance(); }
    long    maxDistance(void)       const { return urg.max_distance(); }
    long    scanUsec(void)          const { return urg.scan_usec(); }
    int     maxDataSize(void)       const { return urg.max_data_size(); }
    int     maxEchoSize(void)       const { return urg.max_echo_size(); }
    
    const vector<long>& getData() const { return data; }
    long getData(int index) const { return data.at(index); }
    const vector<unsigned short>& getIntensity() const { return intensity; }
    unsigned short getIntensity(int index) const { return intensity.at(index); }
    
protected:
    
    void threadedFunction()
    {
        while (isThreadRunning())
        {
            if (lock())
            {
                if (active)
                {
                    if (mode == DISTANCE)
                    {
                        if (!urg.get_distance(data, &timestamp))
                            ofLogError("urg get distance", urg.what());
                    }
                    if (mode == DISTANCE_INTENSITY)
                    {
                        if (!urg.get_distance_intensity(data, intensity, &timestamp))
                            ofLogError("urg get distance intensity", urg.what());
                    }
                }
            }
            unlock();
        }
    }
    string device_or_ip_name;
    int baudrate_or_port_number;
    ConnectionType connect_type;
    Mode mode;
    qrk::Urg_driver urg;
    
    bool active;
    bool is_frame_new;
    vector<long> data;
    vector<unsigned short> intensity;
    long timestamp;
};

OFX_URG_END_NAMESPACE

namespace ofxUrg = ofx::Urg;
