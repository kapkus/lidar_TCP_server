#ifndef LIDAR_H
#define LIDAR_H

#include <rplidar.h>
#include <thread>
#include <string>
#include <chrono>

using namespace sl;

class Lidar {
public:
    Lidar();
    ~Lidar();

    bool initialize(const std::string& port, sl_u32 baudrate);
    bool checkHealth();
    void shutdown();

    ILidarDriver* getDriver();

private:
    ILidarDriver* drv;
    bool isHealthy;
};

#endif // LIDAR_H
