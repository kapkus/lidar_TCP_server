#include "lidar.h"
#include <iostream>

Lidar::Lidar() : drv(nullptr), isHealthy(false) {}

Lidar::~Lidar() {
    shutdown();
}

bool Lidar::initialize(const std::string& port, sl_u32 baudrate) {
    drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "Failed to create LIDAR driver instance\n";
        return false;
    }

    IChannel* channel = *createSerialPortChannel(port.c_str(), baudrate);
    if (SL_IS_FAIL(drv->connect(channel))) {
        std::cerr << "Failed to connect to LIDAR on port " << port << "\n";
        delete drv;
        drv = nullptr;
        return false;
    }

    isHealthy = checkHealth();
    return isHealthy;
}

bool Lidar::checkHealth() {
    if (!drv) {
        std::cerr << "LIDAR driver is null\n";
        return false;
    }

    sl_lidar_response_device_health_t health;
    if (SL_IS_OK(drv->getHealth(health))) {
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            std::cerr << "LIDAR internal error detected, error code: " << health.error_code << "\n";
            return false;
        }
        return true;
    }
    else {
        std::cerr << "Failed to retrieve LIDAR health\n";
        return false;
    }
}

void Lidar::shutdown() {
    if (drv) {
        drv->stop();
        drv->setMotorSpeed(0);
        delete drv;
        drv = nullptr;
    }
}

ILidarDriver* Lidar::getDriver() {
    return drv;
}
