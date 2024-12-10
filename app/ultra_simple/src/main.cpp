#include "lidar.h"
#include "tcp_server.h"
#include <csignal>
#include <iostream>
#include <thread>
#include <atomic>

std::atomic<bool> ctrl_c_pressed(false);

void signalHandler(int) {
    ctrl_c_pressed = true;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <baudrate> <port_path>\n";
        return 1;
    }

    std::string portPath = argv[2];
    sl_u32 baudrate = std::stoul(argv[1]);

    Lidar lidar;
    std::atomic<bool> isLidarAvailable(false);

    // Background thread to retry LIDAR initialization
    std::thread lidarInitThread([&]() {
        while (!ctrl_c_pressed) {
            if (!lidar.getDriver()) {
                std::cout << "Attempting to initialize LIDAR...\n";
                if (lidar.initialize(portPath, baudrate)) {
                    isLidarAvailable = true;
                    std::cout << "LIDAR initialized successfully\n";
                }
                else {
                    isLidarAvailable = false;
                    std::cerr << "Failed to initialize LIDAR\n";
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        });

    TcpServer server(8002, lidar);

    std::thread serverThread([&]() {
        server.start();
        });

    // Wait for Ctrl+C
    while (!ctrl_c_pressed) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Shutting down...\n";
    server.stop();
    if (serverThread.joinable()) serverThread.join();
    if (lidarInitThread.joinable()) lidarInitThread.join();

    return 0;
}
