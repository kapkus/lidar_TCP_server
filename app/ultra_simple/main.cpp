#include <iostream>
#include <string>
#include <thread>
#include <cstring>
#include <rplidar.h>
#include <signal.h>
#include <vector>
#include <sstream>
#include <json.hpp>

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif


using namespace sl;
using json = nlohmann::json;

bool ctrl_c_pressed = false;

void signalHandler(int);
void runServer(ILidarDriver* drv);
void handleClient(int clientSocket, ILidarDriver* drv);
void processCommand(const std::string& command, int clientSocket, ILidarDriver* drv);
void sendResponse(int clientSocket, const std::string& response);
bool checkLidarHealth(ILidarDriver* drv);
void sendJsonResponse(int clientSocket, const std::string& command, const json& response);

void cleanupSocket(int socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

void showUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <baudrate> <port_path>\n"
        << "Example: " << programName << " 256000 COM3\n";
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    if (argc != 3) {
        std::cerr << "Error: Invalid arguments.\n";
        showUsage(argv[0]);
        return 1;
    }

    const char* port = argv[2];
    sl_u32 baudrate;
    try {
        baudrate = std::stoul(argv[1]);
    }
    catch (...) {
        std::cerr << "Error: Invalid baudrate value.\n";
        showUsage(argv[0]);
        return 1;
    }

    ILidarDriver* drv = *createLidarDriver();
    if (!drv) {
        std::cerr << "Failed to create LIDAR driver instance\n";
        return 1;
    }
    

    IChannel* channel = *createSerialPortChannel(port, baudrate);
    if (SL_IS_FAIL(drv->connect(channel))) {
        std::cerr << "Failed to connect to LIDAR on " << port << "\n";
        delete drv;
        return 1;
    }

    if (!checkLidarHealth(drv)) {
        delete drv;
        return 1;
    }

    runServer(drv);

    drv->stop();
    drv->setMotorSpeed(0);
    delete drv;

#ifdef _WIN32
    WSACleanup();
#endif

    return 0;
}

void runServer(ILidarDriver* drv) {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        std::cerr << "Failed to create socket\n";
        return;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(4000);

    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Failed to bind socket\n";
        cleanupSocket(serverSocket);
        return;
    }

    if (listen(serverSocket, 5) < 0) {
        std::cerr << "Failed to listen on socket\n";
        cleanupSocket(serverSocket);
        return;
    }

    std::cout << "Server listening on port 4000...\n";

    while (!ctrl_c_pressed) {
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);

        if (clientSocket < 0) {
            std::cerr << "Failed to accept connection\n";
            continue;
        }

        std::cout << "Client connected\n";
        std::thread clientThread(handleClient, clientSocket, drv);
        clientThread.detach();
    }

    cleanupSocket(serverSocket);
}

void handleClient(int clientSocket, ILidarDriver* drv) {
    char buffer[1024];
    while (!ctrl_c_pressed) {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        if (bytesRead <= 0) {
            std::cout << "Client disconnected\n";
            break;
        }

        std::string command(buffer);
        processCommand(command, clientSocket, drv);
    }

    cleanupSocket(clientSocket);
}

void sendJsonResponse(int clientSocket, const std::string& command, const json& response) {
    json jsonResponse = {
        {"command", command},
        {"response", response}
    };
    std::string jsonString = jsonResponse.dump();
    jsonString += "\n";
    send(clientSocket, jsonString.c_str(), jsonString.size(), 0);
}

void processCommand(const std::string& command, int clientSocket, ILidarDriver* drv) {
    if (command == "START_SCAN") {
        std::cout << "Starting LIDAR scan...\n";

        drv->setMotorSpeed();
        if (SL_IS_OK(drv->startScan(false, true))) {
            sendJsonResponse(clientSocket, "START_SCAN", { {"status", "SCAN_STARTED"} });
        }
        else {
            sendJsonResponse(clientSocket, "START_SCAN", { {"status", "SCAN_FAILED"} });
        }
    }
    else if (command == "GET_SAMPLE") {
        std::cout << "Capturing full 360-degree rotation...\n";

        std::vector<sl_lidar_response_measurement_node_hq_t> nodes;
        nodes.reserve(8192);

        bool fullRotationCaptured = false;
        while (!ctrl_c_pressed && !fullRotationCaptured) {
            std::vector<sl_lidar_response_measurement_node_hq_t> tempNodes(8192);
            size_t count = tempNodes.size();

            if (SL_IS_OK(drv->grabScanDataHq(tempNodes.data(), count))) {
                drv->ascendScanData(tempNodes.data(), count);

                for (size_t i = 0; i < count; ++i) {
                    float angle = tempNodes[i].angle_z_q14 * 90.f / (1 << 14);

                    if (!nodes.empty() && angle < nodes.back().angle_z_q14 * 90.f / (1 << 14)) {
                        fullRotationCaptured = true;
                        break;
                    }

                    nodes.push_back(tempNodes[i]);
                }
            }
            else {
                std::cerr << "Failed to grab scan data\n";
                break;
            }
        }

        if (fullRotationCaptured) {
            json responseJson = json::array();
            for (const auto& node : nodes) {
                responseJson.push_back({
                    {"angle", node.angle_z_q14 * 90.f / (1 << 14)},
                    {"distance", node.dist_mm_q2 / 4.0f},
                    {"quality", node.quality}
                    });
            }
            sendJsonResponse(clientSocket, "GET_SAMPLE", responseJson);
        }
        else {
            sendJsonResponse(clientSocket, "GET_SAMPLE", { {"status", "ROTATION_CAPTURE_FAILED"} });
        }
    }
    else if (command == "STOP") {
        drv->stop();
        drv->setMotorSpeed(0);
        //ctrl_c_pressed = true;
        sendJsonResponse(clientSocket, "STOP", { {"status", "SCAN_STOPPED"} });
    }
    else if (command == "GET_INFO") {
        sl_lidar_response_device_info_t info;
        if (SL_IS_OK(drv->getDeviceInfo(info))) {
            json responseJson = {
                {"model", info.model},
                {"firmware_version", {{"major", info.firmware_version >> 8}, {"minor", info.firmware_version & 0xFF}}},
                {"hardware_version", info.hardware_version}
            };
            sendJsonResponse(clientSocket, "GET_INFO", responseJson);
        }
        else {
            sendJsonResponse(clientSocket, "GET_INFO", { {"status", "INFO_FAILED"} });
        }
    }
    else if (command == "GET_HEALTH") {
        sl_lidar_response_device_health_t health;
        if (SL_IS_OK(drv->getHealth(health))) {
            unsigned short errorCode = health.error_code;

            json responseJson = {
                {"status", (health.status == SL_LIDAR_STATUS_OK ? "HEALTH_OK" : "HEALTH_ERROR")},
                {"error_code", errorCode}
            };
            sendJsonResponse(clientSocket, "GET_HEALTH", responseJson);
        }
        else {
            sendJsonResponse(clientSocket, "GET_HEALTH", { {"status", "HEALTH_FAILED"} });
        }
    }
    else {
        sendJsonResponse(clientSocket, command, { {"status", "UNKNOWN_COMMAND"} });
    }
}

void sendResponse(int clientSocket, const std::string& response) {
    send(clientSocket, response.c_str(), response.size(), 0);
}

bool checkLidarHealth(ILidarDriver* drv) {
    sl_lidar_response_device_health_t health;
    if (SL_IS_OK(drv->getHealth(health))) {
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            std::cerr << "LIDAR internal error detected\n";
            return false;
        }
        return true;
    }
    else {
        std::cerr << "Failed to retrieve LIDAR health\n";
        return false;
    }
}

void signalHandler(int) {
    ctrl_c_pressed = true;
}
