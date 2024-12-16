#include "tcp_server.h"
#include <iostream>
#include <cstring>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

void TcpServer::cleanupSocket(int socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

TcpServer::TcpServer(int port, Lidar& lidar)
    : serverSocket(-1), port(port), running(false), lidar(lidar) {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Failed to initialize Winsock");
    }
#endif
}

TcpServer::~TcpServer() {
    stop();
#ifdef _WIN32
    WSACleanup(); 
#endif
}

void TcpServer::start() {
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        std::cerr << "Failed to create socket\n";
        return;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

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

    running = true;
    std::cout << "Server listening on port " << port << "...\n";

    while (running) {
        sockaddr_in clientAddr{};
#ifdef _WIN32
        int clientLen = sizeof(clientAddr);
#else
        socklen_t clientLen = sizeof(clientAddr);
#endif
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);

        if (clientSocket < 0) {
            if (running) std::cerr << "Failed to accept connection\n";
            continue;
        }

        std::cout << "Client connected\n";
        std::thread(&TcpServer::handleClient, this, clientSocket).detach();
    }
}

void TcpServer::stop() {
    running = false;
    if (serverSocket >= 0) {
        cleanupSocket(serverSocket);
    }
}

void TcpServer::handleClient(int clientSocket) {
    if (!lidar.getDriver() || !lidar.checkHealth()) {
        sendJsonResponse(clientSocket, "CONNECT", {
            {"status", "LIDAR_ERROR"},
            {"message", "LIDAR is unavailable or unhealthy"}
            });
        cleanupSocket(clientSocket);
        return;
    }

    sendJsonResponse(clientSocket, "CONNECT", {
        {"status", "OK"},
        {"message", "LIDAR connection established"}
        });

    std::string buffer;
    char chunk[1024];

    while (true) {
        memset(chunk, 0, sizeof(chunk));
        int bytesRead = recv(clientSocket, chunk, sizeof(chunk) - 1, 0);
        if (bytesRead <= 0) break;

        buffer.append(chunk, bytesRead);

        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string command = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);

            if (command == "START_SCAN") {
                handleStartScan(clientSocket);
            }
            else if (command == "GET_SAMPLE") {
                handleGetSample(clientSocket);
            }
            else if (command == "STOP") {
                handleStopScan(clientSocket);
            }
            else {
                sendJsonResponse(clientSocket, "ERROR", { {"message", "Unknown command"} });
            }
        }
    }

    cleanupSocket(clientSocket);
}

void TcpServer::handleStartScan(int clientSocket) {
    auto driver = lidar.getDriver();
    if (driver) {
        driver->setMotorSpeed(600);
        if (SL_IS_OK(driver->startScan(false, true))) {
            sendJsonResponse(clientSocket, "START_SCAN", {
                {"status", "OK"},
                {"message", "LiDAR scan started"}
                });
            return;
        }
    }
    sendJsonResponse(clientSocket, "START_SCAN", {
        {"status", "FAILED"},
        {"message", "Failed to start LiDAR scan"}
        });
}

void TcpServer::handleGetSample(int clientSocket) {
    if (!lidar.getDriver()) {
        sendJsonResponse(clientSocket, "GET_SAMPLE", {
            {"status", "FAILED"},
            {"message", "LiDAR driver unavailable"}
        });
        return;
    }

    auto driver = lidar.getDriver();
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = _countof(nodes);

    if (SL_IS_OK(driver->grabScanDataHq(nodes, nodeCount))) {
        driver->ascendScanData(nodes, nodeCount);

        nlohmann::json sampleJson = nlohmann::json::array();
        for (size_t i = 0; i < nodeCount; ++i) {
            int rawQuality = nodes[i].quality;
            int adjustedQuality = rawQuality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

            std::cout << "{ angle: " << nodes[i].angle_z_q14 * 90.f / (1 << 14)
                      << ", distance: " << nodes[i].dist_mm_q2 / 4.0
                      << ", raw quality: " << rawQuality
                      << ", adjusted quality: " << adjustedQuality
                      << " }" << std::endl;

            sampleJson.push_back({
                {"angle", nodes[i].angle_z_q14 * 90.f / (1 << 14)},
                {"distance", nodes[i].dist_mm_q2 / 4.0},
                {"quality", adjustedQuality}
            });
        }

        sendJsonResponse(clientSocket, "GET_SAMPLE", {
            {"status", "OK"},
            {"data", sampleJson}
        });
    } else {
        sendJsonResponse(clientSocket, "GET_SAMPLE", {
            {"status", "FAILED"},
            {"message", "Failed to retrieve LiDAR sample"}
        });
    }
}



//void TcpServer::handleGetSample(int clientSocket) {
//    if (!lidar.getDriver()) {
//        sendJsonResponse(clientSocket, "GET_SAMPLE", {
//            {"status", "FAILED"},
//            {"message", "LiDAR driver unavailable"}
//            });
//        return;
//    }
//
//    auto driver = lidar.getDriver();
//    sl_lidar_response_measurement_node_hq_t nodes[8192];
//    size_t nodeCount = _countof(nodes);
//
//    if (SL_IS_OK(driver->grabScanDataHq(nodes, nodeCount))) {
//        nlohmann::json sampleJson = nlohmann::json::array();
//        for (size_t i = 0; i < nodeCount; ++i) {
//            sampleJson.push_back({
//                {"angle", nodes[i].angle_z_q14 * 90.f / (1 << 14)},
//                {"distance", nodes[i].dist_mm_q2 / 4.0},
//                {"quality", nodes[i].quality}
//                });
//        }
//
//        sendJsonResponse(clientSocket, "GET_SAMPLE", {
//            {"status", "OK"},
//            {"data", sampleJson}
//            });
//    }
//    else {
//        sendJsonResponse(clientSocket, "GET_SAMPLE", {
//            {"status", "FAILED"},
//            {"message", "Failed to retrieve LiDAR sample"}
//            });
//    }
//}

void TcpServer::handleStopScan(int clientSocket) {
    auto driver = lidar.getDriver();
    if (driver) {
        driver->stop();
        driver->setMotorSpeed(0);
        sendJsonResponse(clientSocket, "STOP", {
            {"status", "OK"},
            {"message", "LiDAR scan stopped"}
            });
        return;
    }
    sendJsonResponse(clientSocket, "STOP", {
        {"status", "FAILED"},
        {"message", "Failed to stop LiDAR scan"}
        });
}

void TcpServer::sendJsonResponse(int clientSocket, const std::string& command, const nlohmann::json& response) {
    nlohmann::json jsonResponse = {
        {"command", command},
        {"response", response}
    };
    std::string jsonString = jsonResponse.dump() + "\n";
    send(clientSocket, jsonString.c_str(), jsonString.size(), 0);
}


