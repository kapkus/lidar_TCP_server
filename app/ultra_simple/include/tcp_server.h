#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <string>
#include <thread>
#include <vector>
#include <json.hpp>
#include "lidar.h"

class TcpServer {
public:
    TcpServer(int port, Lidar& lidar);
    ~TcpServer();

    void start();
    void stop();

private:
    void handleClient(int clientSocket);
    void handleStartScan(int clientSocket);
    void handleGetSample(int clientSocket);
    void handleStopScan(int clientSocket);
    void sendJsonResponse(int clientSocket, const std::string& command, const nlohmann::json& response);
    void cleanupSocket(int socket);

    int serverSocket;
    int port;
    bool running;
    Lidar& lidar;
};

#endif // TCP_SERVER_H
