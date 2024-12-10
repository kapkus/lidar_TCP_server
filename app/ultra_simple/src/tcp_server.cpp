#include "tcp_server.h"
#include <iostream>
#include <cstring>
#include <thread>

// Platform-specific includes
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib") // Link against Winsock library
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

// Cross-platform cleanup function
void TcpServer::cleanupSocket(int socket) {
#ifdef _WIN32
    closesocket(socket);
#else
    close(socket);
#endif
}

// Constructor
TcpServer::TcpServer(int port, Lidar& lidar)
    : serverSocket(-1), port(port), running(false), lidar(lidar) {
#ifdef _WIN32
    // Initialize Winsock on Windows
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("Failed to initialize Winsock");
    }
#endif
}

// Destructor
TcpServer::~TcpServer() {
    stop();
#ifdef _WIN32
    WSACleanup(); // Cleanup Winsock on Windows
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

    char buffer[1024];
    while (true) {
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        if (bytesRead <= 0) break;

        std::string command(buffer);
        // Process the command here (e.g., START_SCAN, GET_SAMPLE, etc.)
    }

    cleanupSocket(clientSocket);
}


void TcpServer::sendJsonResponse(int clientSocket, const std::string& command, const nlohmann::json& response) {
    nlohmann::json jsonResponse = {
        {"command", command},
        {"response", response}
    };
    std::string jsonString = jsonResponse.dump() + "\n";
    send(clientSocket, jsonString.c_str(), jsonString.size(), 0);
}
