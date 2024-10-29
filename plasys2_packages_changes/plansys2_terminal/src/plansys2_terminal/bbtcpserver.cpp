#include "plansys2_terminal/TCPServer.h"

TCPServer::TCPServer(int port) {
    // Create socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

     // Set socket options to allow reusing address
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = INADDR_ANY;

    // bind server to port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // listen for incoming calls
    if (listen(server_fd, SOMAXCONN) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
}

char* TCPServer::receiveMessage() {
    char buffer[4096] = {0};
    // wait for message
    int bytes_received = recv(client_socket, buffer, sizeof(buffer), 0);
    if (bytes_received < 0) {
        std::cerr << "There was a connection issue" << std::endl;
        return nullptr; // Return nullptr to indicate failure
    }
    if (bytes_received == 0) {
        std::cout << "Client disconnected" << std::endl;
        return nullptr; // Return nullptr to indicate client disconnection
    }

    // Create a dynamic buffer and copy the received data
    char* message = new char[bytes_received + 1];
    memcpy(message, buffer, bytes_received);
    message[bytes_received] = '\0'; // Null-terminate the string

    return message;
}


void TCPServer::sendMessage(const std::string& message) {
    send(client_socket, message.c_str(), message.size(), 0);
}

void TCPServer::acceptClient() {
    sockaddr_in client;
    socklen_t clientSize = sizeof(client);

    if ((client_socket = accept(server_fd, (struct sockaddr *)&client, &clientSize))<0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    // Get client information
    char host[NI_MAXHOST];
    char svc[NI_MAXSERV];
    getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, svc, NI_MAXSERV, 0);
    std::cout << "Connected to " << host << " on port " << svc << std::endl;
}

TCPServer::~TCPServer() {
    close(server_fd);
    close(client_socket); // Close client socket in destructor
}
