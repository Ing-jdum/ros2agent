#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>

class TCPServer {
private:
    int server_fd;
    sockaddr_in address;
    int client_socket;

public:
    TCPServer(int port);
    void acceptClient();
    char * receiveMessage();
    void sendMessage(const std::string& message);
    ~TCPServer();
};

#endif
