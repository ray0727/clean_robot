#include <stdio.h>
#include <thread>
#include <sys/socket.h>


Class MySocket{
    public:
        std::thread t1;
        int newSocket;
        struct sockaddr_in serverAddr;
        struct sockaddr_storage serverStorage;

    Mysocket();

};

MySocket::Mysocket(){

    welcomeSocket = socket(PF_INET, SOCK_STREAM, 0);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(7891);
    //serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serverAddr.sin_addr.s_addr = inet_addr("192.168.89.5");
    
    
}