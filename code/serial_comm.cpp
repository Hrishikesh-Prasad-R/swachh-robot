#include "serial_comm.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>

// Global serial instance definition
SerialComm g_serial;

SerialComm::SerialComm() : serial_port(-1), connected(false) {}

SerialComm::~SerialComm() {
    disconnect();
}

bool SerialComm::connect(const char* port) {
    serial_port = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (serial_port < 0) {
        std::cerr << "Error opening serial port: " << port << std::endl;
        return false;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error getting serial attributes" << std::endl;
        return false;
    }
    
    // Configure serial port (115200 baud, 8N1)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;
    
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;
    
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes" << std::endl;
        return false;
    }
    
    connected = true;
    std::cout << "Connected to serial port: " << port << std::endl;
    return true;
}

bool SerialComm::send_command(const std::string& command) {
    if (!connected || serial_port < 0) {
        std::cerr << "Serial port not connected" << std::endl;
        return false;
    }
    
    std::string cmd = command + "\n";
    ssize_t bytes_written = write(serial_port, cmd.c_str(), cmd.length());
    
    if (bytes_written < 0) {
        std::cerr << "Error writing to serial port" << std::endl;
        return false;
    }
    
    std::cout << "Sent command: " << command << std::endl;
    return true;
}

void SerialComm::disconnect() {
    if (serial_port >= 0) {
        close(serial_port);
        serial_port = -1;
        connected = false;
    }
}
