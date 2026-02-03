#ifndef SERIAL_COMM_HPP
#define SERIAL_COMM_HPP

#include <string>

// Serial communication class for Arduino
class SerialComm {
private:
    int serial_port;
    bool connected;
    
public:
    SerialComm();
    ~SerialComm();
    
    bool connect(const char* port = "/dev/ttyCH341USB0");
    bool send_command(const std::string& command);
    void disconnect();
};

// Global serial communication instance
extern SerialComm g_serial;

#endif // SERIAL_COMM_HPP
