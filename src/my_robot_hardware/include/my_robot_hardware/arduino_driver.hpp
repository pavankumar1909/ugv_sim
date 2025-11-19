#ifndef ARDUINO_DRIVER_HPP
#define ARDUINO_DRIVER_HPP

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <cstring>
#include <cmath>
#include <algorithm>

class ArduinoDriver {
public:
    explicit ArduinoDriver(const std::string& device_name)
        : device_name_(device_name), serial_port_(-1), max_speed_(255) {}

    int init()
    {
        serial_port_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_port_ < 0) {
            std::cerr << "❌ Failed to open serial port: " << device_name_ << std::endl;
            return -1;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(serial_port_, &tty) != 0) {
            std::cerr << "❌ Error getting serial port attributes." << std::endl;
            close(serial_port_);
            return -1;
        }

        cfsetospeed(&tty, B9600);
        cfsetispeed(&tty, B9600);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
            std::cerr << "❌ Error setting serial port attributes." << std::endl;
            close(serial_port_);
            return -1;
        }

        std::cout << "✅ Serial port initialized on " << device_name_ << std::endl;
        return 0;
    }

    void activate() { /* No-op for Arduino */ }
    void deactivate() {
        if (serial_port_ >= 0) close(serial_port_);
        serial_port_ = -1;
    }

    void setTargetVelocity(double left_vel, double right_vel)
    {
        if (serial_port_ < 0) return;

        // Convert to 0–255 speed range
        int left_speed = static_cast<int>(std::round(std::clamp(left_vel * max_speed_, -255.0, 255.0)));
        int right_speed = static_cast<int>(std::round(std::clamp(right_vel * max_speed_, -255.0, 255.0)));

        std::string cmd = buildMotorCommand(left_speed, right_speed);
        ssize_t written = ::write(serial_port_, cmd.c_str(), cmd.size());
        if (written != static_cast<ssize_t>(cmd.size())) {
            std::cerr << "⚠️ Warning: Incomplete serial write (" << written << "/" << cmd.size() << " bytes)" << std::endl;
        }
    }

private:
    std::string buildMotorCommand(int left, int right)
    {
      //stop both wheels
      if(left ==0 && right == 0)
      {
           return "S0\n";
      }
      
      // forward
      if(left > 0 && right > 0)
      {
           return "F" + std::to_string((left+right)/2)+"\n";
      }

      // backward
      if(left < 0 && right < 0)
      {
           return "B"+ std::to_string((-left-right)/2)+"\n";
      }
      
      // turn left-- right wheel forward,left wheel backward
      if(left < 0 && right > 0)
      {
           return "L"+ std::to_string((abs(left)+right)/2)+"\n";
      }

      // turn right -- left wheel forward,right wheel backward
      if(left > 0 && right < 0)
      {
           return "R"+ std::to_string((left+abs(right))/2)+"\n";
      }

       // turn back left 
       if(left < 0 && right < 0 && abs(left) < abs(right))
      {
           return "BL\n"+ std::to_string(abs(right))+"\n";
      }

       // turn back right 
       if(left < 0 && right < 0 && abs(left) > abs(right))
      {
           return "BR\n"+ std::to_string(abs(left))+"\n";
      }
    
      //default
       
      return "S0\n";
      
      


    }

    std::string device_name_;
    int serial_port_;
    const int max_speed_;
};

#endif

