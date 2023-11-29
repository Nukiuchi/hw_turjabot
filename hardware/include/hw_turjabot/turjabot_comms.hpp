#ifndef HW_TURJABOT_TURJABOT_COMMS_HPP
#define HW_TURJABOT_TURJABOT_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <iostream>



class TurjabotComms
{

public:

  TurjabotComms() = default;

  void read_encoder_values(int &val_1, int &val_2)
  {
    //std::string response = send_msg("e\r");
    std::string response = "";

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }
  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    //send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    //send_msg(ss.str());
  }

private:
    int timeout_ms_;
};

#endif // HW_TURJABOT_TURJABOT_COMMS_HPP