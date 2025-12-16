#include "can_simple.hpp"

#include "Task/z_main.h"

void CANSimple::handle_can_message(const can_Message_t& msg) {
    // Handle optical flow odometry message
    if (msg.id == MSG_OPTICAL_FLOW_ODOM && msg.len == 8) {
        // Call the registered callback for optical flow
        on_optflow_rx(nullptr, msg);
    }
}