#ifndef __CAN_SIMPLE_HPP
#define __CAN_SIMPLE_HPP

#include "canbus.hpp"

class CANSimple {
public:
    enum {
        MSG_OPTICAL_FLOW_ODOM = 0x300,
    };

    CANSimple(CanBusBase* can_bus) : canbus_(can_bus) {}

private:
    void handle_can_message(const can_Message_t& msg);

    CanBusBase* canbus_;
    CanBusBase::CanSubscription* subscription_handle_ = nullptr;
};

#endif  // __CAN_SIMPLE_HPP