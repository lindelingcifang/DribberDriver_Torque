#include "can.h"

#include "Task/z_main.h"

void init_communication() {
    // Initialize communication interfaces here
    can1_bus.start(&hcan1);
    can2_bus.start(&hcan2);
}