#ifndef __INTERFACES_HPP
#define __INTERFACES_HPP

#pragma GCC push_options
#pragma GCC optimize ("s")

class RobotIntf {
public:
    class CanIntf {
    public:
        enum Error {
            ERROR_NONE = 0,
        };
        enum Protocol {
            PROTOCOL_SIMPLE = 1 << 0,
        };
    };
};

#endif // __INTERFACES_HPP
